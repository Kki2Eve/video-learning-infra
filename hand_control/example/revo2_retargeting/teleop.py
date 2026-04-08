import argparse
import asyncio
import json
import time
from multiprocessing import Process, Queue
from pathlib import Path

import cv2
import numpy as np
import requests
import uvicorn
from fastapi import FastAPI
from fastapi.responses import StreamingResponse

from dex_retargeting.constants import (
    HandType,
    RetargetingType,
    RobotName,
    get_default_config_path,
)
from dex_retargeting.retargeting_config import RetargetingConfig
import pyrealsense2 as rs
from single_hand_detector import SingleHandDetector

MOTOR_MIN = 0
MOTOR_MAX = 100

# [Thumb_Meta, Thumb_Prox, Index, Middle, Ring, Pinky]
JOINT_LIMITS_RAD = [1.57, 1.03, 1.41, 1.41, 1.41, 1.41]

app = FastAPI()
shared_queue = None
shared_qpos_queue = None
HAND_CONTROL_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = HAND_CONTROL_ROOT.parent
DEFAULT_URDF_DIR = REPO_ROOT / "assets" / "robots" / "hands"


def set_global_queue(mapped_q, qpos_q=None):
    global shared_queue
    global shared_qpos_queue
    shared_queue = mapped_q
    shared_qpos_queue = qpos_q


def map_joint(rad_val, max_rad):
    clamped_rad = min(max(0.0, float(rad_val)), max_rad)
    if max_rad < 1e-3:
        return 0
    motor_val = (clamped_rad / max_rad) * (MOTOR_MAX - MOTOR_MIN) + MOTOR_MIN
    return int(motor_val)


def mjpeg_stream(url):
    resp = requests.get(url, stream=True, timeout=(3, None))
    resp.raise_for_status()
    bytes_data = b""
    for chunk in resp.iter_content(chunk_size=1024):
        bytes_data += chunk
        a = bytes_data.find(b"\xff\xd8")
        b = bytes_data.find(b"\xff\xd9")
        if a != -1 and b != -1:
            jpg = bytes_data[a : b + 2]
            bytes_data = bytes_data[b + 2 :]
            img_array = np.frombuffer(jpg, dtype=np.uint8)
            frame = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
            if frame is not None:
                yield frame


def opencv_stream(source):
    cap = cv2.VideoCapture(source)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video source: {source}")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            yield frame
    finally:
        cap.release()


def realsense_stream(width=640, height=480, fps=30):
    if rs is None:
        raise RuntimeError("pyrealsense2 is not installed")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    pipeline.start(config)
    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
            yield frame
    finally:
        pipeline.stop()


def frame_stream(source):
    if source.lower() in {"realsense", "rs"}:
        yield from realsense_stream()
        return

    if source.startswith("http://") or source.startswith("https://"):
        yield from mjpeg_stream(source)
        return

    source_value = int(source) if source.isdigit() else source
    yield from opencv_stream(source_value)


def iter_mapped_joints(
    source,
    config_path,
    hand_type,
    scaling_factor,
    show_window=True,
    return_debug=False,
):
    for item in iter_retarget_outputs(
        source=source,
        config_path=config_path,
        hand_type=hand_type,
        scaling_factor=scaling_factor,
        show_window=show_window,
        return_debug=return_debug,
    ):
        if return_debug:
            mapped, _full_qpos, vis = item
            yield mapped, vis
        else:
            mapped, _full_qpos = item
            yield mapped


def iter_retarget_outputs(
    source,
    config_path,
    hand_type,
    scaling_factor,
    show_window=True,
    return_debug=False,
):
    detector = SingleHandDetector(hand_type=hand_type, selfie=False)
    if DEFAULT_URDF_DIR.exists():
        RetargetingConfig.set_default_urdf_dir(str(DEFAULT_URDF_DIR))
    retargeting = RetargetingConfig.load_from_file(
        config_path, override={"scaling_factor": scaling_factor}
    ).build()
    indices = retargeting.optimizer.target_link_human_indices
    active_indices = retargeting.optimizer.idx_pin2target.astype(int)

    try:
        for frame in frame_stream(source):
            rgb = frame[..., ::-1]
            num_box, joint_pos, keypoint_2d, _ = detector.detect(rgb)

            vis = None
            if show_window or return_debug:
                if keypoint_2d is not None:
                    vis = detector.draw_skeleton_on_image(frame.copy(), keypoint_2d, style="default")
                else:
                    vis = frame.copy()
            if show_window:
                cv2.imshow("Hand Teleop", vis)

            if num_box > 0:
                origin_idx = indices[0, :]
                task_idx = indices[1, :]
                ref_value = joint_pos[task_idx, :] - joint_pos[origin_idx, :]
                full_qpos = retargeting.retarget(ref_value)
                active_qpos = full_qpos[active_indices]
                mapped = [
                    map_joint(
                        float(active_qpos[i]),
                        JOINT_LIMITS_RAD[i],
                    )
                    for i in range(len(active_qpos))
                ]
                if return_debug:
                    yield mapped, full_qpos, vis
                else:
                    yield mapped, full_qpos

            if show_window and (cv2.waitKey(1) & 0xFF == 27):
                break
    finally:
        if show_window:
            cv2.destroyAllWindows()


def hand_detect_and_publish(
    mapped_queue,
    qpos_queue,
    source,
    config_path,
    hand_type,
    scaling_factor,
    show_window=True,
):
    for mapped, full_qpos in iter_retarget_outputs(
        source=source,
        config_path=config_path,
        hand_type=hand_type,
        scaling_factor=scaling_factor,
        show_window=show_window,
    ):
        mapped_queue.put(mapped)
        qpos_queue.put(full_qpos.tolist())


@app.get("/hand_data")
def hand_data_stream():
    async def joint_stream():
        while True:
            try:
                if shared_queue is not None and not shared_queue.empty():
                    data = shared_queue.get()
                    yield json.dumps({"joints": data}) + "\n"
                else:
                    await asyncio.sleep(0.01)
            except Exception:
                await asyncio.sleep(0.01)

    return StreamingResponse(joint_stream(), media_type="application/json")


@app.get("/hand_qpos")
def hand_qpos_stream():
    async def qpos_stream():
        while True:
            try:
                if shared_qpos_queue is not None and not shared_qpos_queue.empty():
                    data = shared_qpos_queue.get()
                    yield json.dumps({"qpos": data}) + "\n"
                else:
                    await asyncio.sleep(0.01)
            except Exception:
                await asyncio.sleep(0.01)

    return StreamingResponse(qpos_stream(), media_type="application/json")


def parse_args():
    parser = argparse.ArgumentParser(description="Local hand teleop pipeline.")
    parser.add_argument(
        "--source",
        type=str,
        default="realsense",
        help="realsense, camera index (0/1), local video file, or MJPEG URL.",
    )
    parser.add_argument(
        "--robot_name",
        type=str,
        default="revo2",
        help="Used only when --config_path is not provided and no local revo2 config is found.",
    )
    parser.add_argument(
        "--retargeting_type",
        type=str,
        default="dexpilot",
        choices=[t.name for t in RetargetingType],
    )
    parser.add_argument("--hand_type", type=str, default="right", choices=[t.name for t in HandType])
    parser.add_argument("--scaling_factor", type=float, default=1.6)
    parser.add_argument("--config_path", type=str, default=None)
    parser.add_argument("--show_window", action="store_true", help="Show visualization window.")
    parser.add_argument(
        "--mode",
        type=str,
        default="print",
        choices=["print", "http"],
        help="print: local pipeline only; http: expose /hand_data stream.",
    )
    parser.add_argument("--host", type=str, default="0.0.0.0")
    parser.add_argument("--port", type=int, default=50001)
    return parser.parse_args()


def main():
    args = parse_args()
    retargeting_type = RetargetingType[args.retargeting_type]
    hand_type = HandType[args.hand_type]

    if args.config_path:
        config_path = Path(args.config_path)
    else:
        local_cfg = (
            HAND_CONTROL_ROOT
            / "configs"
            / "revo2"
            / f"revo2_hand_{hand_type.name}_dexpilot.yml"
        )
        if local_cfg.exists():
            config_path = local_cfg
        else:
            robot_key = args.robot_name.lower()
            try:
                robot_name = RobotName[robot_key]
            except KeyError as exc:
                valid_robot_names = ", ".join(RobotName.__members__.keys())
                raise ValueError(
                    f"Unknown robot_name '{args.robot_name}'. "
                    f"Valid values: {valid_robot_names}. "
                    "You can also pass --config_path explicitly."
                ) from exc
            config_path = get_default_config_path(robot_name, retargeting_type, hand_type)
    detector_hand_type = hand_type.name.capitalize()

    if args.mode == "print":
        for joints in iter_mapped_joints(
            source=args.source,
            config_path=str(config_path),
            hand_type=detector_hand_type,
            scaling_factor=args.scaling_factor,
            show_window=args.show_window,
        ):
            print(joints)
        return

    mapped_queue = Queue()
    qpos_queue = Queue()
    set_global_queue(mapped_queue, qpos_queue)
    p = Process(
        target=hand_detect_and_publish,
        args=(
            mapped_queue,
            qpos_queue,
            args.source,
            str(config_path),
            detector_hand_type,
            args.scaling_factor,
            args.show_window,
        ),
    )
    p.start()
    try:
        uvicorn.run(app, host=args.host, port=args.port)
    finally:
        p.join(timeout=1.0)


if __name__ == "__main__":
    main()
