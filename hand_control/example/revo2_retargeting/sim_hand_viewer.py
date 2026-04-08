import requests
import numpy as np
from pathlib import Path
import json
import queue
import threading
import time
import sapien
from sapien.asset import create_dome_envmap
from sapien.utils import Viewer
from dex_retargeting.retargeting_config import RetargetingConfig

QPOS_STREAM_URL = "your_qpos_stream_url"  # 替换为你的qpos流URL，例如 http://
# 配置机器人
robot_dir = Path(__file__).absolute().parent.parent.parent / "assets" / "robots" / "hands"
RetargetingConfig.set_default_urdf_dir(str(robot_dir))
config_path = "your_config.yaml"  # 替换为你的配置文件路径
config = RetargetingConfig.load_from_file(config_path)
retargeting = config.build()

# Make viewer/camera shader explicit to avoid platform-dependent black screen behavior
sapien.render.set_viewer_shader_dir("default")
sapien.render.set_camera_shader_dir("default")

scene = sapien.Scene()
render_mat = sapien.render.RenderMaterial()
render_mat.base_color = [0.06, 0.08, 0.12, 1]
render_mat.metallic = 0.0
render_mat.roughness = 0.9
render_mat.specular = 0.8
scene.add_ground(-0.2, render_material=render_mat, render_half_size=[1000, 1000])
scene.add_directional_light(np.array([1, 1, -1]), np.array([3, 3, 3]))
scene.add_point_light(np.array([2, 2, 2]), np.array([2, 2, 2]), shadow=False)
scene.add_point_light(np.array([2, -2, 2]), np.array([2, 2, 2]), shadow=False)
scene.set_environment_map(create_dome_envmap(sky_color=[0.2, 0.2, 0.2], ground_color=[0.2, 0.2, 0.2]))
scene.add_area_light_for_ray_tracing(sapien.Pose([2, 1, 2], [0.707, 0, 0.707, 0]), np.array([1, 1, 1]), 5, 5)

cam = scene.add_camera(name="Cheese!", width=600, height=600, fovy=1, near=0.1, far=10)
cam.set_local_pose(sapien.Pose([0.50, 0, 0.0], [0, 0, 0, -1]))

viewer = Viewer()
viewer.set_scene(scene)
viewer.control_window.show_origin_frame = False
viewer.control_window.move_speed = 0.01
viewer.control_window.toggle_camera_lines(False)
viewer.set_camera_pose(cam.get_local_pose())

loader = scene.create_urdf_loader()
filepath = Path(config.urdf_path)
robot_name = filepath.stem
loader.load_multiple_collisions_from_file = True
filepath = str(filepath)
robot = loader.load(filepath)
robot.set_pose(sapien.Pose([0, 0, -0.15]))

sapien_joint_names = [joint.get_name() for joint in robot.get_active_joints()]
retargeting_joint_names = retargeting.joint_names
retargeting_to_sapien = np.array([
    retargeting_joint_names.index(name) for name in sapien_joint_names
]).astype(int)


def parse_qpos_from_line(line: bytes) -> np.ndarray:
    text = line.decode("utf-8", errors="ignore").strip()
    if not text:
        raise ValueError("空消息")

    # 1) JSON object: {"qpos": [...]}
    # 2) JSON array: [....]
    # 3) CSV string: a,b,c,...
    if text.startswith("{") or text.startswith("["):
        payload = json.loads(text)
        if isinstance(payload, dict):
            if "qpos" not in payload:
                raise ValueError("JSON对象中缺少 qpos 字段")
            values = payload["qpos"]
        elif isinstance(payload, list):
            values = payload
        else:
            raise ValueError(f"不支持的 JSON 类型: {type(payload)}")
    else:
        values = [x for x in text.split(",") if x.strip()]

    qpos = np.asarray(values, dtype=np.float32).reshape(-1)
    return qpos


def stream_worker(
    qpos_queue: "queue.Queue[np.ndarray]",
    stop_event: threading.Event,
):
    while not stop_event.is_set():
        try:
            print(f"Connecting to {QPOS_STREAM_URL} ...")
            with requests.get(QPOS_STREAM_URL, stream=True, timeout=(3, 30)) as resp:
                resp.raise_for_status()
                for line in resp.iter_lines():
                    if stop_event.is_set():
                        break
                    if not line:
                        continue
                    try:
                        qpos = parse_qpos_from_line(line)
                    except Exception as e:
                        print(f"解析动作序列失败: {e}; 原始数据: {line[:120]!r}")
                        continue

                    # Keep only latest qpos to reduce latency
                    while not qpos_queue.empty():
                        try:
                            qpos_queue.get_nowait()
                        except queue.Empty:
                            break
                    qpos_queue.put_nowait(qpos)
        except Exception as e:
            print(f"流连接异常: {e}; 1秒后重连")
            time.sleep(1)


def main():
    qpos_queue: "queue.Queue[np.ndarray]" = queue.Queue(maxsize=4)
    stop_event = threading.Event()
    worker = threading.Thread(
        target=stream_worker, args=(qpos_queue, stop_event), daemon=True
    )
    worker.start()
    print(f"[viewer] sapien joints({len(sapien_joint_names)}): {sapien_joint_names}")
    print(f"[viewer] retargeting joints({len(retargeting_joint_names)}): {retargeting_joint_names}")
    expected_dim = len(retargeting_joint_names)

    try:
        while not viewer.closed:
            latest_qpos = None
            while True:
                try:
                    latest_qpos = qpos_queue.get_nowait()
                except queue.Empty:
                    break

            if latest_qpos is not None:
                if latest_qpos.shape[0] != expected_dim:
                    print(
                        f"收到长度 {latest_qpos.shape[0]}，但期望 {expected_dim}"
                    )
                else:
                    robot.set_qpos(latest_qpos[retargeting_to_sapien])

            scene.update_render()
            for _ in range(2):
                viewer.render()
    finally:
        stop_event.set()

if __name__ == "__main__":
    main()
