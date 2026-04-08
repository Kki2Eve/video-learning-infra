import argparse
import json
import time
import sys
from pathlib import Path
import threading
import queue as thread_queue

import requests
import yaml

HAND_CONTROL_ROOT = Path(__file__).resolve().parents[2]
REPO_ROOT = HAND_CONTROL_ROOT.parent
THIRD_PARTY_ROOT = REPO_ROOT / "third_party"
DEFAULT_ARM_CONFIG = HAND_CONTROL_ROOT / "configs" / "arm_connection.yaml"

for candidate in (HAND_CONTROL_ROOT, THIRD_PARTY_ROOT):
    if str(candidate) not in sys.path:
        sys.path.insert(0, str(candidate))

from dex_retargeting.constants import (
    HandType,
    RetargetingType,
    RobotName,
    get_default_config_path,
)
from teleop import iter_mapped_joints
from robotic_arm_package.robotic_arm import Arm, RM65


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.load(f, Loader=yaml.FullLoader)


class SimpleHand:
    def __init__(self, arm_side, robotic_left_arm_ip):
        self.baudrate = 460800
        self.device = 127
        if str(arm_side).strip().lower() == "left":
            self.arm = Arm(RM65, robotic_left_arm_ip)
            self.arm.Close_Modbus_Mode(port=1, block=True)
            self.arm.Set_Modbus_Mode(port=1, baudrate=self.baudrate, timeout=20, block=True)
            self.arm.Write_Single_Register(port=1, address=901, data=2, device=self.device, block=True)
        else:
            raise ValueError("Only left arm side is currently supported.")

    def finger_control(self, finger_data):
        if len(finger_data) != 6:
            raise ValueError("finger_data must contain 6 elements.")
        adjusted_data = [
            finger_data[0],
            finger_data[1],
            finger_data[3],
            finger_data[2],
            finger_data[5],
            finger_data[4],
        ]
        self.arm.Write_Registers(
            port=1, address=1070, num=3, single_data=adjusted_data, device=self.device, block=True
        )


def iter_joint_stream_http(url):
    while True:
        try:
            for line in requests.get(url, stream=True).iter_lines():
                if not line:
                    continue
                s = line.decode().strip()
                try:
                    data = json.loads(s)
                except json.JSONDecodeError:
                    print(f"Invalid json line: {s}")
                    continue
                values = data.get("joints")
                if (
                    not isinstance(values, list)
                    or len(values) != 6
                    or not all(isinstance(v, (int, float)) and 0 <= v <= 100 for v in values)
                ):
                    print(f"Invalid joints payload: {data}")
                    continue
                yield [int(v) for v in values]
        except Exception as e:
            print(f"HTTP stream error: {e}, retrying in 2 seconds...")
            time.sleep(2)


def load_arm_connection(path):
    raw = load_yaml(path) or {}
    if "shared_arm_connection_config" in raw:
        shared_path = (Path(path).parent / str(raw["shared_arm_connection_config"])).resolve()
        return load_arm_connection(shared_path)

    if "realman" in raw:
        left = raw.get("realman", {}).get("left", {})
        left_arm_side = left.get("side", "left")
        robotic_left_arm_ip = left["ip"]
        return str(left_arm_side), str(robotic_left_arm_ip)

    left_arm_side = raw.get("LeftArmSide", raw.get("LeftHandSide", "Left"))
    robotic_left_arm_ip = raw["RoboticLeftArmIP"]
    return str(left_arm_side), str(robotic_left_arm_ip)


def create_hand_from_arm_config(arm_config_path):
    left_arm_side, robotic_left_arm_ip = load_arm_connection(arm_config_path)
    return SimpleHand(left_arm_side, robotic_left_arm_ip)


def resolve_local_config_path(args):
    retargeting_type = RetargetingType[args.retargeting_type]
    hand_type = HandType[args.hand_type]

    if args.config_path:
        return Path(args.config_path), hand_type

    local_cfg = (
        HAND_CONTROL_ROOT
        / "configs"
        / "revo2"
        / f"revo2_hand_{hand_type.name}_dexpilot.yml"
    )
    if local_cfg.exists():
        return local_cfg, hand_type

    # Fallback to dex_retargeting defaults when local config is missing.
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
    return Path(get_default_config_path(robot_name, retargeting_type, hand_type)), hand_type


def parse_args():
    parser = argparse.ArgumentParser(
        description="Brainco hand controller with local-only teleoperation mode."
    )
    parser.add_argument(
        "--arm-config",
        type=str,
        default=str(DEFAULT_ARM_CONFIG),
        help="Path to shared arm connection config. Preferred over --settings.",
    )
    parser.add_argument(
        "--input_mode",
        type=str,
        default="local",
        choices=["local", "http"],
        help="local: camera->retarget->hand in one process; http: keep old remote stream.",
    )
    parser.add_argument("--stream_url", type=str, default="http://127.0.0.1:50001/hand_data")
    parser.add_argument(
        "--source",
        type=str,
        default="realsense",
        help="Used in local mode: realsense, camera index (0/1), file path, or MJPEG URL.",
    )
    parser.add_argument("--robot_name", type=str, default="revo2")
    parser.add_argument(
        "--retargeting_type",
        type=str,
        default="dexpilot",
        choices=[t.name for t in RetargetingType],
    )
    parser.add_argument("--hand_type", type=str, default="right", choices=[t.name for t in HandType])
    parser.add_argument("--scaling_factor", type=float, default=1.0)
    parser.add_argument("--config_path", type=str, default=None)
    parser.add_argument("--show_window", action="store_true")
    parser.add_argument("--control_hz", type=float, default=30, help="Hand command send rate.")
    parser.add_argument(
        "--deadband",
        type=int,
        default=5,
        help="Do not send command when all joint changes are <= deadband.",
    )
    parser.add_argument(
        "--debug_joints",
        action="store_true",
        help="Print converted joint commands (0-100) in real time.",
    )
    parser.add_argument("--verbose", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()

    if args.input_mode == "local":
        config_path, hand_type = resolve_local_config_path(args)

        detector_hand_type = hand_type.name.capitalize()

        if args.debug_joints:
            print(
                f"Debug mode started: source={args.source}, hand_type={detector_hand_type}, "
                f"config={config_path}"
            )
            for values in iter_mapped_joints(
                source=args.source,
                config_path=str(config_path),
                hand_type=detector_hand_type,
                scaling_factor=args.scaling_factor,
                show_window=args.show_window,
            ):
                print(values)
            return

        hand = create_hand_from_arm_config(args.arm_config)
        latest_queue = thread_queue.Queue(maxsize=1)
        stop_event = threading.Event()

        print(
            f"Local mode started: source={args.source}, hand_type={detector_hand_type}, "
            f"config={config_path}"
        )

        def producer():
            try:
                for values in iter_mapped_joints(
                    source=args.source,
                    config_path=str(config_path),
                    hand_type=detector_hand_type,
                    scaling_factor=args.scaling_factor,
                    show_window=args.show_window,
                ):
                    if stop_event.is_set():
                        break
                    if latest_queue.full():
                        try:
                            latest_queue.get_nowait()
                        except thread_queue.Empty:
                            pass
                    latest_queue.put_nowait(values)
            finally:
                stop_event.set()

        producer_thread = threading.Thread(target=producer, daemon=True)
        producer_thread.start()

        min_interval = 1.0 / max(args.control_hz, 1.0)
        last_sent = None
        next_send_ts = time.perf_counter()
        try:
            while not stop_event.is_set():
                try:
                    values = latest_queue.get(timeout=0.05)
                except thread_queue.Empty:
                    continue

                now = time.perf_counter()
                if now < next_send_ts:
                    continue

                cmd_values = [int(v) for v in values]
                if last_sent is not None and all(
                    abs(cmd_values[i] - last_sent[i]) <= args.deadband
                    for i in range(len(cmd_values))
                ):
                    continue

                hand.finger_control(cmd_values)
                last_sent = cmd_values
                next_send_ts = now + min_interval
                if args.verbose:
                    print(f"sent: {cmd_values}")
        except KeyboardInterrupt:
            pass
        finally:
            stop_event.set()
            producer_thread.join(timeout=1.0)
        return

    if args.debug_joints:
        print(f"Debug mode started (HTTP), reading from: {args.stream_url}")
        for values in iter_joint_stream_http(args.stream_url):
            print(values)
        return

    hand = create_hand_from_arm_config(args.arm_config)

    print(f"HTTP mode started, reading from: {args.stream_url}")
    for values in iter_joint_stream_http(args.stream_url):
        hand.finger_control(values)
        print(f"sent: {values}")


if __name__ == "__main__":
    main()
