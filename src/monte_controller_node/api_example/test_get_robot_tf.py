import argparse
import math
import sys
from pathlib import Path

# autopep8: off
SCRIPT_DIR = Path(__file__).resolve().parent
LIB_DIR = SCRIPT_DIR.parent / "lib"
sys.path.append(str(LIB_DIR))
from RobotLib import Robot
# autopep8: on


ARM_PRESETS = {
    "left": {
        "parent_id": "link_l7_wrist_roll",
        "child_id": "link_lt_sensor_rgbd",
        "yaml_key": "joint_lt_sensor_rgbd",
    },
    "right": {
        "parent_id": "link_r7_wrist_roll",
        "child_id": "link_rt_sensor_rgbd",
        "yaml_key": "joint_rt_sensor_rgbd",
    },
}


def get_char():
    import getch
    return getch.getch()

def quat_wxyz_to_rpy(qw, qx, qy, qz):
    # RobotLib get_tf_transform quaternion order is [w, x, y, z]
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def save_transform_to_file(path, yaml_key, transform):
    if len(transform) < 7:
        raise ValueError(f"transform length invalid: {len(transform)}")

    x, y, z = [float(transform[i]) for i in range(3)]
    qw, qx, qy, qz = [float(transform[i]) for i in range(3, 7)]
    roll, pitch, yaw = quat_wxyz_to_rpy(qw, qx, qy, qz)

    lines = [
        f"{yaml_key}:\n",
        f"    x: {x}\n",
        f"    y: {y}\n",
        f"    z: {z}\n",
        f"    roll: {roll}\n",
        f"    pitch: {pitch}\n",
        f"    yaw: {yaw}\n",
    ]

    out_path = Path(path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("".join(lines), encoding="utf-8")
    print(f"saved file: {out_path}")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Query robot TF via RobotLib; optional save as x/y/z/roll/pitch/yaw text."
    )
    parser.add_argument(
        "--robot-ip",
        default="192.168.11.3:50051",
        help="Robot gRPC endpoint, e.g. 192.168.11.3:50051",
    )
    parser.add_argument(
        "--arm",
        choices=["left", "right", "both"],
        default="left",
        help="Use preset frames for left/right/both hands.",
    )
    parser.add_argument(
        "--parent-id",
        default="",
        help="Custom parent frame. If set, must be used with --child-id.",
    )
    parser.add_argument(
        "--child-id",
        default="",
        help="Custom child frame. If set, must be used with --parent-id.",
    )
    parser.add_argument(
        "--output-file",
        default="",
        help="Output file for single-arm/custom mode. In both mode, auto-creates *_left/*_right.",
    )
    parser.add_argument(
        "--yaml-key",
        default="",
        help="Top-level key used in output txt. Default: preset yaml key or child-id.",
    )
    parser.add_argument(
        "--no-prompt",
        action="store_true",
        help="Disable keypress prompts for scripting.",
    )
    return parser.parse_args()


def build_jobs(args):
    has_parent = bool(args.parent_id)
    has_child = bool(args.child_id)

    if has_parent != has_child:
        raise ValueError("--parent-id and --child-id must be set together")

    if has_parent and has_child:
        yaml_key = args.yaml_key if args.yaml_key else args.child_id
        return [{
            "name": "custom",
            "parent_id": args.parent_id,
            "child_id": args.child_id,
            "yaml_key": yaml_key,
            "output_file": args.output_file,
        }]

    if args.arm in ("left", "right"):
        preset = ARM_PRESETS[args.arm]
        yaml_key = args.yaml_key if args.yaml_key else preset["yaml_key"]
        return [{
            "name": args.arm,
            "parent_id": preset["parent_id"],
            "child_id": preset["child_id"],
            "yaml_key": yaml_key,
            "output_file": args.output_file,
        }]

    # both
    left = ARM_PRESETS["left"]
    right = ARM_PRESETS["right"]
    left_output = ""
    right_output = ""
    if args.output_file:
        out = Path(args.output_file)
        left_output = str(out.with_name(f"{out.stem}_left{out.suffix}"))
        right_output = str(out.with_name(f"{out.stem}_right{out.suffix}"))

    left_yaml_key = f"{args.yaml_key}_left" if args.yaml_key else left["yaml_key"]
    right_yaml_key = f"{args.yaml_key}_right" if args.yaml_key else right["yaml_key"]
    return [
        {
            "name": "left",
            "parent_id": left["parent_id"],
            "child_id": left["child_id"],
            "yaml_key": left_yaml_key,
            "output_file": left_output,
        },
        {
            "name": "right",
            "parent_id": right["parent_id"],
            "child_id": right["child_id"],
            "yaml_key": right_yaml_key,
            "output_file": right_output,
        },
    ]


def run_job(robot, job):
    parent_id = job["parent_id"]
    child_id = job["child_id"]
    success, transform = robot.get_tf_transform(parent_id, child_id)
    print(
        f"[{job['name']}] get_tf_transform success: {success}, "
        f"parent={parent_id}, child={child_id}, transform={transform}"
    )
    if not success:
        return False

    if job["output_file"]:
        save_transform_to_file(job["output_file"], job["yaml_key"], transform)

    return True


def main():
    args = parse_args()
    try:
        jobs = build_jobs(args)
    except ValueError as exc:
        print(f"ERROR: {exc}")
        return 2

    robot = Robot(args.robot_ip, "", "")

    if not args.no_prompt:
        print(f"press any key to get robot tf for {len(jobs)} job(s) ...")
        get_char()

    all_ok = True
    for job in jobs:
        if not run_job(robot, job):
            all_ok = False

    if not args.no_prompt:
        print("press any key to exit ...")
        get_char()

    return 0 if all_ok else 1


if __name__ == '__main__':
    raise SystemExit(main())
