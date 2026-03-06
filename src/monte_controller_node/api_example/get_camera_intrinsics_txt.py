import argparse
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class CameraIntrinsicsTxtSaver(Node):
    def __init__(self, topic_name, output_file, frame_label, side_label):
        super().__init__("camera_intrinsics_txt_saver")
        self.topic_name = topic_name
        self.output_file = Path(output_file)
        self.frame_label = frame_label
        self.side_label = side_label
        self.got_info = False

        self.subscription = self.create_subscription(
            CameraInfo,
            self.topic_name,
            self._camera_info_callback,
            10,
        )
        self.get_logger().info(f"Subscribed to {self.topic_name}, waiting for CameraInfo...")

    @staticmethod
    def _fmt(v):
        return str(float(v))

    @classmethod
    def _fmt_row(cls, a, b, c):
        return f"{cls._fmt(a):<22} {cls._fmt(b):<22} {cls._fmt(c)}\n"

    def _camera_info_callback(self, msg):
        if self.got_info:
            return

        k = list(msg.k) if msg.k else [0.0] * 9
        d = list(msg.d) if msg.d else []

        lines = [
            f"# Camera intrinsics for {self.frame_label} ({self.side_label})\n",
            "# Format: K matrix (3x3)\n",
            "\n",
            "K:\n",
            self._fmt_row(k[0], k[1], k[2]),
            self._fmt_row(k[3], k[4], k[5]),
            self._fmt_row(k[6], k[7], k[8]),
            "\n",
            "# Camera parameters:\n",
            f"# fx = {self._fmt(k[0])}\n",
            f"# fy = {self._fmt(k[4])}\n",
            f"# cx = {self._fmt(k[2])}\n",
            f"# cy = {self._fmt(k[5])}\n",
            f"# width  = {int(msg.width)}\n",
            f"# height = {int(msg.height)}\n",
            f"# distortion_model = {msg.distortion_model}\n",
            "# distortion_coeffs = [\n",
        ]

        for i, value in enumerate(d):
            suffix = "," if i < len(d) - 1 else ""
            lines.append(f"#     {self._fmt(value)}{suffix}\n")
        lines.append("# ]\n")

        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        self.output_file.write_text("".join(lines), encoding="utf-8")
        self.get_logger().info(f"Saved camera intrinsics to {self.output_file}")
        self.got_info = True


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Subscribe one CameraInfo and save intrinsics to txt."
    )
    parser.add_argument(
        "--topic-name",
        "-t",
        required=True,
        help="CameraInfo topic, e.g. /left/color/camera_info",
    )
    parser.add_argument(
        "--output-file",
        "-o",
        required=True,
        help="Output txt path.",
    )
    parser.add_argument(
        "--frame-label",
        default="left_color_optical_frame",
        help="Comment label used in first line.",
    )
    parser.add_argument(
        "--side-label",
        default="LEFT COLOR",
        help="Comment label used in first line.",
    )
    parser.add_argument(
        "--wait-seconds",
        type=float,
        default=8.0,
        help="Max wait before returning failure.",
    )
    return parser.parse_known_args(argv)


def main(argv=None):
    args, remaining = parse_args(argv)
    rclpy.init(args=remaining)

    saver = CameraIntrinsicsTxtSaver(
        topic_name=args.topic_name,
        output_file=args.output_file,
        frame_label=args.frame_label,
        side_label=args.side_label,
    )

    t0 = time.monotonic()
    try:
        while rclpy.ok() and not saver.got_info:
            if time.monotonic() - t0 > args.wait_seconds:
                print(
                    f"ERROR: timeout waiting CameraInfo on {args.topic_name} "
                    f"after {args.wait_seconds}s"
                )
                return 1
            rclpy.spin_once(saver, timeout_sec=0.1)
    except KeyboardInterrupt:
        return 130
    finally:
        saver.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
