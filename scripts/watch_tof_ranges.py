#!/usr/bin/env python3

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Range


TOPICS = [
    "/high/front_left/range",
    "/high/front_right/range",
    "/high/left_left/range",
    "/high/left_right/range",
    "/high/right_left/range",
    "/high/right_right/range",
    "/high/back_left/range",
    "/high/down_back2/range",
    "/high/up_front/range",
    "/high/down_front/range",
    "/high/up_back/range",
    "/high/down_back/range",
]


class TofRangeWatch(Node):
    def __init__(self):
        super().__init__("tof_range_watch")
        self.latest = {}
        self.start_time = time.monotonic()
        for topic in TOPICS:
            self.create_subscription(
                Range,
                topic,
                lambda msg, topic=topic: self.on_range(topic, msg),
                qos_profile_sensor_data,
            )
        self.create_timer(0.2, self.print_table)

    def on_range(self, topic, msg):
        self.latest[topic] = (time.monotonic(), msg)

    def print_table(self):
        now = time.monotonic()
        sys.stdout.write("\033[2J\033[H")
        print(f"TFmini-S Range Monitor  t={now - self.start_time:8.1f}s")
        print("-" * 88)
        print(f"{'topic':32s} {'frame':14s} {'range_m':>9s} {'range_cm':>9s} {'age_ms':>8s} status")
        print("-" * 88)
        for topic in TOPICS:
            item = self.latest.get(topic)
            if item is None:
                print(f"{topic:32s} {'-':14s} {'-':>9s} {'-':>9s} {'-':>8s} NO_DATA")
                continue

            stamp, msg = item
            age_ms = (now - stamp) * 1000.0
            value = msg.range
            if not math.isfinite(value):
                range_m = str(value)
                range_cm = "-"
                status = "BAD"
            else:
                range_m = f"{value:9.3f}"
                range_cm = f"{value * 100.0:9.1f}"
                status = "OK" if age_ms <= 500.0 else "STALE"

            print(
                f"{topic:32s} {msg.header.frame_id:14s} "
                f"{range_m:>9s} {range_cm:>9s} {age_ms:8.0f} {status}"
            )
        print("-" * 88)
        print("Ctrl-C to quit")


def main():
    rclpy.init()
    node = TofRangeWatch()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
