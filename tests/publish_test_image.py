#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Publish synthetic sensor_msgs/Image frames for perception topic tests."""

from __future__ import annotations

import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class TestImagePublisher(Node):
    def __init__(self, topic: str, count: int, image_path: str = "", strict_image: bool = False) -> None:
        super().__init__("test_image_publisher")
        self._publisher = self.create_publisher(Image, topic, 10)
        self._count = count
        self._published = 0
        self._done = False
        self._strict_image = strict_image
        self._fixed_image = self._load_image(image_path)
        self._timer = self.create_timer(0.1, self._publish_image)
        if self._fixed_image is not None:
            self.get_logger().info(f"Publishing to {topic} using image: {image_path}")
        else:
            self.get_logger().info(f"Publishing to {topic} using synthetic random frames")

    def _load_image(self, image_path: str):
        if not image_path:
            return None
        try:
            import cv2
        except Exception as exc:
            message = f"cv2 unavailable, cannot load image '{image_path}': {exc}"
            if self._strict_image:
                raise RuntimeError(message) from exc
            self.get_logger().warning(f"{message}; fallback to random frames")
            return None
        image = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if image is None:
            message = f"Failed to read image '{image_path}'"
            if self._strict_image:
                raise RuntimeError(message)
            self.get_logger().warning(f"{message}, fallback to random frames")
            return None
        h, w = image.shape[:2]
        return (h, w, (w * 3), image.tobytes())

    @property
    def done(self) -> bool:
        return self._done

    def _publish_image(self) -> None:
        if self._done:
            return
        if self._published >= self._count:
            self.get_logger().info(f"Published {self._published} images, done")
            self._timer.cancel()
            self._done = True
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        if self._fixed_image is not None:
            msg.height, msg.width, msg.step, msg.data = self._fixed_image
        else:
            msg.height = 480
            msg.width = 640
            msg.step = 640 * 3
            msg.data = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8).tobytes()
        msg.encoding = "bgr8"
        self._publisher.publish(msg)
        self._published += 1
        if self._published == 1:
            self.get_logger().info("First image published")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--count", type=int, default=10)
    parser.add_argument("--image", default="", help="Optional image path for deterministic frames")
    parser.add_argument(
        "--strict-image",
        action="store_true",
        help="Fail instead of fallback when --image cannot be loaded",
    )
    args = parser.parse_args()

    rclpy.init()
    try:
        node = TestImagePublisher(args.topic, args.count, args.image, args.strict_image)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        if rclpy.ok():
            rclpy.shutdown()
        return 2
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
