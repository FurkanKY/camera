#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import time
import os


IMAGE_TOPIC = "/zed2/low_res_image"


class ZedLowResGUI(Node):
    def __init__(self):
        super().__init__("zed_lowres_gui")

        self.bridge = CvBridge()
        self.latest_frame = None
        self.frame_counter = 0

        self.subscription = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.image_callback,
            10
        )

        self.get_logger().info(f"Dinlenen topic: {IMAGE_TOPIC}")
        self.get_logger().info("s = foto kaydet | q = cikis")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.latest_frame = frame
            self.frame_counter += 1

        except Exception as e:
            self.get_logger().error(f"Goruntu donusturme hatasi: {e}")


def resize_for_display(frame, target_width=960):
    h, w = frame.shape[:2]

    if w == 0 or h == 0:
        return frame

    scale = target_width / float(w)
    target_height = int(h * scale)

    resized = cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_LINEAR)
    return resized


def draw_overlay(frame):
    h, w = frame.shape[:2]

    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.55
    thickness = 1

    # Üst siyah yarı panel
    overlay = frame.copy()
    cv2.rectangle(overlay, (0, 0), (w, 75), (0, 0, 0), -1)
    frame = cv2.addWeighted(overlay, 0.45, frame, 0.55, 0)

    cv2.putText(
        frame,
        f"Topic: {IMAGE_TOPIC}",
        (15, 28),
        font,
        font_scale,
        (0, 255, 0),
        thickness,
        cv2.LINE_AA
    )

    cv2.putText(
        frame,
        "s: save photo   |   q: quit",
        (15, 58),
        font,
        font_scale,
        (0, 255, 255),
        thickness,
        cv2.LINE_AA
    )

    return frame


def create_waiting_screen():
    waiting = 255 * cv2.UMat(540, 960, cv2.CV_8UC3).get()

    cv2.putText(
        waiting,
        "Goruntu bekleniyor...",
        (270, 250),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.9,
        (0, 0, 255),
        2,
        cv2.LINE_AA
    )

    cv2.putText(
        waiting,
        IMAGE_TOPIC,
        (300, 295),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 0, 0),
        1,
        cv2.LINE_AA
    )

    return waiting


def main(args=None):
    rclpy.init(args=args)

    node = ZedLowResGUI()

    save_folder = os.path.expanduser("~/zed_photos")
    os.makedirs(save_folder, exist_ok=True)

    window_name = "ZED Low Resolution Camera"

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 540)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            if node.latest_frame is not None:
                raw_frame = node.latest_frame.copy()

                # Önce görüntüyü büyüt
                display_frame = resize_for_display(raw_frame, target_width=960)

                # Sonra yazıyı ekle
                display_frame = draw_overlay(display_frame)

                cv2.imshow(window_name, display_frame)

            else:
                waiting = create_waiting_screen()
                cv2.imshow(window_name, waiting)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            elif key == ord("s"):
                if node.latest_frame is not None:
                    filename = time.strftime("zed_photo_%Y%m%d_%H%M%S.jpg")
                    path = os.path.join(save_folder, filename)

                    cv2.imwrite(path, node.latest_frame)
                    print(f"[KAYDEDILDI] {path}")
                else:
                    print("[UYARI] Kaydedilecek goruntu yok.")

    except KeyboardInterrupt:
        pass

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()