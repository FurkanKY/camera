#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2


class WebcamPublisher(Node):
    def __init__(self):
        super().__init__('webcam_publisher')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/webcam/image_compressed',
            qos_profile
        )

        # Bu node ANA kamera akisinda (webcam/camera_manager) kullanilmaz;
        # ayri/eski bir yardimci yayindir. Yine de /dev/videoN numaralari
        # USB algilanma sirasina gore degisebilir; ONERILEN kullanim sabit
        # kalan bir yol vermektir:
        #   ls -la /dev/v4l/by-id/    (kameranin seri numarasina gore)
        #   ls -la /dev/v4l/by-path/  (USB portuna gore)
        # DIKKAT: bu node calistirilacaksa, ayni fiziksel kamerayi baska
        # bir node (ornegin camera_manager'in acdigi bir webcam) ayni anda
        # kullanmasin; yoksa cihaz "busy" olur ve biri acamaz.
        self.declare_parameter('camera_path', '/dev/video6')
        self.camera_path = str(self.get_parameter('camera_path').value)

        self.cap = cv2.VideoCapture(self.camera_path, cv2.CAP_V4L2)

        # Kamera ayarları
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        if not self.cap.isOpened():
            self.get_logger().error(f"Kamera acilamadi: {self.camera_path}")
            self.get_logger().error("Kamera busy olabilir veya yanlis video device secilmis olabilir.")
        else:
            self.get_logger().info(f"Kamera acildi: {self.camera_path}")

        self.timer = self.create_timer(1.0 / 30.0, self.timer_callback)

        self.get_logger().info("Webcam yayini basladi.")
        self.get_logger().info("Topic: /webcam/image_compressed")

    def timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().warning("Kamera acik degil.")
            return

        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warning("Kameradan goruntu okunamadi.")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'webcam_link'
        msg.format = 'jpeg'

        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 80]
        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)

        if not success:
            self.get_logger().warning("JPEG encode basarisiz.")
            return

        msg.data = encoded_image.tobytes()
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = WebcamPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
