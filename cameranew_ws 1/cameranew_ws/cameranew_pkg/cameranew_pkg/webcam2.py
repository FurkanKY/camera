#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class Webcam2CompressedPublisher(Node):
    def __init__(self):
        super().__init__('webcam2_compressed_publisher')

        # =========================================================
        # PARAMETERS
        # =========================================================
        # /dev/videoN numaralari USB algilanma sirasina gore
        # degisebilir. ONERILEN kullanim, sabit kalan
        # /dev/v4l/by-path/... yolunu vermektir:
        #   ls -la /dev/v4l/by-path/
        # ros2 run camera_pkg webcam2 --ros-args -p camera_index:=/dev/v4l/by-path/pci-....-usb-0:...-video-index0
        # Not: parametre artik string. Duz sayi ile eski /dev/videoN
        # kullanimi gerekirse tirnak icinde ver: -p camera_index:="6"
        self.declare_parameter('camera_index', '4')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 15)
        self.declare_parameter('jpeg_quality', 70)

        self.camera_index = self.normalize_device(
            self.get_parameter('camera_index').value
        )
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        # =========================================================
        # ROS 2 PUBLISHER
        # =========================================================
        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/cam2/image_compressed',
            10
        )

        # =========================================================
        # CAMERA OPEN
        # =========================================================
        self.get_logger().info(f'Webcam2 aciliyor: {self.camera_index}')

        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(f'Webcam2 acilamadi: {self.camera_index}')
            raise RuntimeError('Webcam2 acilamadi')

        # Logitech C922 için MJPG daha stabil olur
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().error(f'{self.camera_index} acildi ama frame okunamadi.')
            raise RuntimeError('Webcam2 frame okunamadi')

        real_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        real_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        real_fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(f'Webcam2 baslatildi: {self.camera_index}')
        self.get_logger().info(f'Cozunurluk: {real_width}x{real_height}, FPS: {real_fps}')
        self.get_logger().info('Publishing: /cam2/image_compressed')

        # =========================================================
        # TIMER
        # =========================================================
        self.timer = self.create_timer(
            1.0 / float(self.fps),
            self.timer_callback
        )

    def normalize_device(self, device):
        """
        Sayisal string ("4") gelirse int index'e cevirir (eski /dev/videoN
        davranisi). /dev/v4l/by-path/... veya /dev/v4l/by-id/... gibi
        sabit bir yol gelirse oldugu gibi string olarak kullanir.
        """

        device = str(device).strip()

        if device.lstrip('-').isdigit():
            return int(device)

        return device

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.get_logger().warn('Webcam2 frame okunamadi')
            return

        encode_param = [
            int(cv2.IMWRITE_JPEG_QUALITY),
            self.jpeg_quality
        ]

        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)

        if not success:
            self.get_logger().warn('Webcam2 JPEG encode basarisiz')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'cam2'
        msg.format = 'jpeg'
        msg.data = encoded_image.tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = Webcam2CompressedPublisher()

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
