#!/usr/bin/env python3

import cv2
import time
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy


class CameraWorker:
    def __init__(self, node, name, device, width, height, fps, jpeg_quality, publisher):
        self.node = node
        self.name = name
        self.device = device

        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality

        self.publisher = publisher

        self.cap = None
        self.running = True
        self.thread = None

        self.last_publish_time = time.time()
        self.fail_count = 0

    def normalize_device(self, device):
        device = str(device).strip()

        if device.isdigit():
            return int(device)

        return device

    def open_camera(self):
        device_to_open = self.normalize_device(self.device)

        self.node.get_logger().info(f'{self.name} aciliyor: {self.device}')

        cap = cv2.VideoCapture(device_to_open, cv2.CAP_V4L2)

        if not cap.isOpened():
            self.node.get_logger().error(f'{self.name} acilamadi: {self.device}')
            return None

        # Eski frame birikmesini azaltır.
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # Logitech C922 için MJPG daha az USB bandwidth kullanır.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

        # USB bandwidth'i asıl düşüren ayarlar: width, height, fps.
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        ret, frame = cap.read()

        if not ret or frame is None:
            self.node.get_logger().error(f'{self.name} acildi ama frame okunamadi: {self.device}')
            cap.release()
            return None

        real_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        real_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        real_fps = cap.get(cv2.CAP_PROP_FPS)
        real_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))

        fourcc_text = ''.join([
            chr((real_fourcc >> 8 * i) & 0xFF)
            for i in range(4)
        ])

        self.node.get_logger().info(
            f'{self.name} baslatildi | device={self.device} | '
            f'{real_width}x{real_height} @ {real_fps} FPS | FOURCC={fourcc_text}'
        )

        return cap

    def start(self):
        self.thread = threading.Thread(target=self.loop, daemon=True)
        self.thread.start()

    def stop(self):
        self.running = False

        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def reconnect(self):
        self.node.get_logger().warn(f'{self.name} yeniden baslatiliyor...')

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        time.sleep(1.0)

        self.cap = self.open_camera()

        if self.cap is not None:
            self.fail_count = 0
            self.node.get_logger().info(f'{self.name} yeniden baslatildi.')

    def loop(self):
        period = 1.0 / float(self.fps)

        self.cap = self.open_camera()

        while self.running and rclpy.ok():
            start_time = time.time()

            if self.cap is None:
                self.reconnect()
                time.sleep(1.0)
                continue

            ret, frame = self.cap.read()

            if not ret or frame is None:
                self.fail_count += 1

                self.node.get_logger().warn(
                    f'{self.name} frame okunamadi. fail_count={self.fail_count}'
                )

                if self.fail_count >= 5:
                    self.reconnect()

                time.sleep(0.2)
                continue

            self.fail_count = 0

            encode_param = [
                int(cv2.IMWRITE_JPEG_QUALITY),
                self.jpeg_quality
            ]

            success, encoded_image = cv2.imencode('.jpg', frame, encode_param)

            if not success:
                self.node.get_logger().warn(f'{self.name} JPEG encode basarisiz')
                time.sleep(0.1)
                continue

            msg = CompressedImage()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = self.name
            msg.format = 'jpeg'
            msg.data = encoded_image.tobytes()

            self.publisher.publish(msg)

            self.last_publish_time = time.time()

            elapsed = time.time() - start_time
            sleep_time = period - elapsed

            if sleep_time > 0:
                time.sleep(sleep_time)


class DualWebcamCompressedPublisher(Node):
    def __init__(self):
        super().__init__('dual_webcam_compressed_publisher')

        # =========================================================
        # PARAMETERS
        # Bunlar launch dosyasından değiştirilecek.
        #
        # /dev/videoN numaralari USB algilanma sirasina gore
        # degisebildigi icin buraya /dev/v4l/by-path/... gibi sabit
        # bir yol yazilmasi onerilir:
        #   ls -la /dev/v4l/by-path/
        # (asagidaki /dev/videoN degerleri sadece gecici varsayilan)
        # =========================================================
        self.declare_parameter('camera1_device', '/dev/video2')
        self.declare_parameter('camera2_device', '/dev/video6')

        self.declare_parameter('width', 160)
        self.declare_parameter('height', 120)
        self.declare_parameter('fps', 2)
        self.declare_parameter('jpeg_quality', 25)

        self.camera1_device = str(self.get_parameter('camera1_device').value)
        self.camera2_device = str(self.get_parameter('camera2_device').value)

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        if self.fps <= 0:
            self.get_logger().warn('FPS gecersiz geldi. FPS 2 yapiliyor.')
            self.fps = 2

        # =========================================================
        # QOS
        # Görüntüde eski frame birikmesin.
        # =========================================================
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # =========================================================
        # PUBLISHERS
        # =========================================================
        self.pub_cam1 = self.create_publisher(
            CompressedImage,
            '/cam1/image_compressed',
            qos
        )

        self.pub_cam2 = self.create_publisher(
            CompressedImage,
            '/cam2/image_compressed',
            qos
        )

        # =========================================================
        # CAMERA WORKERS
        # Her kamera ayrı thread.
        # Bir kamera takılırsa diğeri mümkün olduğunca devam eder.
        # =========================================================
        self.cam1 = CameraWorker(
            node=self,
            name='cam1',
            device=self.camera1_device,
            width=self.width,
            height=self.height,
            fps=self.fps,
            jpeg_quality=self.jpeg_quality,
            publisher=self.pub_cam1
        )

        self.cam2 = CameraWorker(
            node=self,
            name='cam2',
            device=self.camera2_device,
            width=self.width,
            height=self.height,
            fps=self.fps,
            jpeg_quality=self.jpeg_quality,
            publisher=self.pub_cam2
        )

        self.cam1.start()
        self.cam2.start()

        self.watchdog_timer = self.create_timer(2.0, self.watchdog_callback)

        self.get_logger().info('Threadli dual webcam node baslatildi.')
        self.get_logger().info('Publishing: /cam1/image_compressed')
        self.get_logger().info('Publishing: /cam2/image_compressed')

    def watchdog_callback(self):
        now = time.time()

        cam1_age = now - self.cam1.last_publish_time
        cam2_age = now - self.cam2.last_publish_time

        if cam1_age > 5.0:
            self.get_logger().warn(f'cam1 son {cam1_age:.1f} saniyedir publish etmiyor.')

        if cam2_age > 5.0:
            self.get_logger().warn(f'cam2 son {cam2_age:.1f} saniyedir publish etmiyor.')

    def destroy_node(self):
        self.get_logger().info('Dual webcam node kapatiliyor...')

        self.cam1.stop()
        self.cam2.stop()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = DualWebcamCompressedPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Dual webcam node durduruldu.')

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
