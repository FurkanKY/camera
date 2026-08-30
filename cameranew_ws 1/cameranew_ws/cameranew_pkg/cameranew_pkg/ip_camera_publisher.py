#!/usr/bin/env python3

import os

os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
    "rtsp_transport;udp|"
    "fflags;nobuffer|"
    "flags;low_delay|"
    "stimeout;500000|"
    "rw_timeout;500000|"
    "max_delay;100000"
)

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import threading
import time


class IpCameraPublisher(Node):
    def __init__(self):
        super().__init__('ip_camera_publisher')

        # NOT: Varsayilan olarak gercek bir kimlik bilgisi VERILMEZ (guvenlik).
        # Calistirirken launch/parametre dosyasindan gecin:
        #   -p ip_url:="rtsp://<kullanici>:<sifre>@<ip>:554/<yol>"
        self.declare_parameter('ip_url', '')

        self.declare_parameter('fps', 20)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('jpeg_quality', 50)

        self.ip_url = str(self.get_parameter('ip_url').value)

        if not self.ip_url:
            self.get_logger().error(
                'ip_url parametresi bos. Kimlik bilgisi artik kod icinde '
                'tutulmuyor; -p ip_url:="rtsp://..." ile gecin.'
            )

        self.fps = int(self.get_parameter('fps').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self.publisher_ = self.create_publisher(
            CompressedImage,
            '/ip_cam/image_compressed',
            10
        )

        self.cap = None
        self.running = True

        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.last_frame_time = 0.0

        self.get_logger().info('IP camera publisher baslatildi.')
        self.get_logger().info(f'RTSP URL: {self.ip_url}')
        self.get_logger().info(f'FPS: {self.fps}')
        self.get_logger().info(f'Cozunurluk: {self.width}x{self.height}')
        self.get_logger().info(f'JPEG quality: {self.jpeg_quality}')
        self.get_logger().info('Publishing: /ip_cam/image_compressed')

        # RTSP kamera okuma ayrı thread'de çalışır.
        self.capture_thread = threading.Thread(
            target=self.capture_loop,
            daemon=True
        )
        self.capture_thread.start()

        # ROS topic publish hızı.
        self.timer = self.create_timer(
            1.0 / float(self.fps),
            self.timer_callback
        )

    def open_camera(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None

        self.get_logger().info('IP kamera acilmaya calisiliyor...')

        self.cap = cv2.VideoCapture(self.ip_url, cv2.CAP_FFMPEG)

        # Buffer düşük olsun, eski frame birikmesin.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        try:
            self.cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 2000)
            self.cap.set(cv2.CAP_PROP_READ_TIMEOUT_MSEC, 2000)
        except Exception:
            pass

        if not self.cap.isOpened():
            self.get_logger().warn('IP kamera acilamadi.')
            self.cap.release()
            self.cap = None
            return False

        self.get_logger().info('IP kamera baglandi.')
        return True

    def capture_loop(self):
        fail_count = 0

        while self.running and rclpy.ok():
            if self.cap is None:
                ok = self.open_camera()

                if not ok:
                    time.sleep(1.0)
                    continue

            # -------------------------------------------------
            # GECIKME AZALTMA
            # -------------------------------------------------
            # RTSP buffer'da eski frame kalabiliyor.
            # Bu yüzden birkaç frame grab ile atılıp en güncel frame'e yaklaşılır.
            grabbed_ok = True

            for _ in range(3):
                grabbed_ok = self.cap.grab()

                if not grabbed_ok:
                    break

            if not grabbed_ok:
                fail_count += 1
                self.get_logger().warn(f'IP kamera grab basarisiz. fail_count={fail_count}')

                if fail_count >= 10:
                    self.get_logger().warn('IP kamera yeniden baglanacak.')

                    if self.cap is not None:
                        self.cap.release()
                        self.cap = None

                    fail_count = 0
                    time.sleep(1.0)

                continue

            ret, frame = self.cap.retrieve()

            if not ret or frame is None:
                fail_count += 1
                self.get_logger().warn(f'IP kamera frame okunamadi. fail_count={fail_count}')

                if fail_count >= 10:
                    self.get_logger().warn('IP kamera yeniden baglanacak.')

                    if self.cap is not None:
                        self.cap.release()
                        self.cap = None

                    fail_count = 0
                    time.sleep(1.0)

                continue

            fail_count = 0

            frame = cv2.resize(frame, (self.width, self.height))

            with self.frame_lock:
                self.latest_frame = frame.copy()
                self.last_frame_time = time.time()

            # CPU'yu gereksiz yormasın.
            time.sleep(0.003)

    def timer_callback(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return

            frame = self.latest_frame.copy()
            frame_age = time.time() - self.last_frame_time

        # Çok eski frame'i tekrar tekrar basma.
        if frame_age > 1.0:
            self.get_logger().warn('IP kamera son frame eski. Yayinlanmiyor.')
            return

        encode_param = [
            int(cv2.IMWRITE_JPEG_QUALITY),
            self.jpeg_quality
        ]

        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)

        if not success:
            self.get_logger().warn('IP kamera JPEG encode basarisiz.')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ip_cam'
        msg.format = 'jpeg'
        msg.data = encoded_image.tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        self.running = False

        if self.cap is not None:
            self.cap.release()
            self.cap = None

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = IpCameraPublisher()

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
