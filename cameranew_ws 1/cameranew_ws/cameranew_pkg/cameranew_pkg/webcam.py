#!/usr/bin/env python3

import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class WebcamCompressedPublisher(Node):
    def __init__(self):
        super().__init__('webcam_compressed_publisher')

        # =========================================================
        # PARAMETERS
        # =========================================================
        # camera_index = "-1" olursa sadece /dev/video0,1,2 denenir.
        # /dev/videoN numaralari USB algilanma sirasina gore
        # degisebildigi icin ONERILEN kullanim, sabit kalan
        # /dev/v4l/by-path/... yolunu vermektir:
        #   ls -la /dev/v4l/by-path/
        # ros2 run camera_pkg webcam --ros-args -p camera_index:=/dev/v4l/by-path/pci-....-usb-0:...-video-index0
        # Not: parametre artik string. Duz sayi ile eski /dev/videoN
        # kullanimi gerekirse tirnak icinde ver: -p camera_index:="3"
        self.declare_parameter('camera_index', '-1')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('jpeg_quality', 80)

        # Bu node 4 webcam icin de kullanilir. Her calistirmada
        # farkli topic/frame_id verilir (varsayilan cam1).
        # Ornek: -p topic:=/cam3/image_compressed -p frame_id:=cam3
        self.declare_parameter('topic', '/cam1/image_compressed')
        self.declare_parameter('frame_id', 'cam1')

        self.camera_index = self.normalize_device(
            self.get_parameter('camera_index').value
        )
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self.topic = str(self.get_parameter('topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        # =========================================================
        # DAYANIKLILIK (yeniden baglanma) durumu
        # Kamera acilamazsa veya calisirken koparsa node OLMEZ;
        # arka planda otomatik yeniden baglanmayi dener.
        # =========================================================
        self.cap = None
        self.fail_count = 0            # ust uste okunamayan frame sayaci
        self.max_fail = 5             # bu kadar hatadan sonra kamera resetlenir
        self.reconnect_interval = 2.0  # yeniden acma denemeleri arasi min sure (sn)
        self.last_open_attempt = 0.0

        # =========================================================
        # ROS 2 PUBLISHER
        # =========================================================
        self.publisher_ = self.create_publisher(
            CompressedImage,
            self.topic,
            10
        )

        # =========================================================
        # KAMERAYI ILK ACMA DENEMESI
        # Basarisiz olursa raise ETMEZ; timer arka planda tekrar dener.
        # =========================================================
        self.cap = self.open_camera()

        if self.cap is None:
            self.get_logger().warn(
                'Kamera ilk acilista acilamadi. Arka planda yeniden denenecek.'
            )

        # =========================================================
        # TIMER
        # =========================================================
        timer_period = 1.0 / float(self.fps)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def open_camera(self):
        """
        Kamerayi acmayi dener. Basarili olursa acik cap doner,
        basarisiz olursa None doner (raise ETMEZ). camera_index == -1
        ise otomatik kamera aramasi yapar.
        """

        device = self.camera_index

        if device == -1:
            device = self.find_working_camera()

            if device == -1:
                self.get_logger().error(
                    '/dev/video0,1,2 denendi ama calisan kamera bulunamadi.'
                )
                return None

        self.get_logger().info(f'Kamera acilmaya calisiliyor: {device}')

        cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

        if not cap.isOpened():
            self.get_logger().error(f'Kamera acilamadi: {device}')
            cap.release()
            return None

        # Logitech C922 için MJPG daha stabil olur
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)

        # İlk frame testi
        ret, frame = cap.read()

        if not ret or frame is None:
            self.get_logger().error(f'{device} acildi ama frame okunamadi.')
            cap.release()
            return None

        real_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        real_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        real_fps = cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(f'Kamera acildi: {device}')
        self.get_logger().info(f'Cozunurluk: {real_width}x{real_height}, FPS: {real_fps}')
        self.get_logger().info(f'Publishing topic: {self.topic}')

        return cap

    def normalize_device(self, device):
        """
        Sayisal string ("3") gelirse int index'e cevirir (eski /dev/videoN
        davranisi). /dev/v4l/by-path/... veya /dev/v4l/by-id/... gibi
        sabit bir yol gelirse oldugu gibi string olarak kullanir.
        """

        device = str(device).strip()

        if device.lstrip('-').isdigit():
            return int(device)

        return device

    def find_working_camera(self):
        """
        Sadece /dev/video0, /dev/video1, /dev/video2 dener.
        Çalışan ilk kamerayı seçer.
        """

        self.get_logger().info('Otomatik kamera aramasi baslatiliyor...')
        self.get_logger().info('Sadece /dev/video0, /dev/video1, /dev/video2 denenecek.')

        for i in [0, 1, 2]:
            self.get_logger().info(f'/dev/video{i} deneniyor...')

            cap = cv2.VideoCapture(i, cv2.CAP_V4L2)

            if not cap.isOpened():
                self.get_logger().warn(f'/dev/video{i} acilamadi.')
                cap.release()
                continue

            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            cap.set(cv2.CAP_PROP_FPS, self.fps)

            ret, frame = cap.read()
            cap.release()

            if ret and frame is not None:
                self.get_logger().info(f'Calisan kamera bulundu: /dev/video{i}')
                return i

            self.get_logger().warn(f'/dev/video{i} acildi ama frame okunamadi.')

        return -1

    def timer_callback(self):
        # ---------------------------------------------------------
        # Kamera bagli degilse: throttle'li yeniden acma denemesi.
        # ---------------------------------------------------------
        if self.cap is None:
            now = time.time()

            if now - self.last_open_attempt < self.reconnect_interval:
                return

            self.last_open_attempt = now
            self.get_logger().warn('Kamera bagli degil, yeniden aciliyor...')

            self.cap = self.open_camera()

            if self.cap is not None:
                self.fail_count = 0
                self.get_logger().info('Kamera yeniden acildi.')

            return

        # ---------------------------------------------------------
        # Normal okuma. Ust uste hata olursa kamerayi resetle.
        # ---------------------------------------------------------
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.fail_count += 1
            self.get_logger().warn(
                f'Kameradan frame okunamadi. fail_count={self.fail_count}'
            )

            if self.fail_count >= self.max_fail:
                self.get_logger().warn(
                    'Cok fazla hata. Kamera kapatilip yeniden aciliyor.'
                )
                self.cap.release()
                self.cap = None
                self.fail_count = 0
                self.last_open_attempt = time.time()

            return

        self.fail_count = 0

        # JPEG sıkıştırma
        encode_param = [
            int(cv2.IMWRITE_JPEG_QUALITY),
            self.jpeg_quality
        ]

        success, encoded_image = cv2.imencode('.jpg', frame, encode_param)

        if not success:
            self.get_logger().warn('JPEG kodlama basarisiz')
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = encoded_image.tobytes()

        self.publisher_.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = WebcamCompressedPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('Webcam yayini kullanici tarafindan durduruldu')

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
