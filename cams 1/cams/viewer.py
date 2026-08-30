#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

import cv2
import numpy as np
import threading
import time


class RoverCameraViewer(Node):
    def __init__(self):
        super().__init__('rover_camera_viewer_node')

        # =================================================
        # PANEL BOYUTLARI
        # =================================================
        self.W = 480
        self.H = 270

        # Gelen görüntü çok hızlı olsa bile viewer max bu kadar decode eder.
        self.cam_fps = 8.0
        self.cam_update_period = 1.0 / self.cam_fps

        self.last_zed1_time = 0.0
        self.last_zed2_time = 0.0
        self.last_web1_time = 0.0
        self.last_web2_time = 0.0
        self.last_ip_time = 0.0

        self.running = True
        self.frame_lock = threading.Lock()

        # Kamera açık/kapalı durumları
        self.camera_states = {
            'zed1': False,
            'zed2': False,
            'web1': False,
            'web2': False,
            'ip': False,
        }

        # İlk açılışta hepsi kapalı görünsün
        self.img_zed1 = self.get_placeholder("ZED 1 KAPALI")
        self.img_zed2 = self.get_placeholder("ZED 2 KAPALI")
        self.img_web1 = self.get_placeholder("WEB CAM 1 KAPALI")
        self.img_web2 = self.get_placeholder("WEB CAM 2 KAPALI")
        self.img_ip = self.get_placeholder("IP CAM KAPALI")

        # =================================================
        # QOS
        # =================================================
        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        cmd_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # =================================================
        # CAMERA MANAGER KOMUT PUBLISHER
        # =================================================
        self.camera_cmd_pub = self.create_publisher(
            String,
            '/camera_manager/command',
            cmd_qos
        )

        self.camera_status_sub = self.create_subscription(
            String,
            '/camera_manager/status',
            self.camera_status_cb,
            cmd_qos
        )

        # =================================================
        # GÖRÜNTÜ TOPICLERİ
        # =================================================
        self.zed1_topic = '/zed1/image_compressed_low'
        self.zed2_topic = '/zed2/image_compressed_low'

        self.webcam1_topic = '/cam1/image_compressed'
        self.webcam2_topic = '/cam2/image_compressed'
        self.ip_topic = '/ip_cam/image_compressed'

        self.create_subscription(
            CompressedImage,
            self.zed1_topic,
            self.zed1_cb,
            image_qos
        )

        self.create_subscription(
            CompressedImage,
            self.zed2_topic,
            self.zed2_cb,
            image_qos
        )

        self.create_subscription(
            CompressedImage,
            self.webcam1_topic,
            self.webcam1_cb,
            image_qos
        )

        self.create_subscription(
            CompressedImage,
            self.webcam2_topic,
            self.webcam2_cb,
            image_qos
        )

        self.create_subscription(
            CompressedImage,
            self.ip_topic,
            self.ip_cb,
            image_qos
        )

        # =================================================
        # ARAYÜZ
        # =================================================
        self.win_name = "Yildiz Rover Camera Viewer - 5 Cameras"

        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win_name, 1440, 540)

        self.process_loop()

        self.get_logger().info("Rover camera viewer baslatildi.")
        self.get_logger().info("Tuslar:")
        self.get_logger().info("1 -> ZED 1 ac/kapat")
        self.get_logger().info("2 -> ZED 2 ac/kapat")
        self.get_logger().info("3 -> WEB CAM 1 ac/kapat")
        self.get_logger().info("4 -> WEB CAM 2 ac/kapat")
        self.get_logger().info("5 -> IP CAM ac/kapat")
        self.get_logger().info("q -> Hepsini kapat ve cik")

        self.get_logger().info(f"ZED 1 topic: {self.zed1_topic}")
        self.get_logger().info(f"ZED 2 topic: {self.zed2_topic}")
        self.get_logger().info(f"WEB CAM 1 topic: {self.webcam1_topic}")
        self.get_logger().info(f"WEB CAM 2 topic: {self.webcam2_topic}")
        self.get_logger().info(f"IP CAM topic: {self.ip_topic}")

        # Arayüz yenileme
        self.timer = self.create_timer(1.0 / 15.0, self.process_loop)

    # =================================================
    # CAMERA MANAGER STATUS
    # =================================================
    def camera_status_cb(self, msg):
        """
        Beklenen format:
        zed1:ON,zed2:OFF,web1:ON,web2:OFF,ip:OFF,zed_throttle:ON
        """

        parts = msg.data.split(',')

        new_states = {}

        for part in parts:
            if ':' not in part:
                continue

            name, state = part.split(':', 1)
            name = name.strip()
            state = state.strip()

            if name in self.camera_states:
                new_states[name] = (state == 'ON')

        with self.frame_lock:
            for name, is_on in new_states.items():
                old_state = self.camera_states.get(name, False)
                self.camera_states[name] = is_on

                # Kamera yeni kapandıysa paneli kapalı yap
                if old_state and not is_on:
                    self.set_camera_placeholder_locked(name, "KAPALI")

                # Kamera yeni açıldıysa ama görüntü henüz gelmediyse açılıyor yaz
                elif not old_state and is_on:
                    self.set_camera_placeholder_locked(name, "ACILIYOR...")

    # =================================================
    # KOMUT GÖNDERME
    # =================================================
    def send_camera_command(self, key):
        key_map = {
            ord('1'): ('1', 'zed1', "ZED 1"),
            ord('2'): ('2', 'zed2', "ZED 2"),
            ord('3'): ('3', 'web1', "WEB CAM 1"),
            ord('4'): ('4', 'web2', "WEB CAM 2"),
            ord('5'): ('5', 'ip', "IP CAM"),
        }

        if key not in key_map:
            return

        command, name, label = key_map[key]

        msg = String()
        msg.data = command
        self.camera_cmd_pub.publish(msg)

        # Hemen kullanıcıya görsel tepki ver
        with self.frame_lock:
            if self.camera_states.get(name, False):
                self.set_camera_placeholder_locked(name, "KAPATILIYOR...")
            else:
                self.set_camera_placeholder_locked(name, "ACILIYOR...")

        self.get_logger().info(f"{label} icin komut gonderildi: {command}")

    def send_all_off(self):
        msg = String()
        msg.data = "all_off"
        self.camera_cmd_pub.publish(msg)

        with self.frame_lock:
            self.img_zed1 = self.get_placeholder("ZED 1 KAPALI")
            self.img_zed2 = self.get_placeholder("ZED 2 KAPALI")
            self.img_web1 = self.get_placeholder("WEB CAM 1 KAPALI")
            self.img_web2 = self.get_placeholder("WEB CAM 2 KAPALI")
            self.img_ip = self.get_placeholder("IP CAM KAPALI")

        self.get_logger().info("Tum kameralari kapatma komutu gonderildi.")

    # =================================================
    # PLACEHOLDER YARDIMCI
    # =================================================
    def set_camera_placeholder_locked(self, name, state_text):
        if name == 'zed1':
            self.img_zed1 = self.get_placeholder(f"ZED 1 {state_text}")

        elif name == 'zed2':
            self.img_zed2 = self.get_placeholder(f"ZED 2 {state_text}")

        elif name == 'web1':
            self.img_web1 = self.get_placeholder(f"WEB CAM 1 {state_text}")

        elif name == 'web2':
            self.img_web2 = self.get_placeholder(f"WEB CAM 2 {state_text}")

        elif name == 'ip':
            self.img_ip = self.get_placeholder(f"IP CAM {state_text}")

    # =================================================
    # ORTAK FRAME İŞLEME
    # =================================================
    def decode_resize_compressed(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            raise RuntimeError("Compressed image decode edilemedi.")

        frame = cv2.resize(frame, (self.W, self.H))
        return frame

    # =================================================
    # CALLBACKLER
    # =================================================
    def zed1_cb(self, msg):
        now = time.time()

        if now - self.last_zed1_time < self.cam_update_period:
            return

        self.last_zed1_time = now

        try:
            frame = self.decode_resize_compressed(msg)

            with self.frame_lock:
                self.img_zed1 = frame.copy()
                self.camera_states['zed1'] = True

        except Exception as e:
            self.get_logger().error(f"ZED 1 kamera hatasi: {e}")

            with self.frame_lock:
                self.img_zed1 = self.get_placeholder("ZED 1 HATA")

    def zed2_cb(self, msg):
        now = time.time()

        if now - self.last_zed2_time < self.cam_update_period:
            return

        self.last_zed2_time = now

        try:
            frame = self.decode_resize_compressed(msg)

            with self.frame_lock:
                self.img_zed2 = frame.copy()
                self.camera_states['zed2'] = True

        except Exception as e:
            self.get_logger().error(f"ZED 2 kamera hatasi: {e}")

            with self.frame_lock:
                self.img_zed2 = self.get_placeholder("ZED 2 HATA")

    def webcam1_cb(self, msg):
        now = time.time()

        if now - self.last_web1_time < self.cam_update_period:
            return

        self.last_web1_time = now

        try:
            frame = self.decode_resize_compressed(msg)

            with self.frame_lock:
                self.img_web1 = frame.copy()
                self.camera_states['web1'] = True

        except Exception as e:
            self.get_logger().error(f"WEB CAM 1 hatasi: {e}")

            with self.frame_lock:
                self.img_web1 = self.get_placeholder("WEB CAM 1 HATA")

    def webcam2_cb(self, msg):
        now = time.time()

        if now - self.last_web2_time < self.cam_update_period:
            return

        self.last_web2_time = now

        try:
            frame = self.decode_resize_compressed(msg)

            with self.frame_lock:
                self.img_web2 = frame.copy()
                self.camera_states['web2'] = True

        except Exception as e:
            self.get_logger().error(f"WEB CAM 2 hatasi: {e}")

            with self.frame_lock:
                self.img_web2 = self.get_placeholder("WEB CAM 2 HATA")

    def ip_cb(self, msg):
        now = time.time()

        if now - self.last_ip_time < self.cam_update_period:
            return

        self.last_ip_time = now

        try:
            frame = self.decode_resize_compressed(msg)

            with self.frame_lock:
                self.img_ip = frame.copy()
                self.camera_states['ip'] = True

        except Exception as e:
            self.get_logger().error(f"IP CAM hatasi: {e}")

            with self.frame_lock:
                self.img_ip = self.get_placeholder("IP CAM HATA")

    # =================================================
    # GÖRSEL YARDIMCILAR
    # =================================================
    def get_placeholder(self, text):
        img = np.zeros((self.H, self.W, 3), dtype=np.uint8)

        cv2.rectangle(
            img,
            (0, 0),
            (self.W - 1, self.H - 1),
            (80, 80, 80),
            3
        )

        cv2.putText(
            img,
            text,
            (25, self.H // 2),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 0, 255),
            2
        )

        return img

    def put_label(self, frame, text, state_on):
        frame = frame.copy()

        cv2.rectangle(
            frame,
            (0, 0),
            (self.W, 42),
            (0, 0, 0),
            -1
        )

        state_text = "ON" if state_on else "OFF"

        cv2.putText(
            frame,
            f"{text} [{state_text}]",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0) if state_on else (0, 0, 255),
            2
        )

        return frame

    def get_help_panel(self):
        img = np.zeros((self.H, self.W, 3), dtype=np.uint8)

        lines = [
            "KONTROL",
            "1: ZED 1 ON/OFF",
            "2: ZED 2 ON/OFF",
            "3: WEB1 ON/OFF",
            "4: WEB2 ON/OFF",
            "5: IP CAM ON/OFF",
            "q: ALL OFF + EXIT",
        ]

        y = 35

        for i, line in enumerate(lines):
            color = (0, 255, 0) if i == 0 else (255, 255, 255)

            cv2.putText(
                img,
                line,
                (25, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2
            )

            y += 32

        return img

    # =================================================
    # ARAYÜZ LOOP
    # =================================================
    def process_loop(self):
        try:
            with self.frame_lock:
                img_zed1 = self.img_zed1.copy()
                img_zed2 = self.img_zed2.copy()
                img_web1 = self.img_web1.copy()
                img_web2 = self.img_web2.copy()
                img_ip = self.img_ip.copy()

                zed1_on = self.camera_states['zed1']
                zed2_on = self.camera_states['zed2']
                web1_on = self.camera_states['web1']
                web2_on = self.camera_states['web2']
                ip_on = self.camera_states['ip']

            frame_zed1 = self.put_label(img_zed1, "CAM 1 - ZED 1", zed1_on)
            frame_zed2 = self.put_label(img_zed2, "CAM 2 - ZED 2", zed2_on)
            frame_web1 = self.put_label(img_web1, "CAM 3 - WEB CAM 1", web1_on)
            frame_web2 = self.put_label(img_web2, "CAM 4 - WEB CAM 2", web2_on)
            frame_ip = self.put_label(img_ip, "CAM 5 - IP CAM", ip_on)
            frame_help = self.get_help_panel()

            top_row = cv2.hconcat([
                frame_zed1,
                frame_zed2,
                frame_web1
            ])

            bottom_row = cv2.hconcat([
                frame_web2,
                frame_ip,
                frame_help
            ])

            combined = cv2.vconcat([
                top_row,
                bottom_row
            ])

            cv2.imshow(self.win_name, combined)

            key = cv2.waitKey(1) & 0xFF

            if key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
                self.send_camera_command(key)

            elif key == ord('q'):
                self.get_logger().info("q basildi. Viewer kapatiliyor.")
                self.send_all_off()
                self.running = False
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Arayuz hatasi: {e}")

    # =================================================
    # NODE KAPATMA
    # =================================================
    def destroy_node(self):
        self.get_logger().info("Viewer kapatiliyor.")
        self.running = False
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = RoverCameraViewer()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        node.send_all_off()

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()