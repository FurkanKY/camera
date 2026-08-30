#!/usr/bin/env python3
"""Mikroskop etabi: 4 USB kamera + IP kamera + mikroskop kamerasi viewer'i."""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


CAMERAS = ('web1', 'web2', 'web3', 'web4', 'web5', 'ip', 'microscope')
TOPICS = {
    'web1': '/cam1/image_compressed',
    'web2': '/cam2/image_compressed',
    'web3': '/cam3/image_compressed',
    'web4': '/cam4/image_compressed',
    'web5': '/cam5/image_compressed',
    'ip': '/ip_cam/image_compressed',
    'microscope': '/microscope/image_compressed',
}
LABELS = {
    'web1': 'CAM 1 - WEB 1',
    'web2': 'CAM 2 - WEB 2',
    'web3': 'CAM 3 - WEB 3',
    'web4': 'CAM 4 - WEB 4',
    'web5': 'CAM 5 - WEB 5',
    'ip': 'CAM 6 - IP CAM',
    'microscope': 'CAM 7 - MICROSCOPE',
}
KEY_COMMANDS = {
    ord('1'): '1', ord('2'): '2', ord('3'): '3', ord('4'): '4', ord('5'): '5', ord('6'): '6',
}


class MicroscopeMissionViewer(Node):
    def __init__(self):
        super().__init__('microscope_mission_viewer_node')
        self.declare_parameter('ui_fps', 20.0)
        self.ui_fps = max(1.0, float(self.get_parameter('ui_fps').value))

        self.panel_width = 640
        self.panel_height = 480
        self.stale_seconds = 2.0
        self.running = True
        self.lock = threading.Lock()
        self.images = {name: self.placeholder(f'{LABELS[name]} KAPALI') for name in CAMERAS}
        self.states = {name: False for name in CAMERAS}
        self.last_frame = {name: 0.0 for name in CAMERAS}
        self.count = {name: 0 for name in CAMERAS}
        self.last_count = {name: 0 for name in CAMERAS}
        self.display_fps = {name: 0.0 for name in CAMERAS}
        self.last_fps_time = time.monotonic()

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.command_pub = self.create_publisher(
            String, '/camera_manager/command', command_qos
        )
        self.create_subscription(
            String, '/camera_manager/status', self.status_callback, command_qos
        )
        for name in CAMERAS:
            self.create_subscription(
                CompressedImage, TOPICS[name], self.make_image_callback(name), image_qos
            )

        self.window_name = 'Yildiz Rover Camera Viewer - 5 WEB + IP + MICROSCOPE'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.panel_width * 3, self.panel_height * 3)
        self.get_logger().info('Mikroskop etap viewer baslatildi.')
        self.get_logger().info('1-5: WEB1-5, 6: IP CAM ac-kapat | q: ana kameralari kapat ve cik')

    def placeholder(self, text):
        frame = np.zeros((self.panel_height, self.panel_width, 3), dtype=np.uint8)
        cv2.rectangle(frame, (0, 0), (self.panel_width - 1, self.panel_height - 1), (80, 80, 80), 3)
        cv2.putText(frame, text, (24, self.panel_height // 2), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 0, 255), 2, cv2.LINE_AA)
        return frame

    def status_callback(self, msg):
        # camera_manager web1..web5 ve IP kamera durumunu yonetir;
        # mikroskop publisher'i launch dosyasindan ayri baslatilir.
        with self.lock:
            for part in msg.data.split(','):
                if ':' not in part:
                    continue
                name, state = (value.strip() for value in part.split(':', 1))
                if name not in self.states or name == 'microscope':
                    continue
                is_on = state == 'ON'
                if self.states[name] and not is_on:
                    self.images[name] = self.placeholder(f'{LABELS[name]} KAPALI')
                self.states[name] = is_on

    def make_image_callback(self, name):
        def callback(msg):
            decoded = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if decoded is None:
                return
            source_height, source_width = decoded.shape[:2]
            scale = min(self.panel_width / source_width, self.panel_height / source_height)
            resized = cv2.resize(decoded, (round(source_width * scale), round(source_height * scale)))
            panel = np.zeros((self.panel_height, self.panel_width, 3), dtype=np.uint8)
            y = (self.panel_height - resized.shape[0]) // 2
            x = (self.panel_width - resized.shape[1]) // 2
            panel[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
            with self.lock:
                self.images[name] = panel
                self.states[name] = True
                self.last_frame[name] = time.monotonic()
                self.count[name] += 1
        return callback

    def update_fps(self):
        now = time.monotonic()
        elapsed = now - self.last_fps_time
        if elapsed < 1.0:
            return
        with self.lock:
            for name in CAMERAS:
                self.display_fps[name] = (self.count[name] - self.last_count[name]) / elapsed
                self.last_count[name] = self.count[name]
        self.last_fps_time = now

    def labelled_frame(self, name, image, is_on, last_frame, fps):
        frame = image.copy()
        cv2.rectangle(frame, (0, 0), (self.panel_width, 42), (0, 0, 0), -1)
        color = (0, 255, 0) if is_on else (0, 0, 255)
        cv2.putText(frame, f'{LABELS[name]} [{"ON" if is_on else "OFF"}]', (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv2.LINE_AA)
        if is_on:
            text = f'{fps:.0f} FPS'
            (width, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.52, 2)
            cv2.putText(frame, text, (self.panel_width - width - 12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2, cv2.LINE_AA)
        if is_on and last_frame and time.monotonic() - last_frame > self.stale_seconds:
            cv2.putText(frame, 'GORUNTU DONDU', (120, self.panel_height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 0, 255), 2, cv2.LINE_AA)
        return frame

    def help_panel(self):
        frame = np.zeros((self.panel_height, self.panel_width, 3), dtype=np.uint8)
        lines = ('KONTROL', '1-5: WEB 1-5', '6: IP CAM', 'q / ESC: CIKIS')
        for index, text in enumerate(lines):
            color = (0, 255, 0) if index == 0 else (255, 255, 255)
            cv2.putText(frame, text, (28, 45 + index * 38), cv2.FONT_HERSHEY_SIMPLEX,
                        0.65, color, 2, cv2.LINE_AA)
        return frame

    def draw(self):
        self.update_fps()
        with self.lock:
            frames = {
                name: self.labelled_frame(name, self.images[name], self.states[name],
                                          self.last_frame[name], self.display_fps[name])
                for name in CAMERAS
            }
        image = cv2.vconcat((
            cv2.hconcat((frames['web1'], frames['web2'], frames['web3'])),
            cv2.hconcat((frames['web4'], frames['web5'], frames['ip'])),
            cv2.hconcat((frames['microscope'], self.help_panel(), self.placeholder('MICROSCOPE ETABI'))),
        ))
        cv2.imshow(self.window_name, image)
        key = cv2.waitKey(1) & 0xFF
        if key in KEY_COMMANDS:
            command = String()
            command.data = KEY_COMMANDS[key]
            self.command_pub.publish(command)
        elif key in (ord('q'), 27):
            command = String()
            command.data = 'all_off'
            self.command_pub.publish(command)
            self.running = False

    def run(self, executor):
        period = 1.0 / self.ui_fps
        while self.running and rclpy.ok():
            started = time.monotonic()
            executor.spin_once(timeout_sec=0.0)
            self.draw()
            sleep_time = period - (time.monotonic() - started)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def destroy_node(self):
        self.running = False
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MicroscopeMissionViewer()
    executor = MultiThreadedExecutor(num_threads=7)
    executor.add_node(node)
    try:
        node.run(executor)
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()