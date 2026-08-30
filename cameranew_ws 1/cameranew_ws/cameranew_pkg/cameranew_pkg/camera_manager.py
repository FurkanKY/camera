#!/usr/bin/env python3
import os
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


WEBCAMS = {
    "web1": {
        "key": "1",
        "cam_name": "cam1",
        "topic": "/cam1/image_compressed",
        "device": (
            "/dev/v4l/by-id/"
            "usb-046d_Brio_100_2523ZB739SU8-video-index0"
        ),
        "width": 320,
        "height": 240,
        "fps": 10,
        "jpeg_quality": 70,
        "input_fourcc": "MJPG",
    },
    "web2": {
        "key": "2",
        "cam_name": "cam2",
        "topic": "/cam2/image_compressed",
        "device": (
            "/dev/v4l/by-id/"
            "usb-046d_C922_Pro_Stream_Webcam_68F453DF-video-index0"
        ),
        "width": 320,
        "height": 240,
        "fps": 10,
        "jpeg_quality": 70,
        "input_fourcc": "MJPG",
    },
    "web4": {
        "key": "4",
        "cam_name": "cam4",
        "topic": "/cam4/image_compressed",
        "device": (
            "/dev/v4l/by-id/"
            "usb-046d_C922_Pro_Stream_Webcam_211533DF-video-index0"
        ),
        "width": 320,
        "height": 240,
        "fps": 10,
        "jpeg_quality": 70,
        "input_fourcc": "MJPG",
    },
}

AUTO_START_CAMERAS = ["web1", "web2", "web4"]

# 20 ardisik okuma hatasi yaklasik 1 saniye sonra yeniden baglanmayi tetikler.
READ_ERROR_LIMIT = 20
RECONNECT_DELAY = 1.0
RECONNECT_RETRY_DELAY = 3.0


class CameraWorker:
    def __init__(self, node, camera_name, config, publisher):
        self.node = node
        self.camera_name = camera_name
        self.config = config
        self.publisher = publisher

        self.capture = None
        self.thread = None
        self.stop_event = threading.Event()
        self.running = False
        self.reconnecting = False

        self.published_count = 0
        self.read_error_count = 0
        self.reconnect_count = 0
        self.last_publish_time = 0.0

    def start(self):
        if self.running:
            return

        self.stop_event.clear()
        self.running = True
        self.thread = threading.Thread(
            target=self.run,
            name=f"{self.camera_name}_worker",
            daemon=True,
        )
        self.thread.start()

    def stop(self):
        if not self.running:
            return

        self.stop_event.set()

        if self.thread is not None:
            self.thread.join(timeout=3.0)

        self.release_capture()
        self.thread = None
        self.running = False
        self.reconnecting = False

    def is_running(self):
        return self.running and not self.stop_event.is_set()

    def open_capture(self):
        device_path = self.config["device"]

        if not os.path.exists(device_path):
            self.node.get_logger().error(
                f"{self.camera_name}: cihaz bulunamadi: {device_path}"
            )
            return False

        source = os.path.realpath(device_path)
        self.node.get_logger().info(
            f"{self.camera_name} cihaz yolu: {device_path} -> {source}"
        )

        if not source.startswith("/dev/video"):
            self.node.get_logger().error(
                f"{self.camera_name}: gecersiz video cihaz yolu: {source}"
            )
            return False

        try:
            device_index = int(source.replace("/dev/video", "", 1))
        except ValueError:
            self.node.get_logger().error(
                f"{self.camera_name}: video cihaz numarasi okunamadi: {source}"
            )
            return False

        capture = cv2.VideoCapture(device_index, cv2.CAP_V4L2)

        if not capture.isOpened():
            self.node.get_logger().error(
                f"{self.camera_name} acilamadi: {source}"
            )
            capture.release()
            return False

        width = int(self.config["width"])
        height = int(self.config["height"])
        fps = float(self.config["fps"])
        fourcc_text = str(self.config.get("input_fourcc", "MJPG"))[:4]
        fourcc_text = fourcc_text.ljust(4, " ")
        fourcc_value = cv2.VideoWriter_fourcc(*fourcc_text)

        # Formati cozunurluk ve FPS'ten once vermek V4L2'de daha sagliklidir.
        capture.set(cv2.CAP_PROP_FOURCC, fourcc_value)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        capture.set(cv2.CAP_PROP_FPS, fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = capture.get(cv2.CAP_PROP_FPS)

        self.capture = capture
        self.node.get_logger().info(
            f"{self.camera_name} acildi | "
            f"Istenen: {width}x{height} {fps:.1f} FPS | "
            f"Gercek: {actual_width}x{actual_height} {actual_fps:.2f} FPS"
        )
        return True

    def release_capture(self):
        capture = self.capture
        self.capture = None

        if capture is not None:
            try:
                capture.release()
            except Exception as exc:
                self.node.get_logger().warn(
                    f"{self.camera_name}: capture release hatasi: {exc}"
                )

    def reconnect_capture(self):
        """Yalnizca hata veren kamerayi yeniden acar."""
        self.reconnecting = True
        self.release_capture()

        if self.stop_event.wait(RECONNECT_DELAY):
            self.reconnecting = False
            return False

        attempt = 0

        while not self.stop_event.is_set():
            attempt += 1
            self.node.get_logger().warn(
                f"{self.camera_name}: yeniden baglanma denemesi {attempt}"
            )

            if self.open_capture():
                self.read_error_count = 0
                self.reconnect_count += 1
                self.reconnecting = False
                self.node.get_logger().info(
                    f"{self.camera_name}: yeniden baglandi | "
                    f"toplam yeniden baglanma={self.reconnect_count}"
                )
                return True

            self.node.get_logger().error(
                f"{self.camera_name}: yeniden baglanamadi; "
                f"{RECONNECT_RETRY_DELAY:.1f} sn sonra tekrar denenecek."
            )

            if self.stop_event.wait(RECONNECT_RETRY_DELAY):
                break

        self.reconnecting = False
        return False

    def run(self):
        try:
            # Baslangicta kamera yoksa thread kapanmaz; takilinca yeniden acar.
            if not self.open_capture() and not self.reconnect_capture():
                return

            target_fps = max(float(self.config["fps"]), 1.0)
            frame_period = 1.0 / target_fps

            while not self.stop_event.is_set():
                loop_start = time.monotonic()

                if self.capture is None or not self.capture.isOpened():
                    if not self.reconnect_capture():
                        break
                    continue

                success, frame = self.capture.read()

                if not success or frame is None:
                    self.read_error_count += 1

                    if self.read_error_count == 1:
                        self.node.get_logger().warn(
                            f"{self.camera_name}: kare okunamadi."
                        )

                    if self.read_error_count % 10 == 0:
                        self.node.get_logger().warn(
                            f"{self.camera_name}: "
                            f"{self.read_error_count} ardisik okuma hatasi."
                        )

                    if self.read_error_count >= READ_ERROR_LIMIT:
                        self.node.get_logger().warn(
                            f"{self.camera_name}: kamera akisi koptu; "
                            "yalnizca bu kamera yeniden baslatiliyor."
                        )

                        if not self.reconnect_capture():
                            break
                        continue

                    self.stop_event.wait(0.05)
                    continue

                self.read_error_count = 0
                self.process_and_publish(frame)

                elapsed = time.monotonic() - loop_start
                remaining = frame_period - elapsed

                if remaining > 0.0:
                    self.stop_event.wait(remaining)

        except Exception as exc:
            self.node.get_logger().error(
                f"{self.camera_name} thread hatasi: {exc}"
            )

        finally:
            self.release_capture()
            self.reconnecting = False
            self.running = False
            self.node.get_logger().info(
                f"{self.camera_name} kamera thread'i kapandi."
            )

    def process_and_publish(self, frame):
        try:
            width = int(self.config["width"])
            height = int(self.config["height"])
            jpeg_quality = int(self.config["jpeg_quality"])

            if frame.shape[1] != width or frame.shape[0] != height:
                frame = cv2.resize(
                    frame,
                    (width, height),
                    interpolation=cv2.INTER_AREA,
                )

            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            encode_success, encoded = cv2.imencode(
                ".jpg",
                gray_frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), jpeg_quality],
            )

            if not encode_success:
                self.node.get_logger().error(
                    f"{self.camera_name}: JPEG sikistirma basarisiz."
                )
                return

            message = CompressedImage()
            message.header.stamp = self.node.get_clock().now().to_msg()
            message.header.frame_id = self.config["cam_name"]
            message.format = "jpeg"
            message.data = encoded.tobytes()

            self.publisher.publish(message)
            self.published_count += 1
            self.last_publish_time = time.monotonic()

        except Exception as exc:
            self.node.get_logger().error(
                f"{self.camera_name} yayin hatasi: {exc}"
            )


class CameraManager(Node):
    def __init__(self):
        super().__init__("camera_manager")

        self.valid_cameras = list(WEBCAMS.keys())
        self.key_to_camera = {
            config["key"]: name for name, config in WEBCAMS.items()
        }

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.camera_publishers = {
            name: self.create_publisher(
                CompressedImage,
                config["topic"],
                camera_qos,
            )
            for name, config in WEBCAMS.items()
        }

        self.workers = {
            name: CameraWorker(
                node=self,
                camera_name=name,
                config=config,
                publisher=self.camera_publishers[name],
            )
            for name, config in WEBCAMS.items()
        }

        self.command_sub = self.create_subscription(
            String,
            "/camera_manager/command",
            self.command_callback,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/camera_manager/status",
            10,
        )

        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.log_timer = self.create_timer(5.0, self.print_camera_counts)

        self.autostart_done = False
        self.autostart_timer = self.create_timer(
            2.0,
            self.autostart_callback,
        )

        self.get_logger().info("Camera Manager baslatildi.")
        self.get_logger().info(
            "1=web1, 2=web2, 4=web4 | "
            "BEST_EFFORT + KEEP_LAST + depth=1"
        )

    def autostart_callback(self):
        if self.autostart_done:
            return

        self.autostart_done = True
        self.autostart_timer.cancel()
        self.get_logger().info(
            f"Acilista otomatik baslatiliyor: {AUTO_START_CAMERAS}"
        )

        for name in AUTO_START_CAMERAS:
            self.start_camera(name)
            time.sleep(0.2)

    def start_camera(self, name):
        worker = self.workers.get(name)

        if worker is None:
            self.get_logger().warn(f"Bilinmeyen kamera adi: {name}")
            return

        if worker.is_running():
            self.get_logger().info(f"{name} zaten acik.")
            return

        self.get_logger().info(f"{name} baslatiliyor...")
        worker.start()

    def stop_camera(self, name):
        worker = self.workers.get(name)

        if worker is None:
            self.get_logger().warn(f"Bilinmeyen kamera adi: {name}")
            return

        if not worker.is_running():
            self.get_logger().info(f"{name} zaten kapali.")
            return

        self.get_logger().info(f"{name} kapatiliyor...")
        worker.stop()
        self.get_logger().info(f"{name} kapatildi.")

    def toggle_camera(self, name):
        worker = self.workers.get(name)

        if worker is None:
            self.get_logger().warn(f"Bilinmeyen kamera adi: {name}")
        elif worker.is_running():
            self.stop_camera(name)
        else:
            self.start_camera(name)

    def command_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f"Komut geldi: {command}")

        if command in self.key_to_camera:
            self.toggle_camera(self.key_to_camera[command])
            return

        for prefix, action in (
            ("toggle ", self.toggle_camera),
            ("on ", self.start_camera),
            ("off ", self.stop_camera),
        ):
            if command.startswith(prefix):
                name = command[len(prefix):].strip()
                action(name)
                return

        if command == "all_off":
            self.stop_all()
            return

        self.get_logger().warn(f"Bilinmeyen komut: {command}")

    def publish_status(self):
        # Arayuz uyumlulugu icin ON/OFF bicimi korunuyor.
        states = [
            f"{name}:{'ON' if worker.is_running() else 'OFF'}"
            for name, worker in self.workers.items()
        ]
        message = String()
        message.data = ",".join(states)
        self.status_pub.publish(message)

    def print_camera_counts(self):
        status_parts = []

        for name, worker in self.workers.items():
            if not worker.is_running():
                continue

            state = "RECONNECTING" if worker.reconnecting else "ON"
            age = (
                time.monotonic() - worker.last_publish_time
                if worker.last_publish_time > 0.0
                else -1.0
            )
            status_parts.append(
                f"{name}:{worker.published_count} "
                f"state={state} reconnect={worker.reconnect_count} "
                f"last_age={age:.1f}s"
            )

        if status_parts:
            self.get_logger().info(
                "Yayin durumu | " + ", ".join(status_parts)
            )

    def stop_all(self):
        for name in reversed(self.valid_cameras):
            if self.workers[name].is_running():
                self.stop_camera(name)

    def destroy_node(self):
        self.get_logger().info("Camera Manager kapatiliyor...")
        self.stop_all()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
