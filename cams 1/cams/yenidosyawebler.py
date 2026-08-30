#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rclpy.qos import HistoryPolicy

import cv2
import numpy as np
import threading
import time


# =================================================
# KAMERA TANIMLARI
# 3 Logitech webcam. Tuslar:
#   1 -> web1, 2 -> web2, 4 -> web4
# camera_manager.py ile ayni isimler kullanilmali.
# =================================================
CAMERAS = ["web1", "web2", "web4"]

CAMERA_LABELS = {
    "web1": "CAM 1 - WEB 1",
    "web2": "CAM 2 - WEB 2",
    "web4": "CAM 4 - WEB 4",
}

CAMERA_TOPICS = {
    "web1": "/cam1/image_compressed",
    "web2": "/cam2/image_compressed",
    "web4": "/cam4/image_compressed",
}

# Viewer tarafinda gosterilen kisa placeholder metni.
CAMERA_SHORT = {
    "web1": "WEB 1",
    "web2": "WEB 2",
    "web4": "WEB 4",
}

# Tus -> kamera esleme (cv2 key kodu ile).
KEY_TO_CAMERA = {
    ord('1'): "web1",
    ord('2'): "web2",
    ord('4'): "web4",
}

# camera_manager komut esleme (viewer tusu -> manager komutu).
CAMERA_TO_COMMAND = {
    "web1": "1",
    "web2": "2",
    "web4": "4",
}


class RoverCameraViewer(Node):
    def __init__(self):
        super().__init__('rover_camera_viewer_node')

        # =================================================
        # PANEL BOYUTLARI
        # =================================================
        self.W = 640
        self.H = 480

        # Kaynak IP akisi 15 FPS iken 30 FPS cizmek fayda saglamaz; sadece
        # yer istasyonu CPU'sunu yorar. Gerektiginde parametreyle artirilir.
        self.declare_parameter('ui_fps', 20.0)
        self.ui_fps = max(1.0, float(self.get_parameter('ui_fps').value))

        # Viewer'in kendi throttle tavani. Yayincilar (webcam/ip publisher)
        # fps'i zaten sinirliyor, o yuzden buradaki ikinci bir dusuk tavan
        # gereksiz gecikme ekler ve gelen fps'i aliaslayip dusurur (orn.
        # 10 fps gelen IP, 8 tavaninda ~5 fps'e duserdi). Tavani yayin
        # hizinin uzerine cekiyoruz ki her kare geldigi an islensin ->
        # manuel surus icin en dusuk gecikme. Bant maliyeti yok, sadece
        # PC biraz daha decode eder (kucuk JPEG'ler icin onemsiz).
        self.cam_fps = 30.0
        self.cam_update_period = 1.0 / self.cam_fps

        # Her kamera icin son decode zamani (throttle).
        self.last_update_time = {name: 0.0 for name in CAMERAS}

        # =================================================
        # TESHIS: panel basina FPS ve donma (stale) takibi
        # FPS icin: her gelen mesajda sayac artar; process_loop
        # saniyede bir sayaci okuyup FPS'e cevirir. Bu, aga gelen
        # gercek frame hizini gosterir; "takiliyor mu" sorusunu
        # sayiya dokerek teshise yardimci olur.
        # Donma icin: son frame'in geldigi duvar-saati tutulur.
        # Kamera ON ama X saniyedir yeni frame yoksa "DONDU" yazilir.
        # =================================================
        self.frame_counter = {name: 0 for name in CAMERAS}
        self._fps_last_counter = {name: 0 for name in CAMERAS}
        self._fps_last_time = time.time()
        self.display_fps = {name: 0.0 for name in CAMERAS}

        # Son frame'in ulastigi an (wall clock). 0.0 = henuz gelmedi.
        self.last_frame_wall = {name: 0.0 for name in CAMERAS}
        # Kamera ON iken bu kadar saniye yeni frame gelmezse donmus say.
        self.stale_seconds = 2.0

        self.running = True
        self.frame_lock = threading.Lock()

        # Kamera açık/kapalı durumları
        self.camera_states = {name: False for name in CAMERAS}

        # İlk açılışta hepsi kapalı görünsün
        self.images = {}
        for name in CAMERAS:
            self.images[name] = self.get_placeholder(
                f"{CAMERA_SHORT[name]} KAPALI"
            )

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
        # CALLBACK GROUPLARI
        # Her abonelik/timer ayrı grupta olmali, yoksa hepsi
        # varsayilan tek gruba dusup MultiThreadedExecutor'a
        # ragmen sirayla (birbirini bekleyerek) calisir. Bu da
        # ornegin IP kamera yogunken diger kameralarin ve
        # arayuzun (process_loop) kilitlenmesine yol acar.
        # =================================================
        self.cam_cbg = {name: MutuallyExclusiveCallbackGroup() for name in CAMERAS}
        self.status_cbg = MutuallyExclusiveCallbackGroup()

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
            cmd_qos,
            callback_group=self.status_cbg
        )

        # =================================================
        # GÖRÜNTÜ TOPICLERİ
        # Her kamera icin ayri abonelik + ayri callback group.
        # =================================================
        for name in CAMERAS:
            self.create_subscription(
                CompressedImage,
                CAMERA_TOPICS[name],
                self.make_image_cb(name),
                image_qos,
                callback_group=self.cam_cbg[name]
            )

        # =================================================
        # ARAYÜZ
        # =================================================
        self.win_name = "Yildiz Rover Camera Viewer - 3 LOGITECH"

        cv2.namedWindow(self.win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.win_name, 1440, 540)

        self.get_logger().info(
            f"Rover camera viewer baslatildi (arayuz: {self.ui_fps:.0f} FPS)."
        )
        self.get_logger().info("Tuslar:")
        self.get_logger().info("1 -> WEB 1 ac/kapat")
        self.get_logger().info("2 -> WEB 2 ac/kapat")
        self.get_logger().info("4 -> WEB 4 ac/kapat")
        self.get_logger().info("q -> Hepsini kapat ve cik")

        for name in CAMERAS:
            self.get_logger().info(f"{name} topic: {CAMERA_TOPICS[name]}")


        # OpenCV/Qt pencere islemleri ana is parcaciginda calismalidir.
        # Bu nedenle process_loop bir ROS timer'i olarak calistirilmaz;
        # main() icindeki run_gui_loop onu ana thread'de yeniler.
        # GUI dongusu, MultiThreadedExecutor.spin_once() ile ROS
        # callback'lerini de duzenli olarak dispatch eder.

    # =================================================
    # CAMERA MANAGER STATUS
    # =================================================
    def camera_status_cb(self, msg):
        """
        Beklenen format:
        web1:ON,web2:OFF,web4:ON
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
                    # Yeni acildi: eski frame zamanini sifirla ki ilk frame
                    # gelene kadar "DONDU" degil "ACILIYOR" gorunsun.
                    self.last_frame_wall[name] = 0.0

    # =================================================
    # KOMUT GÖNDERME
    # =================================================
    def send_camera_command(self, key):
        if key not in KEY_TO_CAMERA:
            return

        name = KEY_TO_CAMERA[key]
        command = CAMERA_TO_COMMAND[name]

        msg = String()
        msg.data = command
        self.camera_cmd_pub.publish(msg)

        # Hemen kullanıcıya görsel tepki ver
        with self.frame_lock:
            if self.camera_states.get(name, False):
                self.set_camera_placeholder_locked(name, "KAPATILIYOR...")
            else:
                self.set_camera_placeholder_locked(name, "ACILIYOR...")
                # Yeni aciyoruz: eski frame zamani "DONDU" tetiklemesin.
                self.last_frame_wall[name] = 0.0

        self.get_logger().info(f"{name} icin komut gonderildi: {command}")

    def send_all_off(self):
        msg = String()
        msg.data = "all_off"
        self.camera_cmd_pub.publish(msg)

        with self.frame_lock:
            for name in CAMERAS:
                self.images[name] = self.get_placeholder(
                    f"{CAMERA_SHORT[name]} KAPALI"
                )

        self.get_logger().info("Tum kameralari kapatma komutu gonderildi.")

    # =================================================
    # PLACEHOLDER YARDIMCI
    # =================================================
    def set_camera_placeholder_locked(self, name, state_text):
        if name in self.images:
            self.images[name] = self.get_placeholder(
                f"{CAMERA_SHORT[name]} {state_text}"
            )

    # =================================================
    # ORTAK FRAME İŞLEME
    # =================================================
    def decode_resize_compressed(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            raise RuntimeError("Compressed image decode edilemedi.")

        # Farkli en-boy oranlarini (orn. IP kanal 2: 4:3) esnetmeden
        # panele sigdir. Bos alanlar siyah kalir; goruntunun geometrisi
        # korunur ve hedef tespiti/operator algisi bozulmaz.
        src_h, src_w = frame.shape[:2]
        scale = min(self.W / src_w, self.H / src_h)
        dst_w = max(1, round(src_w * scale))
        dst_h = max(1, round(src_h * scale))
        resized = cv2.resize(frame, (dst_w, dst_h))

        panel = np.zeros((self.H, self.W, 3), dtype=np.uint8)
        x = (self.W - dst_w) // 2
        y = (self.H - dst_h) // 2
        panel[y:y + dst_h, x:x + dst_w] = resized
        return panel

    # =================================================
    # CALLBACK URETICI
    # Her kamera icin ayni mantikta callback uretilir.
    # =================================================
    def make_image_cb(self, name):
        def _cb(msg):
            now = time.time()

            # Gelen her mesaji say (throttle'dan ONCE). Bu sayac aga
            # ulasan gercek frame hizini yansitir; son frame zamanini
            # da guncelle ki donma tespiti dogru calissin.
            self.frame_counter[name] += 1
            self.last_frame_wall[name] = now

            if now - self.last_update_time[name] < self.cam_update_period:
                return

            self.last_update_time[name] = now

            try:
                frame = self.decode_resize_compressed(msg)

                with self.frame_lock:
                    self.images[name] = frame.copy()
                    self.camera_states[name] = True

            except Exception as e:
                self.get_logger().error(f"{name} kamera hatasi: {e}")

                with self.frame_lock:
                    self.images[name] = self.get_placeholder(
                        f"{CAMERA_SHORT[name]} HATA"
                    )

        return _cb

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

    def put_label(self, frame, text, state_on, fps=0.0, stale=False, age=0.0):
        frame = frame.copy()

        # Ust bilgi seridi (isim + durum + FPS).
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
            0.6,
            (0, 255, 0) if state_on else (0, 0, 255),
            2
        )

        # Sag ust kose: FPS (sadece kamera ON iken gosterilir).
        # Yesil = akici, sari = dusuk, kirmizi = neredeyse yok.
        if state_on:
            if fps >= 5.0:
                fps_color = (0, 255, 0)
            elif fps >= 1.0:
                fps_color = (0, 255, 255)
            else:
                fps_color = (0, 0, 255)

            fps_text = f"{fps:.0f} FPS"
            (tw, _), _ = cv2.getTextSize(
                fps_text, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2
            )
            cv2.putText(
                frame,
                fps_text,
                (self.W - tw - 12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                fps_color,
                2
            )

        # Donma uyarisi: kamera ON ama frame akmiyor. Operatorun
        # hangi kamerayi kapatacagina karar vermesine yardimci olur.
        if stale:
            overlay = frame.copy()
            cv2.rectangle(
                overlay,
                (0, self.H // 2 - 26),
                (self.W, self.H // 2 + 14),
                (0, 0, 120),
                -1
            )
            cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

            cv2.putText(
                frame,
                f"GORUNTU DONDU  {age:.0f}s",
                (20, self.H // 2 + 4),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        return frame

    def get_help_panel(self):
        img = np.zeros((self.H, self.W, 3), dtype=np.uint8)

        lines = [
            "KONTROL",
            "1: WEB 1 ON/OFF",
            "2: WEB 2 ON/OFF",
            "4: WEB 4 ON/OFF",
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
    def run_gui_loop(self, executor):
        """GUI'yi ana thread'de, ROS callback'lerini paralel olarak calistir."""
        target_period = 1.0 / self.ui_fps
        next_draw = time.monotonic()

        while self.running and rclpy.ok():
            # GUI'nin bir sonraki cizimine kadar ROS event'lerini isle.
            # Kisa timeout, pencere tuslarinin ve guncellemesinin tepkisel
            # kalmasini saglar; callback'ler MultiThreadedExecutor'un kendi
            # worker thread'lerinde paralel calisir.
            until_draw = next_draw - time.monotonic()
            executor.spin_once(timeout_sec=max(0.0, min(0.01, until_draw)))

            now = time.monotonic()
            if now < next_draw:
                continue

            self.process_loop()
            next_draw += target_period

            # Cizim hedef periyodun gerisine duserse eski takvimi birak;
            # birikmis cizim yaparak gecikmeyi daha da artirma.
            if next_draw < now:
                next_draw = now + target_period

    def process_loop(self):
        try:
            now = time.time()

            # Saniyede bir panel FPS'lerini guncelle (sayac farki / gecen sure).
            dt = now - self._fps_last_time
            if dt >= 1.0:
                for name in CAMERAS:
                    c = self.frame_counter[name]
                    self.display_fps[name] = (c - self._fps_last_counter[name]) / dt
                    self._fps_last_counter[name] = c
                self._fps_last_time = now

            with self.frame_lock:
                imgs = {name: self.images[name].copy() for name in CAMERAS}
                states = {name: self.camera_states[name] for name in CAMERAS}

            frames = {}
            for name in CAMERAS:
                last_wall = self.last_frame_wall[name]
                age = now - last_wall if last_wall > 0 else 0.0
                stale = (
                    states[name]
                    and last_wall > 0
                    and age > self.stale_seconds
                )

                frames[name] = self.put_label(
                    imgs[name],
                    CAMERA_LABELS[name],
                    states[name],
                    fps=self.display_fps[name],
                    stale=stale,
                    age=age
                )

            combined = cv2.hconcat([
                frames["web1"],
                frames["web2"],
                frames["web4"],
            ])

            cv2.imshow(self.win_name, combined)

            key = cv2.waitKey(1) & 0xFF

            if key in KEY_TO_CAMERA:
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
        node.run_gui_loop(executor)

    except KeyboardInterrupt:
        node.send_all_off()

    finally:
        executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()