#!/usr/bin/env python3
"""
300° Panoramic Photo Capture — ROS2 Versiyonu (Tarihli Otomatik Kayıt & Alt Panel)
RAW karelere de pusula + ölçek + bilgi paneli eklenmiş sürüm
(Kasma/donma sorunları ve örtüşme hataları çözüldü)
"""

import os
import datetime
import cv2
import numpy as np
import time
import threading

# --- ROS2 IMPORTS ---
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Float32

# =========================
# AYARLAR (SETTINGS)
# =========================
CONFIG = {
    "N_FRAMES": 5,
    "SPAN_DEG": 300,
    "START_HEADING": 0,
    "TICK_STEP": 30,

    "BLUR_MIN": 15.0,

    "OVERLAP_IDEAL": 0.35,
    "OVERLAP_MIN": 0.15,
    "OVERLAP_MAX": 0.60,

    "STITCH_WORK_W": 900,
    "MAX_OUT_W": 12000,
    "MAX_OUT_H": 5000,

    "MOTION_THRESH": 5.0,
    "STABLE_NEED": 2,
    "AUTO_TIMEOUT": 20.0,

    "CAPTURE_DIR": "captures",
    "RAW_DIR": os.path.join("captures", "raw"),
    "SINGLE_DIR": os.path.join("captures", "single"),

    "SCALE_PX_PER_M": 300,
    "SCALE_BAR_THICK_PER_M": 10,
    "SCALE_BAR_MIN_THICK": 6,
    "SCALE_BAR_MAX_THICK": 28,
}

SCALE_LENGTH_M = 0.5


# =========================
# ROS2 NODE
# =========================
current_heading = 0.0
class PanoramaCameraSubscriber(Node):
    def __init__(self):
        super().__init__('panorama_capture_node')

        self.latest_frame = None
        self.frame_counter = 0

        self.create_subscription(
            CompressedImage,
            '/webcam/image_compressed',
            self.image_callback,
            qos_profile_sensor_data
        )
        
        self.create_subscription(
            Float32,
            '/rover_heading',
            self.heading_callback,
            10
        )

        self.get_logger().info("Abone olundu: /webcam/image_compressed ve /rover_heading")

    def heading_callback(self, msg):
        global current_heading
        current_heading = msg.data

    def image_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        raw_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if raw_frame is None:
            self.get_logger().warning("Compressed image decode edilemedi.")
            return

        # Hizi artirmak ve donmalari engellemek icin cozunurlugu hemen dusuruyoruz
        self.latest_frame = cv2.resize(raw_frame, (960, 540))
        self.frame_counter += 1


# =========================
# YARDIMCI FONKSIYONLAR
# =========================
def ensure_dirs():
    os.makedirs(CONFIG["RAW_DIR"], exist_ok=True)
    os.makedirs(CONFIG["CAPTURE_DIR"], exist_ok=True)
    os.makedirs(CONFIG["SINGLE_DIR"], exist_ok=True)


def get_gray(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)


def blur_score(img):
    return float(cv2.Laplacian(get_gray(img), cv2.CV_64F).var())


def motion_score(prev_g, cur_g):
    if prev_g is None:
        return 999.0

    return float(np.mean(cv2.absdiff(prev_g, cur_g)))


def put_text(img, text, org, scale=0.7, thick=2, color=(255, 255, 255)):
    cv2.putText(
        img,
        text,
        org,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        (0, 0, 0),
        thick + 2,
        cv2.LINE_AA
    )

    cv2.putText(
        img,
        text,
        org,
        cv2.FONT_HERSHEY_SIMPLEX,
        scale,
        color,
        thick,
        cv2.LINE_AA
    )


def safe_resize(img, max_w, max_h):
    h, w = img.shape[:2]

    if w <= max_w and h <= max_h:
        return img

    scale = min(max_w / float(w), max_h / float(h))

    return cv2.resize(
        img,
        None,
        fx=scale,
        fy=scale,
        interpolation=cv2.INTER_AREA
    )


def apply_clahe(img):
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)

    l_enh = cv2.createCLAHE(
        clipLimit=2.5,
        tileGridSize=(8, 8)
    ).apply(l)

    return cv2.cvtColor(cv2.merge([l_enh, a, b]), cv2.COLOR_LAB2BGR)


# =========================
# OVERLAP & HUD
# =========================
def estimate_overlap(img_a, img_b):
    try:
        ga = get_gray(img_a)
        gb = get_gray(img_b)

        scale = 800.0 / max(ga.shape[1], 1)

        if scale < 1.0:
            ga = cv2.resize(ga, None, fx=scale, fy=scale)
            gb = cv2.resize(gb, None, fx=scale, fy=scale)

        orb = cv2.ORB_create(nfeatures=1500)

        kp1, des1 = orb.detectAndCompute(ga, None)
        kp2, des2 = orb.detectAndCompute(gb, None)

        if des1 is None or des2 is None or len(kp1) < 10 or len(kp2) < 10:
            return -1.0

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        matches = sorted(
            bf.match(des1, des2),
            key=lambda m: m.distance
        )

        good = [m for m in matches if m.distance < 64]

        if len(good) < 4:
            return 0.0

        shifts = [
            kp1[m.queryIdx].pt[0] - kp2[m.trainIdx].pt[0]
            for m in good
        ]

        median_shift = float(np.median(shifts))

        valid_shifts = [
            s for s in shifts
            if abs(s - median_shift) < 100
        ]

        if not valid_shifts:
            return -1.0

        final_shift = float(np.median(valid_shifts))
        overlap = 1.0 - abs(final_shift) / ga.shape[1]

        return float(np.clip(overlap, 0.0, 1.0))

    except Exception:
        return -1.0


def get_overlap_status(frames):
    if len(frames) < 2:
        return "Overlap: -", (200, 200, 200), -1.0

    ov = estimate_overlap(frames[-2], frames[-1])

    if ov < 0:
        return "Overlap: ?", (200, 200, 200), ov

    if ov < CONFIG["OVERLAP_MIN"]:
        return f"UYARI: Az overlap ({ov:.0%})", (0, 0, 255), ov

    if ov > CONFIG["OVERLAP_MAX"]:
        return f"UYARI: Fazla overlap ({ov:.0%})", (0, 165, 255), ov

    return f"Overlap: {ov:.0%} (OK)", (0, 255, 0), ov


def draw_hud(disp, frames, blur, ov_msg, ov_color, is_auto=False, auto_stb=0):
    h, w = disp.shape[:2]

    n_cap = len(frames)
    n_tot = CONFIG["N_FRAMES"]

    cv2.rectangle(disp, (0, 0), (w, 50), (30, 30, 30), -1)

    prog = n_cap / n_tot if n_tot > 0 else 0
    bar_w = w - 40
    fill_col = (0, 220, 80) if prog < 0.7 else (0, 180, 255)

    cv2.rectangle(disp, (20, 12), (20 + bar_w, 28), (80, 80, 80), -1)

    if prog > 0:
        cv2.rectangle(
            disp,
            (20, 12),
            (20 + int(bar_w * prog), 28),
            fill_col,
            -1
        )

    put_text(
        disp,
        f"Kare: {n_cap}/{n_tot} | {int(n_cap * (CONFIG['SPAN_DEG'] / n_tot))} derece",
        (20, 48),
        0.55,
        1
    )

    cv2.rectangle(disp, (0, h - 90), (w, h), (30, 30, 30), -1)

    b_ok = blur >= CONFIG["BLUR_MIN"]

    put_text(
        disp,
        f"Netlik: {int(blur)}" + (" OK" if b_ok else " BULANIK!"),
        (20, h - 55),
        0.55,
        1,
        (0, 255, 0) if b_ok else (0, 0, 255)
    )

    put_text(
        disp,
        ov_msg,
        (250, h - 55),
        0.55,
        1,
        ov_color
    )

    if is_auto:
        stab_bar = "=" * auto_stb + "." * (CONFIG["STABLE_NEED"] - auto_stb)

        put_text(
            disp,
            f"OTO-CEKIM Stabilite: [{stab_bar}]",
            (20, h - 20),
            0.55,
            1,
            (0, 255, 255)
        )

    else:
        put_text(
            disp,
            "[B/SPACE] Cek | [S] Foto Kaydet | [A] Oto | [R] Sifirla | [D] Sil | [ENTER] Birlestir | [Q] Cikis",
            (20, h - 20),
            0.45,
            1,
            (220, 220, 220)
        )

    cx = w // 2
    cy = h // 2

    cv2.drawMarker(
        disp,
        (cx, cy),
        (255, 255, 255),
        cv2.MARKER_CROSS,
        40,
        1,
        cv2.LINE_AA
    )

    for i, fr in enumerate(frames[-8:]):
        th = cv2.resize(fr, (80, 50))

        y = 60 + i * 55
        x = w - 85
        y2 = min(y + 50, h)

        if y < h:
            disp[y:y2, x:x + 80] = th[:(y2 - y), :]

            cv2.putText(
                disp,
                str(n_cap - len(frames[-8:]) + i + 1),
                (x + 2, y + 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.35,
                (0, 255, 0),
                1
            )


def flash_screen(disp, window="Kamera — ROS2 AGI"):
    overlay = disp.copy()

    cv2.rectangle(
        overlay,
        (0, 0),
        (disp.shape[1], disp.shape[0]),
        (255, 255, 255),
        -1
    )

    cv2.addWeighted(overlay, 0.3, disp, 0.7, 0, disp)

    cv2.imshow(window, disp)
    cv2.waitKey(100)


# =========================
# PUSULA & OLCEK CIZIMLERI
# =========================
def draw_compass_rose(img, heading_deg):
    h, w = img.shape[:2]

    cx = w - 80
    cy = h - 60
    r = 40

    n_rad = np.radians(-90 - (heading_deg % 360.0))

    cv2.circle(img, (cx, cy), r, (100, 100, 100), 2, cv2.LINE_AA)

    def pts(a, l):
        return (
            int(cx + l * np.cos(a)),
            int(cy + l * np.sin(a))
        )

    cv2.fillPoly(
        img,
        [
            np.array([
                pts(n_rad + np.pi / 2, 8),
                pts(n_rad + np.pi, r - 15),
                pts(n_rad + np.pi * 1.5, 8)
            ])
        ],
        (220, 220, 220)
    )

    cv2.fillPoly(
        img,
        [
            np.array([
                pts(n_rad + np.pi / 2, 8),
                pts(n_rad, r - 15),
                pts(n_rad + np.pi * 1.5, 8)
            ])
        ],
        (0, 0, 200)
    )

    cv2.circle(img, (cx, cy), 4, (50, 50, 50), -1)

    cv2.putText(
        img,
        "N",
        pts(n_rad, r + 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255),
        2,
        cv2.LINE_AA
    )

    cv2.putText(
        img,
        f"{heading_deg:.1f} deg",
        pts(n_rad + np.pi, r + 25),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.4,
        (200, 200, 200),
        1,
        cv2.LINE_AA
    )


def draw_scale_bar(img):
    h, w = img.shape[:2]

    bx = 20
    by = h - 78

    bar_len = int(SCALE_LENGTH_M * CONFIG["SCALE_PX_PER_M"])
    max_bar_len = max(80, int(w * 0.35))
    bar_len = int(np.clip(bar_len, 60, max_bar_len))

    bar_thick = int(SCALE_LENGTH_M * CONFIG["SCALE_BAR_THICK_PER_M"])
    bar_thick = int(np.clip(
        bar_thick,
        CONFIG["SCALE_BAR_MIN_THICK"],
        CONFIG["SCALE_BAR_MAX_THICK"]
    ))

    cv2.rectangle(
        img,
        (bx - 2, by - 2),
        (bx + bar_len + 2, by + bar_thick + 2),
        (0, 0, 0),
        -1
    )

    cv2.rectangle(
        img,
        (bx, by),
        (bx + bar_len, by + bar_thick),
        (255, 255, 255),
        -1
    )

    cv2.line(img, (bx, by - 8), (bx, by + bar_thick + 8), (255, 255, 255), 2)
    cv2.line(img, (bx + bar_len, by - 8), (bx + bar_len, by + bar_thick + 8), (255, 255, 255), 2)

    put_text(img, "0", (bx - 2, by + bar_thick + 24), 0.4, 1)
    put_text(img, f"{SCALE_LENGTH_M:g} m", (bx + bar_len - 45, by + bar_thick + 24), 0.4, 1)


def annotate_frame(img, start_heading_deg, span_deg, idx, total):
    pad_bottom = 120

    out = cv2.copyMakeBorder(
        img,
        0,
        pad_bottom,
        0,
        0,
        cv2.BORDER_CONSTANT,
        value=(20, 20, 20)
    )

    if total > 1:
        heading = (start_heading_deg + (idx * (span_deg / (total - 1)))) % 360.0
    else:
        heading = start_heading_deg % 360.0

    draw_compass_rose(out, heading)
    
    draw_scale_bar(out)

    put_text(
        out,
        f"Frame {idx + 1}/{total} | Heading: {heading:.1f} deg",
        (20, out.shape[0] - 20),
        0.55,
        1
    )

    return out


def save_single_photo(frame):
    ensure_dirs()

    zaman = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    annotated = annotate_frame(
        frame,
        current_heading,
        0,
        0,
        1
    )

    single_path = os.path.join(
        CONFIG["SINGLE_DIR"],
        f"single_{zaman}.jpg"
    )

    kayit_basarili = cv2.imwrite(single_path, annotated)

    if kayit_basarili:
        print("\n[KAYDEDILDI] Tek kare fotograf suraya kaydedildi:")
        print(f"📂 {os.path.abspath(single_path)}\n")
    else:
        print("\n[HATA] Tek kare fotograf kaydedilemedi.\n")




# =========================
# KATI BIRLESTIRICI
# =========================
def stitch_horizontal_blended(frames):
    if not frames:
        return None

    if len(frames) == 1:
        return frames[0]

    canvas = frames[0].copy()

    orb = cv2.ORB_create(nfeatures=1500)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    for i in range(1, len(frames)):
        nxt_img = frames[i].copy()

        h1, w1 = canvas.shape[:2]
        h2, w2 = nxt_img.shape[:2]

        search_w = min(w1, int(w2 * 1.5))
        canvas_crop = canvas[:, -search_w:]

        kp1, des1 = orb.detectAndCompute(canvas_crop, None)
        kp2, des2 = orb.detectAndCompute(nxt_img, None)

        dx = int(w1 - w2 * (1.0 - CONFIG["OVERLAP_IDEAL"]))

        if des1 is not None and des2 is not None and len(kp1) > 5 and len(kp2) > 5:
            matches = sorted(
                bf.match(des2, des1),
                key=lambda x: x.distance
            )

            good = [m for m in matches if m.distance < 45]

            if len(good) >= 5:
                dxs = []

                for m in good:
                    pt_canvas = kp1[m.trainIdx].pt
                    pt_nxt = kp2[m.queryIdx].pt

                    real_x_canvas = pt_canvas[0] + (w1 - search_w)
                    dxs.append(real_x_canvas - pt_nxt[0])

                calc_dx = int(np.median(dxs))

                if w1 - w2 * 0.9 < calc_dx < w1 + w2 * 0.1:
                    dx = calc_dx

        new_w = max(w1, dx + w2)
        new_h = max(h1, h2)

        new_canvas = np.zeros((new_h, new_w, 3), dtype=np.uint8)

        overlap_start = max(0, dx)
        overlap_end = min(w1, dx + w2)
        overlap_w = overlap_end - overlap_start

        new_canvas[0:h1, 0:w1] = canvas

        if dx + w2 > w1:
            new_canvas[0:h2, w1:dx + w2] = nxt_img[0:h2, w1 - dx:w2]

        if overlap_w > 0:
            blend_band = min(40, overlap_w)

            mid = overlap_w // 2
            start_b = mid - blend_band // 2
            end_b = start_b + blend_band

            alpha = np.zeros((1, overlap_w, 1), dtype=np.float32)

            alpha[0, :start_b, 0] = 1.0
            alpha[0, start_b:end_b, 0] = np.linspace(1, 0, blend_band)
            alpha[0, end_b:, 0] = 0.0

            h_min = min(h1, h2)

            left_part = canvas[0:h_min, overlap_start:overlap_end].astype(np.float32)
            right_part = nxt_img[0:h_min, overlap_start - dx:overlap_end - dx].astype(np.float32)

            blended = (
                left_part * alpha +
                right_part * (1.0 - alpha)
            ).astype(np.uint8)

            new_canvas[0:h_min, overlap_start:overlap_end] = blended

        canvas = new_canvas.copy()

    return canvas


# =========================
# ANA DONGU
# =========================
def main():
    global SCALE_LENGTH_M

    print("\n" + "=" * 40)
    print("   PANORAMA CEKIM ARACI (ROS2 - TARIHLI KAYIT)")
    print("=" * 40)

    try:
        val = input(f"Referans olcek cubugu kac metre? [{SCALE_LENGTH_M}]: ")

        if val.strip():
            SCALE_LENGTH_M = float(val)

    except Exception:
        print("[UYARI] Olcek degeri okunamadi. Varsayilan kullaniliyor.")

    print("\n[INFO] ROS Agindan Pusula ve Kamera bekleniyor...")

    main_win = "Kamera — ROS2 AGI"

    cv2.namedWindow(main_win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(main_win, 1280, 720)

    rclpy.init()
    node = PanoramaCameraSubscriber()

    wait_screen = np.zeros((720, 1280, 3), dtype=np.uint8)

    put_text(
        wait_screen,
        "ROS2 Agindan Goruntu Bekleniyor...",
        (350, 360),
        1.0,
        2,
        (0, 255, 255)
    )

    cv2.imshow(main_win, wait_screen)
    cv2.waitKey(1)

    frames = []

    print("[INFO] Goruntu bekleniyor...")

    last_main_id = -1
    bs = 0.0
    ov_msg = "Overlap: -"
    ov_color = (200, 200, 200)

    disp = None
    live = None

    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.01)

            if node.latest_frame is None:
                cv2.waitKey(10)
                continue

            if node.frame_counter != last_main_id:
                last_main_id = node.frame_counter

                live = node.latest_frame.copy()

                bs = blur_score(live)
                ov_msg, ov_color, _ = get_overlap_status(frames + [live])

                disp = cv2.resize(live, (1280, 720))

                draw_hud(disp, frames, bs, ov_msg, ov_color)

                cv2.imshow(main_win, disp)

            key = cv2.waitKey(10) & 0xFF

            if key == 27 or key == ord('q'):
                print("\n[INFO] Cikis yapiliyor...")
                break

            elif key in (ord('r'), ord('R')):
                frames.clear()
                print("\n[INFO] Sifirlandi.")

            elif key in (ord('d'), ord('D')) and frames:
                frames.pop()
                print(f"[INFO] Son kare silindi. Kalan: {len(frames)}")

            elif key == ord('s'):
                if live is None:
                    print("[UYARI] Kaydedilecek goruntu yok.")
                    continue

                save_single_photo(live.copy())

                if disp is not None:
                    flash_screen(disp, main_win)

            elif key in (ord('b'), ord('B'), ord(' ')):
                if len(frames) >= CONFIG["N_FRAMES"]:
                    print(f"[BILGI] Zaten {CONFIG['N_FRAMES']} kare cektiniz, ENTER ile birlestirin.")
                    continue

                if bs < CONFIG["BLUR_MIN"]:
                    print(f"[UYARI] Goruntu bulanik! (Skor: {bs:.1f} < Limit: {CONFIG['BLUR_MIN']}).")
                    continue

                if len(frames) == 0:
                    CONFIG["START_HEADING"] = current_heading
                    print(f"[BILGI] Ilk kare cekildi. Baslangic pusula acisi: {current_heading}° olarak ayarlandi.")

                frames.append(live)

                if disp is not None:
                    flash_screen(disp, main_win)

                print(f"[OK] Kare {len(frames)}/{CONFIG['N_FRAMES']} cekildi. {get_overlap_status(frames)[0]}")

            elif key in (ord('a'), ord('A')):
                frames.clear()

                print("[AUTO] Oto-cekim basliyor... Yavasca kamerayi cevirin.")

                prev_auto_g = None
                i = 0
                seq = 0
                last_auto_id = -1

                while i < CONFIG["N_FRAMES"]:
                    rclpy.spin_once(node, timeout_sec=0.01)

                    if node.latest_frame is None:
                        cv2.waitKey(10)
                        continue

                    if node.frame_counter == last_auto_id:
                        if (cv2.waitKey(10) & 0xFF) in (27, ord('q')):
                            break

                        continue

                    last_auto_id = node.frame_counter

                    fr = node.latest_frame.copy()
                    cur_g = get_gray(fr)

                    ms = motion_score(prev_auto_g, cur_g)
                    b = blur_score(fr)

                    prev_auto_g = cur_g

                    stable = (ms < CONFIG["MOTION_THRESH"] and b >= CONFIG["BLUR_MIN"])

                    if stable:
                        seq += 1
                    else:
                        seq = 0

                    adisp = cv2.resize(fr, (1280, 720))

                    om, oc, _ = get_overlap_status(frames + [fr])

                    draw_hud(
                        adisp,
                        frames,
                        b,
                        om,
                        oc,
                        is_auto=True,
                        auto_stb=min(seq, CONFIG["STABLE_NEED"])
                    )

                    cv2.imshow(main_win, adisp)

                    if seq >= CONFIG["STABLE_NEED"]:
                        if len(frames) == 0:
                            CONFIG["START_HEADING"] = current_heading
                            print(f"[BILGI] Ilk kare cekildi. Baslangic pusula acisi: {current_heading}° olarak ayarlandi.")

                        frames.append(fr)

                        flash_screen(adisp, main_win)

                        i += 1
                        seq = 0

                        print(f"  [OK] Kare {i}/{CONFIG['N_FRAMES']} cekildi.")

                        cv2.waitKey(500)

                    if (cv2.waitKey(30) & 0xFF) in (27, ord('q')):
                        break

            elif key == 13 and len(frames) >= 2:
                print("\n[INFO] Kati birlestirici calisiyor...")

                ensure_dirs()

                zaman = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

                run_raw_dir = os.path.join(
                    CONFIG["RAW_DIR"],
                    f"run_{zaman}"
                )

                os.makedirs(run_raw_dir, exist_ok=True)

                for idx, f in enumerate(frames):
                    annotated = annotate_frame(
                        f,
                        CONFIG["START_HEADING"],
                        CONFIG["SPAN_DEG"],
                        idx,
                        len(frames)
                    )

                    raw_path = os.path.join(
                        run_raw_dir,
                        f"raw_{idx + 1:02d}.jpg"
                    )

                    cv2.imwrite(raw_path, annotated)

                print(f"[OK] {len(frames)} adet ham fotograf annotated suraya kaydedildi:")
                print(f"📂 {run_raw_dir}")

                pano = stitch_horizontal_blended(frames)

                if pano is not None:
                    print("[OK] Kati Birlestirme BASARILI!")

                    pano_final = apply_clahe(
                        safe_resize(
                            pano,
                            CONFIG["MAX_OUT_W"],
                            CONFIG["MAX_OUT_H"]
                        )
                    )

                    pad_bottom = 120

                    pano_padded = cv2.copyMakeBorder(
                        pano_final,
                        0,
                        pad_bottom,
                        0,
                        0,
                        cv2.BORDER_CONSTANT,
                        value=(20, 20, 20)
                    )

                    p_disp = pano_padded.copy()

                    draw_compass_rose(p_disp, CONFIG["START_HEADING"])
                    draw_scale_bar(p_disp)

                    yeni_pano_isim = f"panorama_{zaman}.jpg"

                    pano_path_final = os.path.join(
                        CONFIG["CAPTURE_DIR"],
                        yeni_pano_isim
                    )

                    kayit_basarili = cv2.imwrite(
                        pano_path_final,
                        p_disp
                    )

                    if kayit_basarili:
                        print("\n[KAYDEDILDI] Panoramik fotograf suraya kaydedildi:")
                        print(f"📂 {os.path.abspath(pano_path_final)}\n")

                    while True:
                        pano_win = "Panorama Onizleme | ENTER: Kapat"

                        cv2.namedWindow(pano_win, cv2.WINDOW_NORMAL)
                        cv2.resizeWindow(pano_win, 1600, 600)

                        cv2.imshow(
                            pano_win,
                            safe_resize(p_disp, 1600, 900)
                        )

                        k2 = cv2.waitKey(0) & 0xFF

                        if k2 == 13:
                            cv2.destroyWindow(pano_win)
                            break

                else:
                    print("\n[HATA] Beklenmeyen bir hata olustu.")

    except KeyboardInterrupt:
        print("\n[INFO] Kapatiliyor...")

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()