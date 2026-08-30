#!/usr/bin/env bash
#
# Faz 1 - web1 icin elle RTSP yayin testi.
#
# Bu script ROS2'ye, camera_manager.py'ye veya mevcut calisan hatta HICBIR
# SEKILDE dokunmaz. Ayni web1 cihazini camera_manager de kullanmaya
# calisirsa "device busy" hatasi alinir; bu yuzden test sirasinda
# camera_manager'i durdurun (ya da web1'i onceden kapatin).
#
# Onkosul:
#   - ffmpeg kurulu olmali (V4L2 + libx264 destekli bir surum)
#   - MediaMTX ayni makinede calisiyor olmali:
#       ./mediamtx streaming_migration/mediamtx.yml
#
# Kullanim:
#   ./test_stream_web1.sh
#   (parametreleri degistirmek icin env degiskeni gecin, ornek:)
#   STREAM_WIDTH=1280 STREAM_HEIGHT=720 STREAM_FPS=15 ./test_stream_web1.sh
#
# Dogrulama (ayri bir terminalde):
#   ffplay rtsp://127.0.0.1:8554/web1
#
# NOT: Asagidaki cihaz yolu camera_manager.py'deki WEBCAMS["web1"]["device"]
# ile ayni (by-id, USB port degisse de sabit kalir). Sizin donanimda farkli
# olabilir, "ls -la /dev/v4l/by-id/" ile dogrulayin.

set -euo pipefail

DEVICE="${WEB1_DEVICE:-/dev/v4l/by-id/usb-046d_Brio_100_2523ZB739SU8-video-index0}"
WIDTH="${STREAM_WIDTH:-640}"
HEIGHT="${STREAM_HEIGHT:-480}"
FPS="${STREAM_FPS:-15}"
BITRATE="${STREAM_BITRATE:-1500k}"
RTSP_URL="${RTSP_URL:-rtsp://127.0.0.1:8554/web1}"

if [ ! -e "$DEVICE" ]; then
    echo "HATA: cihaz bulunamadi: $DEVICE" >&2
    echo "Gercek yolu bulmak icin: ls -la /dev/v4l/by-id/" >&2
    exit 1
fi

if ! command -v ffmpeg >/dev/null 2>&1; then
    echo "HATA: ffmpeg bulunamadi. Once kurun." >&2
    exit 1
fi

echo "web1 -> ${RTSP_URL}  (${WIDTH}x${HEIGHT}@${FPS}fps, hedef bitrate ${BITRATE})"
echo "GOP uzunlugu ~2 saniye (keyframe kurtarma ile bitrate verimliligi arasi denge)."
echo "Durdurmak icin Ctrl+C."
echo

# -f v4l2 -input_format mjpeg: kameradan MJPG istiyoruz (USB2 480Mbps bus'ta
#   ham YUYV yerine donanimsal sikistirilmis akis - camera_manager.py'nin
#   input_fourcc="MJPG" tercihiyle tutarli).
# -c:v libx264 -tune zerolatency: yazilimsal H.264, dusuk gecikme preset'i.
#   Donanim (NVENC) encoder'a gecis, JetPack/L4T dogrulandiktan sonra ayri
#   bir adim (Faz 7) - burada bilincli olarak yazilimsal baslanildi.
# -rtsp_transport tcp: paket kaybi kurtarmasini TCP'ye birakiyoruz (kendi
#   NACK/FEC mantigimizi yazmiyoruz ilk asamada).
exec ffmpeg -hide_banner -loglevel warning \
    -f v4l2 -input_format mjpeg -video_size "${WIDTH}x${HEIGHT}" -framerate "$FPS" \
    -i "$DEVICE" \
    -c:v libx264 -preset veryfast -tune zerolatency -b:v "$BITRATE" \
    -g $((FPS * 2)) \
    -f rtsp -rtsp_transport tcp \
    "$RTSP_URL"
