#!/usr/bin/env bash
#
# Faz 1b - GStreamer + JETSON DONANIM ENCODER ile RTSP yayin testi.
#
# NEDEN BU SCRIPT (test_stream_web1.sh yerine):
#   agx_info.txt ciktisi gosterdi ki AGX'teki ffmpeg 4.4.2 sadece
#   "--enable-nvv4l2dec" ile derlenmis (sadece DECODE). Encoder listesinde
#   NVENC YOK, sadece libx264/libx265/mjpeg var.
#   => Donanim encoder'i kullanmak icin GStreamer sart.
#
# Dogrulanmis donanim (agx_info.txt):
#   JetPack 6.2.1 / L4T R36.4.7 / Ubuntu 22.04 / GStreamer 1.20.3
#   nvv4l2h264enc VAR, nvv4l2h265enc VAR, nvjpegdec VAR, nvvidconv VAR
#
# ONKOSULLAR:
#   1) Disk alani acilmis olmali (agx_info.txt'de kok bolum %98 doluydu)
#   2) rtspclientsink gerekli:
#        sudo apt install gstreamer1.0-rtsp
#      (agx_info.txt'de rtspclientsink YOK olarak raporlandi)
#   3) MediaMTX calisiyor olmali:
#        ./mediamtx streaming_migration/mediamtx.yml
#
# KULLANIM:
#   ./test_stream_gst_hw.sh
#
#   Kamera formatina gore:
#     CAM_FORMAT=mjpeg ./test_stream_gst_hw.sh     (webcam MJPG veriyorsa - tercih edilen)
#     CAM_FORMAT=yuyv  ./test_stream_gst_hw.sh     (webcam sadece YUYV veriyorsa)
#
#   Diger ayarlar:
#     DEVICE=/dev/video2 WIDTH=1280 HEIGHT=720 FPS=30 BITRATE=2000000 \
#       ./test_stream_gst_hw.sh
#
# DOGRULAMA (ayri terminalde):
#   ffplay -fflags nobuffer -flags low_delay rtsp://127.0.0.1:8554/web1
#
# NOT: Bu script mevcut ROS2 hattina DOKUNMAZ. Ayni kamerayi camera_manager
# de acmaya calisirsa "device busy" olur; test sirasinda o kamerayi kapatin.

set -uo pipefail

DEVICE="${DEVICE:-}"
CAM_FORMAT="${CAM_FORMAT:-mjpeg}"
WIDTH="${WIDTH:-1280}"
HEIGHT="${HEIGHT:-720}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-2000000}"          # bit/sn - 2 Mbps
RTSP_URL="${RTSP_URL:-rtsp://127.0.0.1:8554/web1}"

# GOP ~2 saniye. Kisa GOP = paket kaybindan hizli toparlanma,
# uzun GOP = daha iyi bitrate verimliligi. 2sn dengeli bir baslangic.
IFRAME_INTERVAL=$((FPS * 2))

# ---------- On kontroller ----------

if ! command -v gst-launch-1.0 >/dev/null 2>&1; then
    echo "HATA: gst-launch-1.0 bulunamadi." >&2
    exit 1
fi

for el in nvv4l2h264enc nvvidconv rtspclientsink; do
    if ! gst-inspect-1.0 "$el" >/dev/null 2>&1; then
        echo "HATA: GStreamer elementi bulunamadi: $el" >&2
        if [ "$el" = "rtspclientsink" ]; then
            echo "  Cozum: sudo apt install gstreamer1.0-rtsp" >&2
        fi
        exit 1
    fi
done

# Cihaz otomatik secimi: verilmediyse ZED olmayan ilk video cihazini bul.
if [ -z "$DEVICE" ]; then
    echo "DEVICE verilmedi, otomatik aranıyor (ZED haric)..."
    for link in /dev/v4l/by-id/*; do
        [ -e "$link" ] || continue
        case "$link" in
            *ZED*|*zed*) continue ;;
        esac
        DEVICE="$link"
        break
    done
fi

if [ -z "$DEVICE" ] || [ ! -e "$DEVICE" ]; then
    echo "HATA: kullanilabilir webcam bulunamadi." >&2
    echo "  Takili cihazlar:" >&2
    ls -la /dev/v4l/by-id/ 2>/dev/null >&2 || echo "  (by-id dizini bos)" >&2
    echo "  Webcam'lerin takili oldugundan emin olun, ya da DEVICE=... ile elle verin." >&2
    exit 1
fi

echo "Cihaz      : $DEVICE"
echo "Format     : $CAM_FORMAT"
echo "Cozunurluk : ${WIDTH}x${HEIGHT} @ ${FPS}fps"
echo "Bitrate    : $((BITRATE / 1000)) kbps (SABIT - nvv4l2h264enc runtime'da degistirilemiyor)"
echo "GOP        : ${IFRAME_INTERVAL} kare (~2 sn)"
echo "Hedef      : $RTSP_URL"
echo "Durdurmak icin Ctrl+C."
echo

# ---------- Pipeline ----------
#
# MJPEG yolu:  v4l2src -> nvjpegdec (DONANIM jpeg decode) -> nvvidconv -> nvv4l2h264enc
# YUYV  yolu:  v4l2src -> nvvidconv (NVMM'e tasi + format cevir) -> nvv4l2h264enc
#
# nvv4l2h264enc parametreleri:
#   control-rate=1     : CBR (RF linkte ongorulebilir bitrate icin)
#   preset-level=1     : UltraFast - en dusuk encode gecikmesi
#   maxperf-enable=1   : encoder clock'larini maksimumda tut
#   insert-sps-pps=1   : her IDR'de SPS/PPS gonder (gec katilan izleyici
#                        ve paket kaybi sonrasi toparlanma icin sart)
#   iframeinterval     : GOP uzunlugu

if [ "$CAM_FORMAT" = "mjpeg" ]; then
    SRC_CAPS="image/jpeg,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1"
    DECODE="nvjpegdec ! video/x-raw"
else
    SRC_CAPS="video/x-raw,format=YUY2,width=${WIDTH},height=${HEIGHT},framerate=${FPS}/1"
    DECODE="identity"
fi

exec gst-launch-1.0 -e \
    v4l2src device="$DEVICE" io-mode=2 ! \
    $SRC_CAPS ! \
    $DECODE ! \
    nvvidconv ! \
    'video/x-raw(memory:NVMM),format=NV12' ! \
    nvv4l2h264enc \
        bitrate="$BITRATE" \
        control-rate=1 \
        preset-level=1 \
        maxperf-enable=1 \
        insert-sps-pps=1 \
        iframeinterval="$IFRAME_INTERVAL" ! \
    h264parse config-interval=1 ! \
    rtspclientsink location="$RTSP_URL" protocols=tcp latency=0
