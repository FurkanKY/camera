#!/usr/bin/env bash
#
# AGX Orin uzerinde calistirilacak BILGI TOPLAMA scripti.
#
# HICBIR SEY DEGISTIRMEZ, HICBIR SERVIS BASLATMAZ/DURDURMAZ.
# Sadece okur ve ekrana yazar. Guvenle calistirilabilir.
#
# Kullanim:
#   chmod +x collect_agx_info.sh
#   ./collect_agx_info.sh > agx_info.txt 2>&1
#   (sonra agx_info.txt dosyasini paylasin)
#
# Amac: yeni video pipeline mimarisi icin gereken donanim/yazilim
# yeteneklerini tespit etmek. Ozellikle:
#   - Jetson donanim encoder (NVENC) var mi
#   - GStreamer WebRTC/WHIP destegi var mi   <-- EN KRITIK
#   - Kameralar hangi format/cozunurluk/fps destekliyor
#   - CPU/bellek/guc modu ne durumda

line() { echo; echo "==================== $* ===================="; }
try()  { echo "--- \$ $* ---"; "$@" 2>&1 || echo "(komut basarisiz veya bulunamadi)"; }

echo "AGX BILGI TOPLAMA - $(date)"
echo "Hostname: $(hostname 2>/dev/null || echo bilinmiyor)"

line "1. SISTEM / JETPACK / L4T"
try uname -a
echo "--- /etc/nv_tegra_release ---"
cat /etc/nv_tegra_release 2>/dev/null || echo "(dosya yok - Jetson olmayabilir)"
echo "--- /etc/os-release ---"
cat /etc/os-release 2>/dev/null | grep -E "PRETTY_NAME|VERSION" || echo "(okunamadi)"
echo "--- JetPack paketleri ---"
dpkg -l 2>/dev/null | grep -E "nvidia-l4t-core|nvidia-jetpack" || echo "(bulunamadi)"

line "2. GUC MODU / TERMAL (encoder ve CPU frekanslarini belirler)"
try nvpmodel -q
echo "--- jetson_clocks durumu ---"
jetson_clocks --show 2>/dev/null | head -20 || echo "(jetson_clocks yok veya yetki gerekiyor)"

line "3. CPU / BELLEK / DISK"
echo "CPU cekirdek sayisi: $(nproc 2>/dev/null || echo bilinmiyor)"
try free -h
echo "--- Disk (kok bolum) ---"
df -h / 2>/dev/null
echo "--- Anlik yuk ---"
uptime 2>/dev/null

line "4. GSTREAMER - TEMEL"
try gst-inspect-1.0 --version
echo "--- Toplam plugin sayisi ---"
gst-inspect-1.0 2>/dev/null | tail -3 || echo "(gstreamer yok)"

line "5. GSTREAMER - DONANIM ENCODER (Faz 7 karari)"
for el in nvv4l2h264enc nvv4l2h265enc nvjpegenc nvjpegdec nvv4l2decoder nvvidconv; do
    echo "--- $el ---"
    if gst-inspect-1.0 "$el" >/dev/null 2>&1; then
        echo "VAR"
        # bitrate property'si runtime'da degistirilebiliyor mu (adaptif bitrate icin sart)
        gst-inspect-1.0 "$el" 2>/dev/null | grep -A2 -E "^\s+bitrate" | head -6
    else
        echo "YOK"
    fi
done

line "6. GSTREAMER - WEBRTC / WHIP  <<< EN KRITIK BOLUM >>>"
echo "Bu bolum, gercek adaptif bitrate'li WebRTC pipeline'inin"
echo "mumkun olup olmadigini belirler."
for el in webrtcbin whipclientsink whipsink whepsrc whepclientsrc rtspclientsink x264enc; do
    echo -n "$el : "
    if gst-inspect-1.0 "$el" >/dev/null 2>&1; then echo "VAR"; else echo "YOK"; fi
done
echo "--- gst-plugins-rs (Rust) paketleri ---"
dpkg -l 2>/dev/null | grep -iE "gstreamer.*rs|gst-plugins-rs" || echo "(kurulu degil gorunuyor)"
echo "--- gst-plugins-bad (webrtcbin buradan gelir) ---"
dpkg -l 2>/dev/null | grep -E "gstreamer1.0-plugins-bad" || echo "(kurulu degil gorunuyor)"

line "7. FFMPEG (Faz 1 test scripti icin)"
try ffmpeg -version
echo "--- Ilgili encoder'lar ---"
ffmpeg -hide_banner -encoders 2>/dev/null | grep -iE "x264|x265|nvenc|mjpeg" || echo "(ffmpeg yok veya encoder bulunamadi)"
echo "--- V4L2 girdi destegi ---"
ffmpeg -hide_banner -devices 2>/dev/null | grep -i v4l2 || echo "(v4l2 girdi destegi gorunmuyor)"

line "8. KAMERALAR - CIHAZ LISTESI"
echo "--- /dev/v4l/by-id/ (sabit yollar - camera_manager.py bunlari kullaniyor) ---"
ls -la /dev/v4l/by-id/ 2>/dev/null || echo "(dizin yok)"
echo
echo "--- /dev/v4l/by-path/ ---"
ls -la /dev/v4l/by-path/ 2>/dev/null || echo "(dizin yok)"
echo
try v4l2-ctl --list-devices

line "9. KAMERALAR - DESTEKLENEN FORMATLAR (capture ayarlarini belirler)"
echo "Her kamera icin MJPG hangi cozunurluk/fps'lerde destekleniyor?"
echo "Bu, USB 2.0 bant genisligi butcesini dogrudan etkiler."
for dev in /dev/video*; do
    [ -e "$dev" ] || continue
    echo
    echo "########## $dev ##########"
    v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null | head -60 || echo "(okunamadi)"
done

line "10. USB TOPOLOJISI (480Mbps bus paylasimi)"
try lsusb -t
echo "--- Cihaz listesi ---"
lsusb 2>/dev/null || echo "(lsusb yok)"

line "11. AG / ROS 2"
echo "ROS_DISTRO: ${ROS_DISTRO:-<bos>}"
echo "ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-<bos/varsayilan>}"
echo "RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-<bos/varsayilan>}"
echo "--- Ag arayuzleri ---"
ip -br addr 2>/dev/null || ifconfig 2>/dev/null || echo "(okunamadi)"

line "12. MEVCUT CALISAN SUREÇLER (kamera ile ilgili)"
ps aux 2>/dev/null | grep -iE "ros2|camera_manager|ip_camera|mediamtx|ffmpeg|gst-launch" | grep -v grep || echo "(kamera ile ilgili calisan sureç yok)"

line "13. OTOMATIK BASLATMA (hangi launch acilista tetikleniyor?)"
echo "--- systemd (ros/camera ile ilgili) ---"
systemctl list-units --type=service --state=running 2>/dev/null | grep -iE "ros|camera" || echo "(ilgili servis yok)"
echo "--- Kullanici autostart satirlari ---"
grep -nE "ros2 launch|camera_manager|source .*setup.bash" ~/.bashrc ~/.profile 2>/dev/null || echo "(bulunamadi)"
echo "--- crontab ---"
crontab -l 2>/dev/null || echo "(crontab yok)"

line "TOPLAMA TAMAMLANDI"
echo "Bu ciktinin tamamini paylasin."
