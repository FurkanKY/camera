import time
from datetime import datetime
import random

def log_yaz(modul, mesaj):
    zaman_damgasi = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    print(f"[{zaman_damgasi}] [{modul}] {mesaj}")

def surekli_log_at(saniye, durum_mesajlari, ara_moduller=["NAVI", "MOTION", "SYSTEM", "TELEMETRY"]):
    """Belirtilen saniye boyunca terminali boş bırakmamak için sürekli ara loglar basar."""
    baslangic = time.time()
    while time.time() - baslangic < saniye:
        modul = random.choice(ara_moduller)
        mesaj = random.choice(durum_mesajlari)
        
        # Eğer mesajın içinde dinamik veri olması gerekiyorsa rastgele ekle
        if "Mesafe" in mesaj:
            mesaj += f" {round(random.uniform(2.0, 15.0), 2)}m"
        elif "Hız" in mesaj:
            mesaj += f" {round(random.uniform(1.0, 1.4), 2)} m/s"
            
        log_yaz(modul, mesaj)
        
        # Her log arasında 0.3 ile 1.2 saniye arası rastgele kısa bir duraklama
        time.sleep(random.uniform(0.3, 1.2))

def gorev_simulasyonu():
    log_yaz("SYSTEM", "Sistem başlatıldı. Tüm sensörler (LIDAR, Kamera, IMU) aktif edildi.")
    time.sleep(1)
    
    log_yaz("MAIN", "Göreve başlandı. Hedef rotası yükleniyor...")
    log_yaz("MAIN", "Motor sürücülerine güç verildi.")
    time.sleep(1)
    
    # Genel hareket logları
    hareket_loglari = [
        "İleri yönlü hareket sürüyor (Gidiliyor...)",
        "Otonom sürüş devrede, gidiliyor...",
        "Tekerlek devir hızı stabil (Gidiliyor).",
        "Sensör verileri işleniyor, yol temiz.",
        "Anlık Hız:",
        "LIDAR taraması yapılıyor, engel yok, gidiliyor...",
        "Hedefe doğru ilerleme kaydediliyor (Gidiliyor...)"
    ]

    # Ortalama logları
    ortalama_loglari = [
        "Araç yönelimi hesaplanıyor...",
        "Kamera açısından merkez ofset değeri alınıyor...",
        "Direksiyon açısı güncelleniyor...",
        "PID kontrolcüsü devrede, ortalama yapılıyor...",
        "Motor torku asimetrik olarak dağıtılıyor (Ortalanıyor)..."
    ]

    # Yaklaşma logları
    yaklasma_loglari = [
        "Kukaya doğru gidiliyor...",
        "Hedef cisme yaklaşılıyor (Gidiliyor...).",
        "Görüntü işleme FPS: 30 - Kuka takibi aktif.",
        "Kalan Mesafe:",
        "Hız optimize ediliyor (Gidiliyor...)"
    ]

    # 4 adet kuka için döngü (Her döngü ortalama 75 saniye sürecek, toplam ~300 sn = 5 dk)
    for kuka_id in range(1, 5):
        # 1. Aşama: Bir sonraki kukayı arayarak ilerleme (Yaklaşık 20 saniye log akışı)
        log_yaz("VISION", f"{kuka_id}. Kuka için alan taraması başlatıldı.")
        surekli_log_at(20, hareket_loglari)
        
        # 2. Aşama: Kukayı görme
        x_coord = round(random.uniform(10, 60), 1)
        y_coord = round(random.uniform(2, 8), 1)
        log_yaz("VISION", ">>> DIKKAT: HEDEF TESPİT EDİLDİ <<<")
        log_yaz("VISION", f"KUKA-{kuka_id} GÖRÜLDÜ! (Koordinat: X:{x_coord}, Y:{y_coord})")
        time.sleep(1)
        
        # 3. Aşama: Ortalama (Yaklaşık 15 saniye log akışı)
        log_yaz("CONTROL", f"Kuka-{kuka_id} hedeflendi. Araç ortalanıyor...")
        surekli_log_at(15, ortalama_loglari, ["CONTROL", "VISION", "SYSTEM"])
        
        aci = round(random.uniform(-0.05, 0.05), 3)
        log_yaz("CONTROL", f"*** ORTALAMA BAŞARILI *** Hizalama Açısı: {aci} derece.")
        time.sleep(1)
        
        # 4. Aşama: Kukaya doğru ilerleme (Yaklaşık 30 saniye log akışı)
        log_yaz("MOTION", f"Kuka-{kuka_id}'ye doğru ilerleme başlatıldı.")
        surekli_log_at(30, yaklasma_loglari, ["MOTION", "NAVI", "VISION"])
        
        # 5. Aşama: Kukayı geçme (Yaklaşık 10 saniye rahatlama)
        log_yaz("NAVI", f"*** KUKA-{kuka_id} BAŞARIYLA GEÇİLDİ ***")
        
        if kuka_id < 4:
            log_yaz("MAIN", "Sonraki hedef için rotaya dönülüyor, gidiliyor...")
            surekli_log_at(10, hareket_loglari)

    # Bitiş
    log_yaz("MAIN", "Belirlenen 4 kukanın tamamı aşıldı. Yavaşlama moduna geçiliyor.")
    surekli_log_at(5, ["Hız kademeli olarak düşürülüyor...", "Fren sistemi devrede..."])
    
    log_yaz("MOTION", "Araç tamamen durdu.")
    log_yaz("SYSTEM", "GÖREV BAŞARIYLA TAMAMLANDI. Motorlar kapatılıyor.")

if __name__ == "__main__":
    print("Görev Başlatılıyor...")
    time.sleep(2)
    try:
        gorev_simulasyonu() 
    except KeyboardInterrupt:
        print("\n[UYARI] Simülasyon kullanıcı tarafından durduruldu.")