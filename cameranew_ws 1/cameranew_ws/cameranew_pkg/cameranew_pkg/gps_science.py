import sys
import rclpy
from rclpy.node import Node
from pymavlink import mavutil

class GPSScienceNode(Node):
    def __init__(self):
        super().__init__('gps_science_node')
        
        # Pixhawk bağlantı ayarları
        self.connection_string = '/dev/ttyACM0' 
        self.baudrate = 115200

        self.get_logger().info(f"{self.connection_string} üzerinden Pixhawk bekleniyor...")
        
        try:
            self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baudrate)
            
            self.get_logger().info("Heartbeat sinyali bekleniyor...")
            self.master.wait_heartbeat(timeout=5.0)
            self.get_logger().info("Pixhawk ile bağlantı başarıyla kuruldu!")
            
            # Sadece POSITION değil, TÜM (ALL) veri akışlarını talep ediyoruz
            self.get_logger().info("Pixhawk'tan TÜM veri akışı talep ediliyor...")
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 
                2, # Saniyede 2 kere gönder
                1  # Başlat
            )
            
        except Exception as e:
            self.get_logger().error(f"Pixhawk bağlantı hatası: {e}")
            sys.exit(1)

        # Timer çalışmaya devam ediyor
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.mesaj_geliyor_mu = False

    def timer_callback(self):
        # TAMPON BOŞALTMA (Buffer Draining) YÖNTEMİ
        # Tamponda bekleyen mesaj kalmayana kadar (veya None dönene kadar) döngüde oku
        while True:
            msg = self.master.recv_match(blocking=False)
            
            if not msg:
                break # Okunacak yeni mesaj kalmadı, döngüden çık
                
            if msg.get_type() == 'BAD_DATA':
                continue
                
            self.mesaj_geliyor_mu = True

            # Gelen mesajlardan sadece GPS_RAW_INT olanı yakala
            if msg.get_type() == 'GPS_RAW_INT':
                latitude = msg.lat / 1e7
                longitude = msg.lon / 1e7
                elevation = msg.alt / 1000.0
                accuracy = msg.eph / 100.0
                fix_type = msg.fix_type
                
                if fix_type > 2:
                    self.get_logger().info(f"Konum: {latitude:.7f}, {longitude:.7f} | Rakım: {elevation:.2f}m | Hassasiyet: ±{accuracy:.2f}m")
                else:
                    self.get_logger().info(f"GPS uyduları aranıyor... (Fix Yok - Tür: {fix_type})")

def main(args=None):
    rclpy.init(args=args)
    node = GPSScienceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Kullanıcı tarafından durduruldu.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()