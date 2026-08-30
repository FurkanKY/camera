#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

from .bus_science import InnerCommunication 

class AgxWebAlici(Node):
    def __init__(self):
        super().__init__('agx_web_alici_node')
        
        # 1. CAN hattını başlat
        self.get_logger().info('CAN hattı (can2 - 1000000 baud) başlatılıyor...')
        self.motor_hub = InnerCommunication(channel='can2', rate=1000000)
        
        # 2. Yer İstasyonundan (senin bilgisayarından) gelecek topic'i dinle
        self.subscription = self.create_subscription(
            Int8,
            '/web_command',
            self.web_callback,
            10
        )
        self.get_logger().info('AGX Hazır: Anten üzerinden /web_command bekleniyor...')

    def web_callback(self, msg):
        val = msg.data
        
        # 3. Gelen veriyi (1, 0, -1) doğrudan CAN donanımına gönder. 
        # Python CAN kütüphanesi negatif sayı kabul etmediği için 8-bit işaretsiz (0xFF maskesi) ile gönderiyoruz.
        # Bu sayede STM32 -1'i (0xFF) doğru şekilde signed byte olarak okuyabilir.
        can_data = val & 0xFF
            
        # 4. bus.py içine eklediğimiz 0x04 ID'li fonksiyona gönder
        self.motor_hub.send_web(can_data)
        
        # Ekranda görmek için (tuşa basılı değilken terminali dondurmamak adına sadece 0 değilse yazdırıyoruz)
        if val != 0:
            self.get_logger().info(f'PC\'den Gelen: {val} ---> CAN Bus\'a Basılan (0x04): {val} (Raw: {can_data})')

def main(args=None):
    rclpy.init(args=args)
    node = AgxWebAlici()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nSistem kapatılıyor...")
    finally:
        # Kod durdurulduğunda aracın dönmeye devam etmemesi için son kez 0 (Dur) komutu gönder
        try:
            node.motor_hub.send_web(0)
        except:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
