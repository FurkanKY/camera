#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from pynput import keyboard

class WebKlavyeKontrol(Node):
    def __init__(self):
        super().__init__('web_klavye_node')
        
        # AGX'e gidecek /web_command topic'i (Anten üzerinden AGX bu topic'i görecek)
        self.publisher_ = self.create_publisher(Int8, '/web_command', 10)
        
        # Başlangıçta "web" değeri 0 (Duruyor)
        self.web_degeri = 0 
        
        self.get_logger().info('--- WEB KLAVYE KONTROLÜ AKTİF ---')
        self.get_logger().info('SAĞ OK   -> Gönderilen: 1')
        self.get_logger().info('SOL OK   -> Gönderilen: -1')
        self.get_logger().info('BIRAKINCA -> Gönderilen: 0')

        # Klavyeyi dinleyen arka plan iş parçacığı (listener)
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

        # Saniyede 10 kere (0.1 saniye) güncel web_degeri'ni AGX'e fırlatır
        self.timer = self.create_timer(0.1, self.yayin_yap)

    def on_press(self, tus):
        """Tuşa basıldığı an tetiklenir"""
        if tus == keyboard.Key.right:
            self.web_degeri = 1
        elif tus == keyboard.Key.left:
            self.web_degeri = -1

    def on_release(self, tus):
        """Tuş bırakıldığı an tetiklenir"""
        if tus in [keyboard.Key.right, keyboard.Key.left]:
            self.web_degeri = 0
            
    def yayin_yap(self):
        """Zamanlayıcı ile sürekli topic'e basar"""
        msg = Int8()
        msg.data = self.web_degeri
        self.publisher_.publish(msg)
        
        # Sadece 0'dan farklı bir şeye basılıyken ekrana yazsın ki terminal kirlenmesin
        if self.web_degeri != 0:
            print(f"AGX'e gönderilen web komutu: {self.web_degeri}", end="\r")

def main(args=None):
    rclpy.init(args=args)
    node = WebKlavyeKontrol()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nÇıkış yapılıyor...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()