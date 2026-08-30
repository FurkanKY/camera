#!/usr/bin/env python3

# ZED wrapper'in compressed topic'ini alip throttle ederek yeniden yayinlar.
# Boylece PC'ye gelen veri miktari azalir, donma olmaz.
#
# ZED wrapper'in yayinladigi gercek compressed topic:
#   /zed2/zed_node/rgb/color/rect/image/compressed
#
# Bu node'un yayinladigi cikis topic'i:
#   /zed2/image_compressed   (output_fps Hz'de throttled)

import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class ZedCompressedThrottle(Node):
    def __init__(self):
        super().__init__('zed_compressed_throttle')

        # =========================================================
        # PARAMETERS
        # =========================================================
        # ZED wrapper'in compressed topic'ini buraya gir.
        # Gercek topic: /zed2/zed_node/rgb/image_rect_color/compressed
        self.declare_parameter(
            'input_topic',
            '/zed2/zed_node/rgb/color/rect/image/compressed'
        )

        # PC'ye yayinlanacak cikis topic'i.
        self.declare_parameter(
            'output_topic',
            '/zed2/image_compressed'
        )

        # Kac fps ile iletilsin? Ag bandi kisaysa 5-10 yeterli.
        # Dondurma olmamasi icin 10 baslangic degeri olarak uygundur.
        self.declare_parameter('output_fps', 10.0)

        self.input_topic = str(
            self.get_parameter('input_topic').value
        )
        self.output_topic = str(
            self.get_parameter('output_topic').value
        )
        self.output_fps = float(
            self.get_parameter('output_fps').value
        )

        if self.output_fps <= 0.0:
            self.get_logger().warn('output_fps gecersiz, 10.0 yapiliyor.')
            self.output_fps = 10.0

        self.min_period = 1.0 / self.output_fps
        self.last_time = 0.0

        # =========================================================
        # QOS - BEST_EFFORT, depth=1 (en son kareyi al, tampon biriktirme)
        # =========================================================
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.pub = self.create_publisher(
            CompressedImage,
            self.output_topic,
            qos
        )

        self.sub = self.create_subscription(
            CompressedImage,
            self.input_topic,
            self.callback,
            qos
        )

        self.get_logger().info('ZED compressed throttle baslatildi.')
        self.get_logger().info(f'Input : {self.input_topic}')
        self.get_logger().info(f'Output: {self.output_topic}')
        self.get_logger().info(f'FPS   : {self.output_fps}')

    def callback(self, msg: CompressedImage):
        now = time.monotonic()

        if now - self.last_time < self.min_period:
            return

        self.last_time = now
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ZedCompressedThrottle()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('ZED compressed throttle durduruldu.')

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
