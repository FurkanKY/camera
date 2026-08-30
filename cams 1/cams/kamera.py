import sys
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data 
from cv_bridge import CvBridge
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel, QHBoxLayout, QWidget
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer, Qt

class PCSubscriber(Node):
    def __init__(self):
        super().__init__('pc_gui_node')
        self.bridge = CvBridge()
        self.img1 = None
        self.img2 = None
        self.img3 = None
        
        # QoS profili ile gecikmesiz abone ol
        self.create_subscription(Image, '/zed1/low_res', self.cb1, qos_profile_sensor_data)
        self.create_subscription(Image, '/zed2/low_res', self.cb2, qos_profile_sensor_data)
        self.create_subscription(Image, '/webcam/low_res', self.cb3, qos_profile_sensor_data)

    def cb1(self, msg):
        self.img1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def cb2(self, msg):
        self.img2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def cb3(self, msg):
        self.img3 = self.bridge.imgmsg_to_cv2(msg, "bgr8")

class RoverGUI(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("Düşük Çözünürlüklü Kamera Arayüzü")
        self.resize(1500, 350) 

        widget = QWidget()
        layout = QHBoxLayout(widget)
        self.setCentralWidget(widget)

        self.label1 = QLabel("ZED 1 (Bağlanıyor...)")
        self.label2 = QLabel("ZED 2i (Bağlanıyor...)")
        self.label3 = QLabel("Webcam (Bağlanıyor...)")
        
        for label in [self.label1, self.label2, self.label3]:
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("border: 1px solid gray; background-color: black; color: white;")
            layout.addWidget(label)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_gui)
        self.timer.start(30)

    def update_gui(self):
        rclpy.spin_once(self.ros_node, timeout_sec=0)

        if self.ros_node.img1 is not None:
            self.display_image(self.ros_node.img1, self.label1)
        if self.ros_node.img2 is not None:
            self.display_image(self.ros_node.img2, self.label2)
        if self.ros_node.img3 is not None:
            self.display_image(self.ros_node.img3, self.label3)

    def display_image(self, cv_img, label):
        rgb_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb_img.shape
        qt_img = QImage(rgb_img.data, w, h, ch * w, QImage.Format_RGB888)
        # Veri boyutu düşük kalsa da ekranda rahat görmek için pencere boyutuna yayılır
        label.setPixmap(QPixmap.fromImage(qt_img).scaled(label.width(), label.height(), Qt.KeepAspectRatio))

def main():
    rclpy.init()
    ros_node = PCSubscriber()
    
    app = QApplication(sys.argv)
    gui = RoverGUI(ros_node)
    gui.show()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()