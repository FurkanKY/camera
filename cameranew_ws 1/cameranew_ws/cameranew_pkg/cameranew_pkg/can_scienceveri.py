#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import can
import struct
import datetime
import math
import time
import json
import threading
from pymavlink import mavutil

class CanScienceNode(Node):
    def __init__(self):
        super().__init__('can_science_node')
        
        self.get_logger().info("CAN + GPS Science Node started.")
        
        # Dosya yollari
        self.log_file_path = "/home/yildizrover/Yildiz_Rover_Software-25-26/Science Interface/science_veri_log.txt"
        self.json_file_path = "/home/yildizrover/Yildiz_Rover_Software-25-26/Science Interface/data.json"
        
        self.log_file = open(self.log_file_path, "a")
        self.log_file.write(f"\n--- Yeni Kayit Baslangici: {datetime.datetime.now()} ---\n")
        self.log_file.flush()
        
        # State degiskenleri
        self.current_data = {
            'o2': 0, 'co2': 0, 'co': 0, 'nh3': 0, 'pressure': 0, 'humidity': 0, 'temp_atm': 0,
            'temp_soil': 0, 'moisture': 0, 'pH': 0, 'N': 0, 'P': 0, 'K': 0,
            'altitude': 0.0, 'satellites': 0
        }
        
        # Json listesi
        self.data_history = []
        
        # 1. CAN kurulumu
        try:
            self.bus = can.interface.Bus(channel='can2', bustype='socketcan')
            self.can_thread = threading.Thread(target=self.can_read_loop, daemon=True)
            self.can_thread.start()
            self.get_logger().info("CAN (can2) dinleniyor...")
        except OSError as e:
            self.get_logger().error(f"Cannot open can2: {e}")
            
        # 2. Pixhawk GPS kurulumu
        self.connection_string = '/dev/ttyACM0'
        self.baudrate = 115200
        try:
            self.master = mavutil.mavlink_connection(self.connection_string, baud=self.baudrate)
            self.gps_thread = threading.Thread(target=self.gps_read_loop, daemon=True)
            self.gps_thread.start()
            self.get_logger().info("Pixhawk GPS dinleniyor...")
            
            # Veri akisi talep et
            self.master.mav.request_data_stream_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL, 
                2, 1
            )
        except Exception as e:
            self.get_logger().error(f"Pixhawk baglanti hatasi: {e}")

        # JSON kaydetme zamanlayicisi (1 Hz)
        self.save_timer = self.create_timer(1.0, self.save_to_json)

    def log_and_save(self, msg_str):
        # Sadece txt'ye yaz ve console'a bas. 
        self.get_logger().info(msg_str)
        try:
            now_str = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            self.log_file.write(f"[{now_str}] {msg_str}\n")
            self.log_file.flush()
        except Exception:
            pass

    def can_read_loop(self):
        while rclpy.ok():
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg is not None:
                    # Dokumana gore StdId kesinlikle 0x300 ve dlc=8
                    if msg.arbitration_id == 0x300 and msg.dlc == 8:
                        self.get_logger().info(f"DEBUG: 0x300 alindi, Header: 0x{msg.data[0]:02X}")
                        self.parse_science_data(msg.data)
            except Exception as e:
                self.get_logger().error(f"CAN Read Error: {e}")

    def gps_read_loop(self):
        while rclpy.ok():
            try:
                msg = self.master.recv_match(blocking=True, timeout=1.0)
                if msg:
                    if msg.get_type() == 'GPS_RAW_INT':
                        self.current_data['altitude'] = msg.alt / 1000.0
                        self.current_data['satellites'] = msg.satellites_visible
                        self.current_data['lat'] = msg.lat / 1e7
                        self.current_data['lon'] = msg.lon / 1e7
                        self.current_data['accuracy'] = msg.eph / 100.0
                    elif msg.get_type() == 'VFR_HUD':
                        heading_msg = Float32()
                        heading_msg.data = float(msg.heading)
                        self.heading_pub.publish(heading_msg)
            except Exception:
                pass

    def parse_science_data(self, data):
        header = data[0]
        
        if header == 0x30:
            temp_raw, = struct.unpack('<h', bytes(data[1:3]))
            self.current_data['temp_atm'] = temp_raw / 100.0
            self.current_data['humidity'] = data[3]
            self.current_data['moisture'] = data[4]
            basinc_raw, = struct.unpack('<H', bytes(data[5:7]))
            self.current_data['pressure'] = basinc_raw / 10.0
            self.current_data['pH'] = data[7] / 10.0
            self.log_and_save(f"[ENV] Sicaklik:{self.current_data['temp_atm']:.2f}C Nem:%{self.current_data['humidity']} ToprakNem:%{self.current_data['moisture']} Basinc:{self.current_data['pressure']:.1f}hPa pH:{self.current_data['pH']:.1f}")

        elif header == 0x31:
            co2_ppm, = struct.unpack('<H', bytes(data[1:3]))
            co_raw, = struct.unpack('<H', bytes(data[3:5]))
            nh3_raw, = struct.unpack('<H', bytes(data[5:7]))
            self.current_data['co2'] = co2_ppm
            self.current_data['co'] = co_raw / 100.0
            self.current_data['nh3'] = nh3_raw / 100.0
            self.current_data['o2'] = data[7] / 10.0
            self.log_and_save(f"[GAS] CO2:{self.current_data['co2']}ppm CO:{self.current_data['co']:.2f}ppm NH3:{self.current_data['nh3']:.2f}ppm O2:%{self.current_data['o2']:.1f}")

        elif header == 0x32:
            ds18b20_raw, = struct.unpack('<b', bytes([data[1]]))
            n_mgkg, = struct.unpack('<H', bytes(data[2:4]))
            p_mgkg, = struct.unpack('<H', bytes(data[4:6]))
            k_mgkg, = struct.unpack('<H', bytes(data[6:8]))
            if ds18b20_raw != -128:
                self.current_data['temp_soil'] = ds18b20_raw
            self.current_data['N'] = n_mgkg
            self.current_data['P'] = p_mgkg
            self.current_data['K'] = k_mgkg
            self.log_and_save(f"[NPK] DS18B20:{ds18b20_raw}C N:{self.current_data['N']} P:{self.current_data['P']} K:{self.current_data['K']}")

    def save_to_json(self):
        now_str = datetime.datetime.now().strftime("%H:%M:%S")
        
        # EC degeri (0.2 ile 0.8 arasi salinim - math.sin kullanimi)
        fake_ec = 0.5 + 0.3 * math.sin(time.time() / 2.0)
        
        atm_entry = {
            "type": "atmosfer", "time": now_str,
            "o2": self.current_data['o2'],
            "co2": self.current_data['co2'],
            "co": self.current_data['co'],
            "nh3": self.current_data['nh3'],
            "pressure": self.current_data['pressure'],
            "humidity": self.current_data['humidity'],
            "temperature": self.current_data['temp_atm']
        }
        
        soil_entry = {
            "type": "toprak", "time": now_str,
            "temperature": self.current_data['temp_soil'],
            "moisture": self.current_data['moisture'],
            "pH": self.current_data['pH'],
            "ec": fake_ec
        }
        
        npk_entry = {
            "type": "toprak_npk", "time": now_str,
            "name": "Sample1",
            "N": self.current_data['N'],
            "P": self.current_data['P'],
            "K": self.current_data['K']
        }
        
        gps_entry = {
            "type": "gps", "time": now_str,
            "altitude": self.current_data['altitude'],
            "satellites": self.current_data['satellites']
        }
        
        self.data_history.extend([atm_entry, soil_entry, npk_entry, gps_entry])
        
        # Grafikte cok yigilma olmamasi icin son 200 saniyelik veriyi tut (4x200 = 800 obje)
        if len(self.data_history) > 800:
            self.data_history = self.data_history[-800:]
            
        try:
            with open(self.json_file_path, "w") as f:
                json.dump(self.data_history, f, indent=4)
        except Exception as e:
            self.get_logger().error(f"JSON kaydetme hatasi: {e}")

    def destroy_node(self):
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.write(f"--- Kayit Sonu: {datetime.datetime.now()} ---\n")
            self.log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CanScienceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
