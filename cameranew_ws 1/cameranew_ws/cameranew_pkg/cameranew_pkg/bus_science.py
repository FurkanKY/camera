import time
import can

class InnerCommunication:
    def __init__(self, channel='can2', rate=1000000):
        self.bus = can.interface.Bus(bustype='socketcan', 
                                     channel=channel, 
                                     bitrate=rate)

    # 0x04 arbitration_id ile web datasını gönderen fonksiyon
    def send_web(self, web):
        # Gömülü sistemler data[0]'ı header olarak bekler, o yüzden 0x04 ilk byte'ta.
        payload = [0x04, web, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        
        msg = can.Message(arbitration_id=0x04, 
                          data=payload, 
                          is_extended_id=False)
        try:
            self.bus.send(msg)
           # time.sleep(0.05)
        except can.CanError:
            print("Message NOT sent")
