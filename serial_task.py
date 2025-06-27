import serial
import threading
class Ser:
    def __init__(self,port_='/dev/ttyUSB0',baudrate_=9600,timeout_=1):
        self.ser=serial.Serial(port=port_,baudrate=baudrate_,timeout=timeout_)
        self.message=None
    def start(self):
        self.running=True
        self.wait_message_thread=threading.Thread(target=self._waiting_message)
        self.wait_message_thread.start()
    def stop(self):
        self.running=False
        if self.ser.is_open:
            self.ser.close()
    def _waiting_message(self):
        try:
            while self.running:
                if self.ser.in_waiting > 0:
                    self.message = self.ser.readline().decode('utf-8').strip()
        except Exception as e:
            if self.ser.is_open:
                self.ser.close()
            print("Error:Serial port closed.")
        finally:
            if self.ser.is_open:
                self.ser.close()
            print("Serial port closed.")
    def get_message(self):
        return self.message
    def send(self, msg):
        self.ser.write(msg.encode())
