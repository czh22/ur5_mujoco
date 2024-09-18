import serial
import struct
import time
import threading

class SerialCommunicator:
    def __init__(self, port, baudrate=9600, timeout=1, send_frequency=5):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.send_frequency = send_frequency  # 发送频率(Hz)
        self.value_to_send = 0.0
        self.running = True

        try:
            self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            print(f"已连接到 {self.port}, 波特率 {self.baudrate}。")
        except serial.SerialException as e:
            print(f"打开串口时出错: {e}")
            self.serial = None
        time.sleep(5)
        # 创建并启动发送线程
        self.thread = threading.Thread(target=self._send_thread)
        self.thread.start()

    def _send_thread(self):
        while self.running:
            if self.serial and self.serial.is_open:
                self.send_float(self.value_to_send)
            time.sleep(1 / self.send_frequency)

    def send_float(self, value):
        if self.serial and self.serial.is_open:
            try:
                data = struct.pack('f', value)
                self.serial.write(data)
            except serial.SerialTimeoutException as e:
                print(f"发送数据时出错: {e}")
        else:
            print("串口未打开。")

    def set_value(self, value):
        self.value_to_send = value

    def close(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        if self.serial and self.serial.is_open:
            self.serial.close()
            print(f"已关闭与 {self.port} 的连接。")

# 使用示例
if __name__ == "__main__":
    communicator = SerialCommunicator('/dev/ttyUSB0', 9600, send_frequency=5)
    
    try:
        for i in range(5):
            communicator.set_value(float(i * 10))
            time.sleep(1)
        communicator.set_value(0.0)
        time.sleep(1)
    finally:
        communicator.close()