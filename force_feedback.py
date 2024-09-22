import serial
import struct
import time
import threading
import matplotlib.pyplot as plt

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

    def receive_float(self):
        if self.serial and self.serial.is_open:
            try:
                while self.serial.in_waiting > 4:
                    self.serial.read(1)  # 确保有足够的字节可读
                data = self.serial.read(4)
                value = struct.unpack('f', data)[0]
                   
                return value
               
            except serial.SerialException as e:
                print(f"接收数据时出错: {e}")
                return None
        else:
            print("串口未打开。")
            return None

    def close(self):
        self.running = False
        if self.thread.is_alive():
            self.thread.join()
        if self.serial and self.serial.is_open:
            self.serial.close()
            print(f"已关闭与 {self.port} 的连接。")

# 修改主程序部分
if __name__ == "__main__":
    communicator = SerialCommunicator('/dev/ttyUSB0', 9600, send_frequency=5)
    
    set_values = []
    received_values = []
    
    try:
        for i in range(100):
            value = float(i * 1)
            communicator.set_value(value)
            set_values.append(value)
            
            time.sleep(0.2)
            received = communicator.receive_float()
            if received is not None:
                received_values.append(received)
                print(f"设置值: {value}, 接收到的值: {received}")
            else:
                received_values.append(None)
        
        communicator.set_value(0.0)
        time.sleep(1)
    finally:
        communicator.close()
    
    # 绘制图表
    plt.figure(figsize=(100, 6))
    plt.plot(set_values, received_values)
   
    plt.xlabel('设置值')
    plt.ylabel('接收值')
    plt.title('设置值与接收值的关系')
   
    plt.grid(True)
    plt.show()