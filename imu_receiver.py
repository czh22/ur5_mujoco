
import serial  # 导入模块
import serial.tools.list_ports
import threading
import struct
import time
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE

# 宏定义参数
PI = 3.1415926
FRAME_HEAD = str('fc')
FRAME_END = str('fd')
TYPE_IMU = str('40')
TYPE_AHRS = str('41')
TYPE_INSGPS = str('42')
TYPE_GEODETIC_POS = str('5c')
TYPE_GROUND = str('f0')
TYPE_SYS_STATE = str('50')
TYPE_BODY_ACCELERATION = str('62')
TYPE_ACCELERATION = str('61')
IMU_LEN = str('38')  # //56
AHRS_LEN = str('30')  # //48
INSGPS_LEN = str('48')  # //72
GEODETIC_POS_LEN = str('20')  # //32
SYS_STATE_LEN = str('64')  # // 100
BODY_ACCELERATION_LEN = str('10') #// 16
ACCELERATION_LEN = str('0c')  # 12
PI = 3.141592653589793
DEG_TO_RAD = 0.017453292519943295







# 寻找输入的port串口
def find_serial(port):
    ports_list = list(serial.tools.list_ports.comports())
    for ports in ports_list:
        if ports.device == port:
            return True
    return False


def open_port(port):
    if find_serial(port):
        print("find this port : " + port)
    else:
        print("error:  unable to find this port : " + port)
        exit(1)






class imu_receiver():

    def __init__(self, port, bps, timeout):
        self.port = port
        self.bps = bps
        self.timeout = timeout
        self.AHRS_DATA = None
        open_port(self.port)
        try:
            self.serial = serial.Serial(port=self.port, baudrate=self.bps, bytesize=EIGHTBITS, parity=PARITY_NONE,
                                stopbits=STOPBITS_ONE,
                                timeout=self.timeout)
            print("baud rates = " + str(self.serial.baudrate))
        except:
            print("error:  unable to open port .")
            exit(1)
        
        # Create and start a thread to run receive_data
        self.receive_thread = threading.Thread(target=self.receive_data)
        self.receive_thread.daemon = True  # Optional: makes the thread a daemon thread
        self.receive_thread.start()


    def receive_data(self):
        
        # 循环读取数据
        while self.serial.isOpen():
            # rbdata = ser.readline()
            # # rbdata = ser.read_all()
            #
            # if len(rbdata) != 0:
            #     rxdata = rbdata.hex()
            #     print(rxdata)
            
            check_head = self.serial.read().hex()
            # 校验帧头
            if check_head != FRAME_HEAD:
                continue
            head_type = self.serial.read().hex()
            # 校验数据类型
            if (head_type != TYPE_IMU and head_type != TYPE_AHRS and head_type != TYPE_INSGPS and
                    head_type != TYPE_GEODETIC_POS and head_type != 0x50 and head_type != TYPE_GROUND and
                    head_type != TYPE_SYS_STATE and head_type!=TYPE_BODY_ACCELERATION and head_type!=TYPE_ACCELERATION):
                continue
            check_len = self.serial.read().hex()
            # 校验数据类型的长度
            if head_type == TYPE_IMU and check_len != IMU_LEN:
                continue
            elif head_type == TYPE_AHRS and check_len != AHRS_LEN:
                continue
            elif head_type == TYPE_INSGPS and check_len != INSGPS_LEN:
                continue
            elif head_type == TYPE_GEODETIC_POS and check_len != GEODETIC_POS_LEN:
                continue
            elif head_type == TYPE_SYS_STATE and check_len != SYS_STATE_LEN:
                continue
            elif head_type == TYPE_GROUND or head_type == 0x50:
                continue
            elif head_type == TYPE_BODY_ACCELERATION and check_len != BODY_ACCELERATION_LEN:
                print("check head type "+str(TYPE_BODY_ACCELERATION)+" failed;"+" check_LEN:"+str(check_len))
                continue
            elif head_type == TYPE_ACCELERATION and check_len != ACCELERATION_LEN:
                print("check head type "+str(TYPE_ACCELERATION)+" failed;"+" ckeck_LEN:"+str(check_len))
                continue
            check_sn = self.serial.read().hex()
            head_crc8 = self.serial.read().hex()
            crc16_H_s = self.serial.read().hex()
            crc16_L_s = self.serial.read().hex()
            # print("head_type: " + head_type)
            if head_type == TYPE_AHRS:
                data_s = self.serial.read(int(AHRS_LEN, 16))
                self.AHRS_DATA = struct.unpack('10f ii',data_s[0:48])
                # print(AHRS_DATA)
                # print("RollSpeed(rad/s): " + str(AHRS_DATA[0]))
                # print("PitchSpeed(rad/s) : " + str(AHRS_DATA[1]))
                # print("HeadingSpeed(rad) : " + str(AHRS_DATA[2]))
                # print("Roll(rad) : " + str(AHRS_DATA[3]))
                # print("Pitch(rad) : " + str(AHRS_DATA[4]))
                # print("Heading(rad) : " + str(AHRS_DATA[5]))
                # print("Q1 : " + str(AHRS_DATA[6]))
                # print("Q2 : " + str(AHRS_DATA[7]))
                # print("Q3 : " + str(AHRS_DATA[8]))
                # print("Q4 : " + str(AHRS_DATA[9]))
                # print("Timestamp(us) : " + str(AHRS_DATA[10]))
                # return AHRS_DATA
    def get_data(self):
        return self.AHRS_DATA
            


if __name__ == '__main__':
    imu_receiver = imu_receiver("/dev/ttyUSB0", 921600, 20)
    while(1):
        data = imu_receiver.get_data()
        if data is not None:
            print("RollSpeed(rad/s): " + str(data[0]))
            print("PitchSpeed(rad/s) : " + str(data[1]))
            print("HeadingSpeed(rad) : " + str(data[2]))
            print("Roll(rad) : " + str(data[3]))
            print("Pitch(rad) : " + str(data[4]))
            print("Heading(rad) : " + str(data[5]))
            print("Q1 : " + str(data[6]))
            print("Q2 : " + str(data[7]))
            print("Q3 : " + str(data[8]))
            print("Q4 : " + str(data[9]))
            print("Timestamp(us) : " + str(data[10]))      
          
                  