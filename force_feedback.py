import serial
import struct

class SerialCommunicator:
    def __init__(self, port, baudrate, timeout=1):
        """
        Initialize the serial communication with the given port, baudrate, and timeout.
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        try:
            self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=self.timeout)
            print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self.serial = None

    def send_float(self, value):
        """
        Send a float value to the Arduino.
        """
        if self.serial and self.serial.is_open:
            try:
                # Pack the float into 4 bytes using struct
                data = struct.pack('f', value)
                self.serial.write(data)
                print(f"Sent: {value}")
            except serial.SerialTimeoutException as e:
                print(f"Error sending data: {e}")
        else:
            print("Serial port is not open.")

    def close(self):
        """
        Close the serial connection.
        """
        if self.serial and self.serial.is_open:
            self.serial.close()
            print(f"Closed connection to {self.port}.")

# Example usage:
if __name__ == "__main__":
    communicator = SerialCommunicator('/dev/ttyUSB0', 9600)
    communicator.send_float(3.14159)
    communicator.close()