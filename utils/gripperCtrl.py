import time 
# import serial
import pyfirmata2 

class ctrlClass:
    def __init__(self, serial_port: str = '/dev/ttyACM0'):
        PORT = pyfirmata2.Arduino.AUTODETECT
        print(PORT)
        self.arduino = pyfirmata2.Arduino(PORT)
        # self.arduino = pyfirmata2.Arduino(port= serial_port, baudrate=115200, timeout=3) 

        # define servo pins
        self.servo = self.arduino.get_pin('d:2:s')

    def write(self, x: int = 0): 
        self.servo.write(x) # bytes(x, 'utf-8') 
        time.sleep(0.5) 
        # data = self.arduino.readline().decode() 
        # return data 
    
    def exit_arduino(self):
        self.arduino.exit()

# example write and read 
if __name__ == "__main__":
    gripper = ctrlClass()
    while True: 
        num = input("Enter a number: ") # Taking input from user 
        value = gripper.write_read(num) 
        print(value) # printing the value 
