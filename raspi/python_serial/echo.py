 
import robot_serial

ser = robot_serial.RobotSerial('/dev/ttyACM0',250000,2)

print(ser.name())

text = input()
while text != 'quit':
    ser.send(text)
    while ser.available() <= 0:
        pass
    
    while ser.available() > 0:    
        response = ser.recv()
        print(response.decode(),end="")

    text = input()
