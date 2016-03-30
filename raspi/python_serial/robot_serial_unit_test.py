#!/usr/bin/env python3

# unit under test
import robot_serial
# unit under test

import random
from time import sleep




def make_msg(op,x,y):
    return str(op) + ',' + str(x) + ',' + str(y) + '\n'

if __name__ == '__main__':

    limit = 150
    num_tests = 100

    #com = robot_serial.RobotSerial("/dev/ttyACM0",250000,2)
    com = robot_serial.RobotSerial('/dev/ttyACM0',115200,2)
    sleep(0.500)

    if not com:
        print("Failed to open device")
        exit()
    
    print(com.name())
    print('\n')


    tests_passed = 0
    op = ''
    for i in range(num_tests):

        print('test #' + str(i+1))

        x = random.randint(1,limit)
        y = random.randint(1,limit)

        if random.randrange(2) == 0:
            op = '*'
            z = x * y #compute correct result
        else:
            op = '+'
            z = x + y

        msg = make_msg(op,x,y)

        print("sent: " + msg)
        com.send(msg.encode())

        ret = com.recv()
        print('received: ' + ret.decode())
        result = ret.decode()
        if len(result) >= 1:
            n = int(result[0:len(result)-1])
        else:
            n = -1


        if n == z:
            print('passed')
            tests_passed += 1
        else:
            print('failed')

        print('')

    
    print("tests completed")    
    print("success: " + str(tests_passed) + '/' + str(num_tests))









    # c = input()
    # while c != 'q':
    #     com.send(c+'\n')
    #     ret = com.recv()
    #     print(ret.decode())
    #     c = input()

