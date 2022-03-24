import serial, time
ser = serial.Serial('COM4', 115200, timeout=.1)
time.sleep(1) #give the connection a second to settle
#input_value = input('Enter pixel position: ')
if True:
    #ser.write(input_value.encode())
    #time.sleep(2)
    ser.write(b'0')
    time.sleep(2)
    ser.write(b'1')
    # ser.close()
    # exit()