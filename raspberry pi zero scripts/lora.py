import time
import serial
lora = serial.Serial('/dev/ttyAMA0', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
time.sleep(1)
print("start")

b = lora.inWaiting()
if b > 0:
    print(lora.read())

while True:
    
    
    #lora.write(bytes("bella bro fin", "utf-8"))
    #print("writed")
    #time.sleep(1)
    if lora.inWaiting() > 0:
            data1 = lora.read(lora.inWaiting())
            print(data1)


lora.close()
