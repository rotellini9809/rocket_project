




        
    #bt.write("bella zi \r\n")
import time
from time import sleep
import time as Time
from machine import Pin, I2C, UART, PWM



def read_bt():
    command = ""
    if bt.any() > 0:
        command = bt.read().decode('utf-8')
    return command

 # this counter count how many consecutive seconds the rosket is ascending or descending, if asc_desc_counter = 0 rocket is still
def asc_desc_Counter(delta,ascending,asc_desc_counter): 
    if abs(delta) < 0.8:     #threshold asc desc counter
        return 0,True
    
    if delta < 0:
        if ascending:
            ascending = False
            asc_desc_counter = 1
        else: 
             asc_desc_counter += 1
    if delta > 0:
        if ascending:
            asc_desc_counter += 1
        else:
            ascending = True
            asc_desc_counter = 1
    
    return asc_desc_counter,ascending

def get_time(start_time): 
    time = Time.time()
    deltaT = time - start_time 
    return deltaT
    
def get_baro_altitude():
    pressure = float(bme.values[1][0:-3])
    baro_altitude = 44330.8 * (1 - (pressure / 1013) ** (1 / 5.255))
    return baro_altitude



buzzer = Pin(2,  Pin.OUT)
buzzer.value(0)
led= Pin(25, Pin.OUT)
led.value(1)

led.value(1)
# razzo uart 0 | razzo 2 uart 1
bt = UART(1,9600)
bt.write("AT")
paired = Pin(7, Pin.IN)



hot_wire = Pin(10,Pin.OUT,Pin.PULL_DOWN)
#hot_wire.value(0)
#print("open")
#sleep(3)
hot_wire.value(0)

print("initializzation")

message=""
previus_paired=0

while True:
    # print('checking BT')
    if paired.value() and previus_paired==0:
        bt.write("rocket connected\r\n")
        print("connected")
        buzzer.value(1)
        time.sleep(1)
        buzzer.value(0)
        
    
    
    
    if previus_paired:
        bt_command = read_bt()
        if bt_command:
            print(bt_command)
        if bt_command == "start\r\n":
            print(time.time())
            hot_wire.value(1)
            #bt.write("chute opened\r\n")
            sleep(1)
            hot_wire.value(0)
            print(time.time())
            
    
    previus_paired = paired.value()





 


