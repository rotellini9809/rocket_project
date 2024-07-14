from time import sleep
import time as Time
from machine import Pin, I2C, UART, PWM
import bme280



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



i2c = I2C(0, sda=Pin(8,  pull=Pin.PULL_UP), scl=Pin(9,  pull=Pin.PULL_UP), freq=10000 )  
sleep(1)
print(i2c.scan())
buzzer = Pin(2,  Pin.OUT)
buzzer.value(0)
led= Pin(25, Pin.OUT)
led.value(1)
bme = bme280.BME280(i2c=i2c)
led.value(1)
bt = UART(0,9600)
bt.write("AT")
paired = Pin(7, Pin.IN)



hot_wire = Pin(10,Pin.OUT,Pin.PULL_DOWN)
hot_wire.value(0)

print("initializzation")

message=""
previus_paired=0

while True:
    # print('checking BT')
    if paired.value() and previus_paired==0:
        sleep(1)
        bt.write("rocket connected\r\n")
        print("connected")
        buzzer.value(1)
        sleep(0.5)
    
    
    
    if previus_paired:
        bt_command = read_bt()
        if bt_command:
            print(bt_command)
        if bt_command == "start\r\n":
            break
    
    previus_paired = paired.value()




start_time = Time.time()
previous_time = 0
baro_altitude = get_baro_altitude()
alt_sec = [baro_altitude]
previous_altitude = baro_altitude
asc_desc_counter = 0
climbing = False
ascending = True
open_chute = False
start_flag = True

file = open("samples_test_altitude.csv", "w", encoding="UTF8")
still_counter = 0
file.write("time, altitude \n")

while True:
    
    time = get_time(start_time)
    baro_altitude = get_baro_altitude()

    
    if time == previous_time:
        alt_sec.append(baro_altitude)
    else:
        alt_sec.append(baro_altitude)
        
        alt_sec = round(sum(alt_sec)/len(alt_sec),2)
        file.write(str(time) + ", " + str(alt_sec) + "\n")
        
        
        one_delta = alt_sec - previous_altitude
        previous_time = time
        previous_altitude = alt_sec
        
        # we need to wait some seconds to let the barometer stabilize
        buzzer.value(0)
        if time>=3:
            if start_flag:
                start_flag = False
                bt.write("started succesfully\r\n")
                buzzer.value(1)
            (asc_desc_counter, ascending) = asc_desc_Counter(one_delta, ascending, asc_desc_counter)
            stringa = str(asc_desc_counter) + " " + str(alt_sec) +"\r\n"
            print(asc_desc_counter," ", alt_sec) 
            bt.write(stringa)
        alt_sec = []

        
        #if (ascending==False and asc_desc_counter>=3):
        if open_chute == False and asc_desc_counter >= 2 and ascending:    #secondi in salita
            climbing = True
            
        if (climbing and (asc_desc_counter == 0 or ascending == False)):
            open_chute = True
            hot_wire.value(1)
            open_chute_time = Time.time()- start_time
            close_wire=False
            climbing = False
            print("chute opened")
            bt.write("chute opened\r\n")
            
            
            
            
        #understand if rocket landed and make sound signalation    
        if open_chute == True:
            if not(close_wire) and (time-open_chute_time >= 1):
                hot_wire.value(0)
                close_wire = True
            
                
                
            if asc_desc_counter == 0:
                still_counter+=1
            else:
                still_counter=0
            
            if still_counter >= 5:
                file.close()
                buzzer.value(1)
                while True:
                    command = read_bt()
                    if command == "stop buzzer\r\n":
                        buzzer.value(0)
                        break
                break





 


