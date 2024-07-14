from time import sleep
import time as Time
from machine import Pin, I2C, UART, PWM
import bme280



def read_bt():
    command = ""
    if bt.any() > 0:
        command = bt.read().decode('utf-8')
    return command

def get_time(start_time): 
    time = Time.ticks_ms()
    deltaT = time - start_time 
    return deltaT
    
def get_baro_altitude():
    pressure = float(bme.values[1][0:-3])
    baro_altitude = 44330.8 * (1 - (pressure / 1013) ** (1 / 5.255))
    return baro_altitude


led= Pin(25, Pin.OUT)
led.value(1)
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
paired = Pin(7, Pin.IN)



hot_wire = Pin(10,Pin.OUT,Pin.PULL_DOWN)
hot_wire.value(0)

print("initializzation")

message=""
connected_flag=False
while True:
    while True:
        # print('checking BT')
        if not connected_flag and paired.value():
            connected_flag = True
            bt.write("rocket connected\r\n")
            print("connected")
            buzzer.value(1)
            sleep(0.5)
            buzzer.value(0)
        
        
        
        if paired.value():
            bt_command = read_bt()
            if bt_command:
                print(bt_command)
                if bt_command == "start\r\n":
                    bt.write("started succesfully\r\n")
                    break
                if bt_command == "ping\r\n":
                    bt.write("ping received\r\n")
                    print("ping received")
                    buzzer.value(1)
                    sleep(0.5)
                    buzzer.value(0)
        else:
            connected_flag = False
        




    start_time = Time.ticks_ms()
    previous_time = 0
    baro_altitude = get_baro_altitude()
    alt_sec = [baro_altitude]
    previous_altitude = baro_altitude
    open_chute = False
    start_flag = True
    flag2=True
    close_wire = False

    file = open("samples_test_altitude.csv", "w", encoding="UTF8")
    file.write("time, altitude \n")

    while True:
        
        time = get_time(start_time)
        baro_altitude = get_baro_altitude()

        
        if time <= previous_time+1000:
            alt_sec.append(baro_altitude)
        else:
            alt_sec.append(baro_altitude)
            
            alt_sec = round(sum(alt_sec)/len(alt_sec),2)
            try:
                file.write(str(time) + ", " + str(alt_sec) + "\n")
            except:
                pass
            alt_sec=[]
            

            previous_time = time
            #previous_altitude = alt_sec
            

            
        bt_command = read_bt()
            
        if not open_chute and (time >= 3000 or bt_command == "open chute\r\n"):
            print("chute opened")
            open_chute = True
            open_chute_time = time
            file.write(str(time) + ", " + "12345" + "\n")   # to check later when chute opened
            hot_wire.value(1)
            
            
        if open_chute:
            if not close_wire and (time-open_chute_time >= 1000):
                hot_wire.value(0)
                close_wire = True
                print("chiuso")
            
            if flag2 and time > 12000:
                buzzer.value(1)
                file.close()
                flag2 = False
                

            
            if not flag2 and bt_command == "stop buzzer\r\n":
                        buzzer.value(0)
                        break
                    
                    
    bt_command = read_bt()      
    if bt_command == "restart\r\n":
        break
    else:
        sleep(0.5)
            





 


