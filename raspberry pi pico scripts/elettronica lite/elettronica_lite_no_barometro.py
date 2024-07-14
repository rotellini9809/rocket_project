from time import sleep
import time as Time
from machine import Pin, I2C, UART, PWM



def read_bt():
    command = ""
    if bt.any() > 0:
        command = bt.read().decode('utf-8')
    return command

def get_time(start_time): 
    time = Time.ticks_ms()
    deltaT = time - start_time 
    return deltaT





buzzer = Pin(2,  Pin.OUT)
buzzer.value(0)
led= Pin(25, Pin.OUT)
led.value(1)

led.value(1)
bt = UART(1,9600)
paired = Pin(7, Pin.IN)



hot_wire = Pin(10,Pin.OUT,Pin.PULL_DOWN)
hot_wire.value(0)

print("initializzation")

message=""
connected_flag=False
while True:
    while True:
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
            sleep(0.5)
      



    start_time = Time.ticks_ms()
    open_chute = False
    start_flag = True
    flag2=True
    close_wire = False

    

    while True:
        
        time = get_time(start_time)   
        bt_command = read_bt()
        
        if bt_command == "restart\r\n":
            bt.write("restarting the program \n")
            buzzer.value(0)
            restart = True
            break
            
        if not open_chute and (time >= 3000 or bt_command == "open chute\r\n"):
            print("chute opened")
            bt.write("chute opened\n")
            open_chute = True
            open_chute_time = time
            hot_wire.value(1)
            
            
        if open_chute:
            if not close_wire and (time-open_chute_time >= 1000):
                hot_wire.value(0)
                close_wire = True
                print("chiuso")
            
            if flag2 and time > 12000:
                buzzer.value(1)
                flag2 = False
                
                
                
                            
    





 







