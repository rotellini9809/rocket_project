from time import sleep
import time as Time
from machine import Pin, I2C, UART, PWM
from lcd_i2c import LCD
import bme280


i2c = I2C(0, sda=Pin(8), scl=Pin(9), freq=400000)


sleep(0.3)
#print(i2c.scan())



start_time=Time.time()
previous_time=0
lcd = LCD(39, cols=16, rows=2, i2c=i2c)
lcd.begin()
bme = bme280.BME280(i2c=i2c)
buzzer = PWM(Pin(20))


def asc_desc_Counter(delta,ascending,asc_desc_counter):
    if abs(delta) < 0.5:
        return 0,True
    
    if delta < 0:
        if ascending:
            ascending=False
            asc_desc_counter = 1
        else: 
             asc_desc_counter += 1
    if delta > 0:
        if ascending:
            asc_desc_counter += 1
        else:
            ascending=False
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



baro_altitude = get_baro_altitude()
alt_sec = [baro_altitude]
previous_altitude = baro_altitude
asc_desc_counter = 0
climbing = False
ascending = True
open_chute = False

file = open("samples_test_altitude.csv", "w", encoding="UTF8")
write_counter = 0
file.write("time, altitude \n")

while True:
    
    time = get_time(start_time)
    baro_altitude = get_baro_altitude()

    
    if time == previous_time:
        alt_sec.append(baro_altitude)
    else:
        alt_sec.append(baro_altitude)
        print(asc_desc_counter)
        alt_sec = round(sum(alt_sec)/len(alt_sec),2)
        file.write(str(time) + ", " + str(alt_sec) + "\n")
        lcd.print(" alt ")
        lcd.print(str(round(alt_sec,1)))
        lcd.print(" tm ")
        lcd.print(str(time))
        lcd.set_cursor(col=0, row=0)
        one_delta = alt_sec - previous_altitude
        previous_time = time
        previous_altitude = alt_sec
        alt_sec = []
        
        
        (asc_desc_counter, ascending) = asc_desc_Counter(one_delta, ascending, asc_desc_counter)

        
        #if (ascending==False and asc_desc_counter>=3):
        if open_chute == False and asc_desc_counter >= 1 and ascending:
            climbing = True
            
        if (climbing and (asc_desc_counter == 0 or ascending == False)):
            open_chute = True
            climbing = False
            
            buzzer.duty_u16(1000)
            buzzer.freq(1000)
            sleep(3)
            buzzer.duty_u16(0)
            
            lcd.set_cursor(col=0, row=1)
            lcd.print("open chute at " + str(time))
            lcd.set_cursor(col=0, row=0)
            
            
            
        if open_chute == True:
            write_counter+=1
            if write_counter >= 7:
                file.close()
                break




