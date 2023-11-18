from time import sleep
from machine import Pin, I2C, UART
import bme280
from micropyGPS import MicropyGPS
import ds1307


led = Pin(25, Pin.OUT)
led.value(1)
gps = UART(1, 9600)

i2c = I2C(0, sda=machine.Pin(8), scl=machine.Pin(9), freq=400000)
i2c1 = I2C(1,sda=machine.Pin(10), scl=machine.Pin(11))

bme = bme280.BME280(i2c=i2c)
rtc = ds1307.DS1307(i2c1)
my_gps = MicropyGPS()

gps_altitudes=[]
gps_time=[]
baro_altitudes = []
baro_time=[]
previus_altitude=0
# print(hex(i2c.scan()[0]))



sleep(1)

start_time=rtc.datetime()
c=0
with open("samples.txt" , "w", encoding="UTF8") as f:
    f.write("time|baro altitude|gps altitude\n")
    while (c<1000):
        print(c)
        c=c+1
        datetime = rtc.datetime()
        #print(datetime)
        time = (datetime[4]-start_time[4])*3600+(datetime[5]-start_time[5])*60+(datetime[6]-start_time[6])
        
        pressure = float(bme.values[1][0:-3])
        baro_altitude = 44330.8 * (1 - (pressure / 1013) ** (1 / 5.255))

        f.write(str(time) + ";")
        f.write(str(baro_altitude)+";")
    
    
        Gps = gps.read()
        #print(Gps)
        for x in str(Gps):
            my_gps.update(x)
        
        
        if my_gps.altitude and my_gps.altitude != previus_altitude:
            f.write(str(my_gps.altitude)+"\n")
            print("gps altitude:",my_gps.altitude)
            #print(Gps)
        else:
            f.write("/\n")
        previus_altitude=my_gps.altitude
        
    #print(my_gps.timestamp)
    #print(rtc.datetime())
    #print(my_gps.altitude)
    #print(my_gps.latitude_string())
    #print(my_gps.longitude_string())
        
        sleep(0.5)
led.value(0)
        
