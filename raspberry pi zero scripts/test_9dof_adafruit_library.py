import time
import board
import adafruit_lsm303dlh_mag
import adafruit_lsm303_accel
import l3gd20
import smbus

bus = smbus.SMBus(1)
time.sleep(1) # wait for i2c to stabilize
i2c = board.I2C()
acc = adafruit_lsm303_accel.LSM303_Accel(i2c)
mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
gyro = l3gd20.L3GD20(bus)
gyro.set_range(l3gd20.RANGE_250DPS)



def get_gyro():
    gx,gy,gz = gyro.read()
    return gx,gy,gz
def get_accel():  
    ax,ay,az = acc.acceleration # read and convert accel data  
    return ax,ay,az
def get_mag():
    mx,my,mz = mag.magnetic
    return mx,my,mz


while True:
    print(get_mag())
    get_accel()
    get_mag()
    try:
        print(get_mag())
        get_accel()
        get_mag()
    except:
        print("erroor")
        continue