import time
import smbus
import lsm303
import l3gd20


bus = smbus.SMBus(1)
time.sleep(1)
acc_mag = lsm303.LSM303(bus)
gyro = l3gd20.L3GD20(bus)
gyro.set_range(l3gd20.RANGE_250DPS)

while True:
    accel_data = acc_mag.read_accel()
    mag_data = acc_mag.read_mag()
    gyro_data = gyro.read()
    print(mag_data )
    #print(gyro_data)
    #print("acc_data: ",accel_data, "mag data: ", mag_data)
     