# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import busio
import digitalio
from adafruit_bno08x import (
 BNO_REPORT_ACCELEROMETER,
 BNO_REPORT_GYROSCOPE,
 BNO_REPORT_MAGNETOMETER,
 BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.spi import BNO08X_SPI
spi = busio.SPI(clock = board.SCK, MOSI =board.MOSI, MISO = board.MISO)
bno = BNO08X_SPI(spi,cspin=digitalio.DigitalInOut(board.D6),intpin=digitalio.DigitalInOut(board.D5),resetpin=digitalio.DigitalInOut(board.D4))
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
while True:

    time.sleep(0.5)
    try:
        '''
        print("Acceleration:")
        accel_x, accel_y, accel_z = bno.acceleration # pylint:disable=no-member
        print("X: %0.6f Y: %0.6f Z: %0.6f m/s^2" % (accel_x, accel_y, accel_z))
        print("")
        print("Gyro:")
        gyro_x, gyro_y, gyro_z = bno.gyro # pylint:disable=no-member
        print("X: %0.6f Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
        print("")
        print("Magnetometer:")
        mag_x, mag_y, mag_z = bno.magnetic # pylint:disable=no-member
        print("X: %0.6f Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
        print("")
        print("Rotation Vector Quaternion:")
        quat_i, quat_j, quat_k, quat_real = bno.quaternion # pylint:disable=no-member
        print(
        "I: %0.6f J: %0.6f K: %0.6f Real: %0.6f" % (quat_i, quat_j, quat_k,
        quat_real)
        )
        print("")
        '''
        print(bno.magnetic )
        time.sleep(1)
    except:
        continue
        print("wanemo")
