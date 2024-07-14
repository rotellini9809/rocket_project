import time
import board
import busio
import digitalio
import pwmio
import json
from ulab import numpy as np




class Buzzer:
    def __init__(self,buzzer_pin):
        self.buzzer = pwmio.PWMOut(buzzer_pin, variable_frequency=True)

        with open('notes-frequencies.json', 'r') as f:
            # Initialize the dictionary to hold note names and their frequencies
            self.notesToFrequencies = json.load(f)
    
    
    def playNote(self,note,timeInSeconds):
        self.buzzer.frequency = self.notesToFrequencies[note]
        self.buzzer.duty_cycle = 32768
        time.sleep(timeInSeconds)
        self.buzzer.duty_cycle = 0
        time.sleep(0.1)  # Pause between notes
        
        
    def playNoteSequence(self,notes,durations=None):
        if not durations:
            durations=[0.2 ]*len(notes)
        
        for note,duration in zip(notes,durations):
            self.playNote(note,duration)
            
            
    def startupBuzz(self):
        startup = ["D4", "Gb4", "D4", "A4", "D4", "D5"]
        self.playNoteSequence(startup)
    
    def calibrationCompleteBuzz(self):
        calibration_complete = ["A3", "D4"]
        calibration_complete_duration = [0.3, 0.5]
        self.playNoteSequence(calibration_complete,calibration_complete_duration)
        
    def errorbuzz():
        error_sound_notes = ["Bb5", "A5", "Ab5", "G5", "Gb5"]
        error_sound_notes_duration = [0.01]*len(error_sound_notes)
        self.playNoteSequence(error_sound_notes)

'''
class Gps():
    def __init__(self,SCL,SDA):
        uart_gps = busio.UART(SCL, SDA, baudrate=9600, timeout=10)
        gps = adafruit_gps.GPS(uart_gps, debug=False)

        gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        gps.send_command(b'PMTK220,1000')
        
'''

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
 BNO_REPORT_ACCELEROMETER,
 BNO_REPORT_GYROSCOPE,
 BNO_REPORT_MAGNETOMETER,
 BNO_REPORT_ROTATION_VECTOR,
)

class IMU():
    def __init__(self,SCL,SDA,calibration = True):
        t0 = time.time()
        start_bool = False # if IMU start fails - stop calibration
        self.offsets = [0,0,0]
        
        
        while time.time()-t0<2:
            try:
                i2c = busio.I2C(SCL, SDA, frequency=400000)
                self.bno = BNO08X_I2C(i2c)
                self.bno.initialize()
                self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
                #self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
                #self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
                self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

                #buffer cleaning
                for i in range(50):
                    self.get_accel()
                    self.bno.quaternion
                
                start_bool = True
                break
            except Exception as e:
                print(e)
            
        if not start_bool:
            
            raise Exception("IMU not Started")
    
        if calibration:
            self.offsets = self.calibrate_accelerometer_mean()
            print("calibrating magnetometer")
            self.bno.begin_calibration()
            if not self.bno.calibration_status:
                print("rotate the electronics")
                time.sleep(1)
        
            
        
        
        
    def calibrate_accelerometer_mean(self):
        print("Calibrating accelerometer put the rocket nose up on a stable and flat surface")
        offsets = [0.0, 0.0, 0.0]  # Offsets for x, y, z axes
        
        z_gravity = 9.81
        NUM_SAMPLES = 500
        # Collect data for calibration
        for _ in range(NUM_SAMPLES):
            # Read raw acceleration data
            x,y,z = self.get_accel() 
            
            z -= z_gravity
            offsets[0] += x
            offsets[1] += y
            offsets[2] += z
               
        # Calculate average offsets
        for i in range(3):
            offsets[i] /= NUM_SAMPLES
     
        print(x,"	",y,"	",z)
        print("offsets: ",offsets)
        return offsets
    
    def apply_calibration(self,raw_data):
        result = [0,0,0]
        for i in range(3):
            result[i] = raw_data[i] - self.offsets[i]
        return result
    
    def get_accel(self):  
        acc = np.array(self.apply_calibration(self.bno.acceleration)) # read and convert accel data  
        return acc
    def get_quaternion(self):
        Q = np.array(self.bno.quaternion)
        return Q
    '''
    def get_gyro(self):
        gx,gy,gz = gyroscope.read()
        return gx,gy,gz
    def get_mag(self):
        mx,my,mz = magnetometer.magnetic
        return mx,my,mz
    '''
    
    
    
    def quaternion_to_rotation_matrix(self,q):
        """
        Convert a quaternion to a rotation matrix.
        
        Args:
            q (list or numpy.ndarray): Quaternion in the form [x, y, z, w].
        
        Returns:
            numpy.ndarray: 3x3 rotation matrix.
        """
        
        q = q / np.linalg.norm(q)  # Normalize quaternion
        
        x, y, z, w = q
        xx, yy, zz = x**2, y**2, z**2
        xy, xz, xw = x * y, x * z, x * w
        yz, yw, zw = y * z, y * w, z * w
        
        rotation_matrix = np.array([
            [1 - 2 * (yy + zz), 2 * (xy - zw), 2 * (xz + yw)],
            [2 * (xy + zw), 1 - 2 * (xx + zz), 2 * (yz - xw)],
            [2 * (xz - yw), 2 * (yz + xw), 1 - 2 * (xx + yy)]])
        
        return rotation_matrix

    def get_absolute_accelerations(self):
        Q = self.get_quaternion()
        acc = self.get_accel()
        
        rotation_matrix = self.quaternion_to_rotation_matrix(Q)
        #rotation_matrix = np.transpose(rotation_matrix)
        abs_acc = np.dot(rotation_matrix,acc)
        return abs_acc

    
    def get_absolute_accelerations_and_quaternion(self):
        Q = self.get_quaternion()
        acc = self.get_accel()
        
        rotation_matrix = self.quaternion_to_rotation_matrix(Q)
        #rotation_matrix = np.transpose(rotation_matrix)
        abs_acc = np.dot(rotation_matrix,acc)
        return abs_acc,Q
    
    

class HotWire:
    def __init__(self,hot_wire_pin):
        self.hot_wire = digitalio.DigitalInOut(hot_wire_pin)
        self.hot_wire.direction = digitalio.Direction.OUTPUT
        self.hot_wire.value = False
    def activation(self):
        self.hot_wire.value = True
        time.sleep(1.5)
        self.hot_wire.value = False
        

from adafruit_bme280 import basic as adafruit_bme280
        
    


if __name__ == '__main__':
    
    i2c = board.I2C(board.GP15, board.GP14)
    bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c)
    print("\nTemperature: %0.1f C" % bme280.temperature)
    print("Humidity: %0.1f %%" % bme280.relative_humidity)
    print("Pressure: %0.1f hPa" % bme280.pressure)
    print("Altitude = %0.2f meters" % bme280.altitude)
    
    #gps = Gps(board.GP4, board.GP5)
    buzzer = Buzzer(board.GP12)
    imu = IMU(board.GP11, board.GP10, False )
    imu.get_absolute_accelerations()
    print(buzzer.notesToFrequencies)
    buzzer.startupBuzz()

                

