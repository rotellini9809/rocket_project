import time
import board
import busio
import numpy as np
import csv
import serial
import sys
import RPi.GPIO as GPIO
#sys.path.append('/home/rotellini/.local/lib/python3.9/site-packages')

from micropyGPS import MicropyGPS
from adafruit_bno08x import (
 BNO_REPORT_ACCELEROMETER,
 BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
from filterpy.kalman import KalmanFilter
#print(sys.path)
#----------------------------------------------
#part to send quaternions to my laptop to see the rotation of the rocket
import socket
socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)



gps_parser = MicropyGPS()
gps = serial.Serial('/dev/ttyACM0',9600)
gps_covariance_matrix = np.diag([6.25,6.25,6.25,0.01,0.01,0.01])

lora = serial.Serial('/dev/ttyAMA0', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)

buzzer = 25
hot_wire = 17
GPIO.setmode(GPIO.BCM)
GPIO.setup(buzzer,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(hot_wire,GPIO.OUT,initial=GPIO.LOW)




def update_gps():
    if gps.in_waiting > 0:
        sentence = gps.read().decode('utf-8')
        for x in sentence:
            gps_parser.update(x)
        return True
    else:
        return False

#this function uses the haversine formula to calculate the distance between two points on the surphace of a shpere in this case hearth
def cordinates_distance(coord1, coord2):
    # Radius of the Earth in meters
    R = 6371000
    
    lat1, lon1 = coord1
    lat2, lon2 = coord2
    
    # Convert latitude and longitude from degrees to radians
    lat1, lon1, lat2, lon2 = map(np.radians, [lat1, lon1, lat2, lon2])
    
    # Haversine formula
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))
    distance = R * c
    
    # Calculate x and y components
    x_distance = distance * np.cos(lat1) * np.sin(dlon)
    y_distance = distance * (np.sin(lat1) * np.cos(lat2) - np.cos(lat1) * np.sin(lat2) * np.cos(dlon))
    
    return x_distance, y_distance

#this function converts the cordinates given as a degree minutes and direction indicator into decimal degrees
def convert_cordinates(cordinates):
    return cordinates[0]+(cordinates[1]/60)
    
def quaternion_to_rotation_matrix(q):
    """
    Convert a quaternion to a rotation matrix.
    
    Args:
        q (list or numpy.ndarray): Quaternion in the form [x, y, z, w].
    
    Returns:
        numpy.ndarray: 3x3 rotation matrix.
    """
    x, y, z, w = q
    norm = np.linalg.norm(q)
    s = 2.0 / norm if norm != 0 else 0.0
    print(s)
    # Calculate the elements of the rotation matrix
    R = np.array([
        [1 - s * (y**2 + z**2), s * (x*y - w*z), s * (x*z + w*y)],
        [s * (x*y + w*z), 1 - s * (x**2 + z**2), s * (y*z - w*x)],
        [s * (x*z - w*y), s * (y*z + w*x), 1 - s * (x**2 + y**2)]
    ])
    
    return R


def get_absolute_accelerations(Q, acc):
    rotation_matrix = quaternion_to_rotation_matrix(Q)
    inverse_rotation = np.transpose(rotation_matrix)
    abs_acc = np.dot(inverse_rotation,acc)
    return abs_acc


t0 = time.time()
start_bool = False # if IMU start fails - stop calibration
time.sleep(0.2)


while time.time()-t0<2:
    try:
        i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
        bno = BNO08X_I2C(i2c)
        bno.initialize()
        bno.enable_feature(BNO_REPORT_ACCELEROMETER)
        #bno.enable_feature(BNO_REPORT_GYROSCOPE)
        #bno.enable_feature(BNO_REPORT_MAGNETOMETER)
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

        start_bool = True
         
        break
    except:
        continue




if not start_bool:  
    print("IMU not Started")
    sys.exit()
    
    
    
def get_gyro():
    gx,gy,gz = gyroscope.read()
    return gx,gy,gz
def get_accel():  
    ax,ay,az = bno.acceleration # read and convert accel data  
    return ax,ay,az
def get_mag():
    mx,my,mz = magnetometer.magnetic
    return mx,my,mz

GPIO.output(buzzer,1)
time.sleep(0.5)
GPIO.output(buzzer,0)

active_calibration = False 
if active_calibration:
    print("start calibration")
    bno.begin_calibration()
    if not bno.calibration_status:
        print("rotate the electronics")
        time.sleep(1)


Q = np.array([[0.,0.,0.,1.]])

t_prev = time.time()
c=0

wait_for_gps=False
if wait_for_gps:
    print("waiting for gps first packet to initialize")
    while gps_parser.altitude == 0.0 or gps_parser.latitude == [0, 0.0, 'N'] or gps_parser.longitude == [0, 0.0, 'W']:
        update_gps()
    





need_start_from_console = False
if need_start_from_console:
    print("waiting for start command")
    while True:
        update_gps()
        if lora.in_waiting:
            message_received = lora.read(lora.in_waiting).decode()
            print(message_received)
            if "sr" in message_received:
                print("ollare")
                lora.write(b"sr_ok")
                break
        time.sleep(0.1)

#kalman filter intializzation
starting_position = (convert_cordinates(gps_parser.latitude),convert_cordinates(gps_parser.longitude))
previus_gps_reading = (gps_parser.altitude,gps_parser.latitude,gps_parser.longitude,gps_parser.speed)

kalman = KalmanFilter(dim_x = 6,dim_z = 6, dim_u = 3)
kalman.x = np.array([0.,0.,gps_parser.altitude,0.,0.,0.]).T  #[px,py,pz,vx,vy,vz]
kalman.R = gps_covariance_matrix  #we apply the covariance matrix of the gps to the kalman filter
kalman.Q = np.eye(6)
kalman.H = np.eye(6)
    
#buffer cleaning
for i in range(50):
    get_accel()
    bno.quaternion

messages=""
start_time = time.time()
lora_time=start_time+0.2


print("started succesfully")
while True:
    try:
        acc = np.array(get_accel())
        Q = bno.quaternion
    except:
        print("imu error")
        continue
        

    t_curr = time.time()
    DT = t_curr-t_prev
    kalman.F = np.array([[1,0,0,DT,0,0],   #state transition matrix
                        [0,1,0,0,DT,0],
                        [0,0,1,0,0,DT],
                        [0,0,0,1,0,0],
                        [0,0,0,0,1,0],
                        [0,0,0,0,0,1]])
    kalman.B = np.array([[(DT**2)/2,0,0],    #control transition matrix
                         [0,(DT**2)/2,0],
                         [0,0,(DT**2)/2],
                         [DT,0,0],s
                         [0,DT,0],
                         [0,0,DT]])
    
    
    t_prev=t_curr
    
    #quaternino on socket
    send_quaternion_on_wi_fi = False
    if send_quaternion_on_wi_fi:
        message = "w" + str(Q[0]) + "w" + "a" + str(Q[1]) + "a" + "b" + str(Q[2]) + "b" + "c" + str(Q[3]) + "c"
        socket.sendto(message.encode("utf-8"),("192.168.1.204",5005))
    
    
    abs_acc = get_absolute_accelerations(Q,acc)
    print(abs_acc)
    
   
    update_gps()
    if not(gps_parser.altitude == previus_gps_reading[0] and gps_parser.latitude == previus_gps_reading[1] and gps_parser.longitude == previus_gps_reading[2] and gps_parser.speed == previus_gps_reading[3]):
        positionxy = cordinates_distance(starting_position, (convert_cordinates(gps_parser.latitude),convert_cordinates(gps_parser.longitude)))
        previus_gps_reading = (gps_parser.altitude,gps_parser.latitude,gps_parser.longitude,gps_parser.speed)
        z = np.array([positionxy[0],positionxy[1],gps_parser.altitude,gps_parser.speed[0],gps_parser.speed[1],gps_parser.speed[2]]).T
        #print("z:",z)
        kalman.update(z)
    else:
        #print("predict")
        kalman.predict(abs_acc.T)
    
    #print(kalman.x)
        
    #send altitude,vaertical velocity,position by the antenna
    

    if lora_time<time.time() and time.time() > start_time+2:
        lora_time = time.time()+0.2
        c+=1
        lora_message = "a"+str(round(kalman.x[2],1))+"b"+str(round(kalman.x[5],1))+"c"+str(round(kalman.x[3],2))+"d"+str(round(kalman.x[4],2))+"z"
        messages+=lora_message
        if c==7:
            #print(lora_message)
            lora.write(bytes(messages,"utf-8" ))
            messages=""
            c=0
        else:
            if lora.in_waiting:
                message_received = lora.read(lora.in_waiting).decode()
                print(message_received)
                if "oc" in message_received:
                    lora.write(b"oc_ok")
                    break
                if "sr" in message_received:    # only for debug               
                    lora.write(b"sr_ok")
                
        #print(lora_message)
        
        
    
      







