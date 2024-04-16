import time
import board
import adafruit_lsm303dlh_mag
import adafruit_lsm303_accel
import l3gd20
import smbus
import numpy as np  
import csv
import serial
from micropyGPS import MicropyGPS
from filterpy.kalman import KalmanFilter

#from myKalmanFilter import MyKalmanFilter
from imu_calibration2 import calibrate

#----------------------------------------------
#part to send quaternions to my laptop to see the rotation of the rocket
import socket
socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)



gps_parser = MicropyGPS()
gps = serial.Serial('/dev/ttyACM0',9600)
gps_covariance_matrix = np.diag([6.25,6.25,6.25,0.01,0.01,0.01])

lora = serial.Serial('/dev/ttyAMA0', baudrate=9600, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)


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
    
def quaternion_to_rotation_matrix(quaternion):
    
    w, x, y, z = quaternion
    rotation_matrix = np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x**2 - 2*y**2]
    ])
    return rotation_matrix


def get_absolute_accelerations(Q, acc):
    rotation_matrix = quaternion_to_rotation_matrix(Q)
    # abs_acc = rotation_matrix^-1 @ acc
    inverse_rotation = np.linalg.inv(rotation_matrix)
    abs_acc = np.dot(inverse_rotation,acc.reshape(3,1))
    # we need to account for the magnetic declination offset that is the difference between the true north and the magnetic north
    # in rome the magn. decl. offset is 3.8833333333333333 degree east
    # to compute the transformation matrix that will rotate the acceleration vector i used this procedure:
    
    # Convert the rotation angle to radians
    # rotation_angle_rad = np.radians(3.8833333333333333)
    # Define the rotation matrix for rotation around the z-axis
    # rotation_matrix = np.array([
    #     [np.cos(rotation_angle_rad), -np.sin(rotation_angle_rad), 0],
    #     [np.sin(rotation_angle_rad), np.cos(rotation_angle_rad), 0],
    #     [0, 0, 1]
    # ])
    
    # so the transformation matrix is:
    transformation_matrix = np.array([[ 0.99770402, -0.06772507,  0.        ],
                                  [ 0.06772507,  0.99770402,  0.        ],
                                  [ 0.        ,  0.        ,  1.        ]])
    #we apply the transformation matrix
    return np.dot(transformation_matrix,abs_acc).reshape(3)


t0 = time.time()  
start_bool = False # if IMU start fails - stop calibration
cal_filename = '9dof_cal.csv'
while time.time()-t0<2:  
    try:
        
        bus = smbus.SMBus(1)
        time.sleep(1) # wait for i2c to stabilize
        i2c = board.I2C()
        global acc
        accelerometer = adafruit_lsm303_accel.LSM303_Accel(i2c)
        global mag
        magnetometer = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
        global gyroscope
        gyroscope = l3gd20.L3GD20(bus)
        gyroscope.set_range(l3gd20.RANGE_250DPS)
        start_bool = True  
        break  
    except:  
        continue  
 
if not start_bool:  
    print("IMU not Started - Check Wiring and I2C")
    
    
    
def get_gyro():
    gx,gy,gz = gyroscope.read()
    return gx,gy,gz
def get_accel():  
    ax,ay,az = accelerometer.acceleration # read and convert accel data  
    return ax,ay,az
def get_mag():
    mx,my,mz = magnetometer.magnetic
    return mx,my,mz


    
'''
print("type ctrl + c to calibrate")
try:
    time.sleep(3)
except KeyboardInterrupt:
    calibrate(True)    

print("no cal selected")
'''  
    
    

# read calibration coefficients from file
cal_offsets = [[],[],[],0.0,0.0,0.0,[],[],[],0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0] # cal vector 
with open(cal_filename,'r',newline='') as csvfile:  
    reader = csv.reader(csvfile,delimiter=',')  
    iter_ii = 0  
    for row in reader:
        if len(row)>2:  
            row_vals = [float(ii) for ii in row[int((len(row)/2)+1):]]  
            cal_offsets[iter_ii] = row_vals  
        else:  
            cal_offsets[iter_ii] = float(row[1])  
        iter_ii+=1

acc_covariance_matrix = np.array(cal_offsets[9:18]).reshape((3,3))

        
#print(cal_offsets)

# reshaping cal_offsets to make matrix multiplication later
acc_m = np.array([a[0] for a in cal_offsets[0:3]]).reshape(1,3)
acc_b = np.array([a[1] for a in cal_offsets[0:3]]).reshape(1,3)
gyro_b = np.array(cal_offsets[3:6]).reshape(1,3)
mag_b = np.array(cal_offsets[6:9]).reshape(1,3)



from ahrs.filters import Madgwick

# initializing extended kalman filter for estimating attitude
madgwick = Madgwick()
Q = np.array([[1.,0.,0.,0.]])

t_prev = time.time()
c=0

print("waiting for gps first packet to initialize")
while gps_parser.altitude == 0.0 or gps_parser.latitude == [0, 0.0, 'N'] or gps_parser.longitude == [0, 0.0, 'W']:
    update_gps()
    





need_start_from_console = True
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
a1 = np.eye(6)
a1[:3, :3] =  acc_covariance_matrix
print(a1)
kalman.Q = a1
kalman.H = np.eye(6)
    
#buffer cleaning
for i in range(50):
    get_gyro()
    get_mag()
    get_accel()

messages=""
start_time = time.time()
lora_time=start_time+0.2


print("started succesfully")
while True:
    try:
        acc = np.array(get_accel()).reshape(1,3)
        acc_cal = (acc * acc_m + acc_b).reshape(3)

        gyro = np.array(get_gyro()).reshape(1,3)
        gyro_cal = (gyro - gyro_b).reshape(3)

        mag = np.array(get_mag()).reshape(1,3)
        mag_cal = (mag- mag_b).reshape(3)
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
                         [DT,0,0],
                         [0,DT,0],
                         [0,0,DT]])
    
    madgwick.DT = DT
    np.append(Q,madgwick.updateIMU(q = Q[-1],gyr = gyro_cal,acc = acc_cal))
    t_prev=t_curr
    
    message = "w" + str(Q[-1][0]) + "w" + "a" + str(Q[-1][1]) + "a" + "b" + str(Q[-1][2]) + "b" + "c" + str(Q[-1][3]) + "c"
    socket.sendto(message.encode("utf-8"),("192.168.1.204",5005))
    
    
    abs_acc = get_absolute_accelerations(Q[-1],acc_cal)
    #print(abs_acc)
    
   
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
        if c==5:
            print(lora_message)
            lora.write(bytes(messages,"utf-8" ))
            messages=""
            c=0
        else:
            if lora.in_waiting:
                message_received = lora.read(lora.in_waiting).decode()
                print(message_received)
                if "oc" in message_received:
                    lora.write(b"oc_ok")
                if "sr" in message_received:    # only for debug               
                    lora.write(b"sr_ok")
                
        #print(lora_message)
        
        
    
      







