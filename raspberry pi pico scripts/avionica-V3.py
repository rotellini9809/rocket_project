import time
import board
import busio
import digitalio
from ulab import numpy as np
import adafruit_gps
from adafruit_bno08x import (
 BNO_REPORT_ACCELEROMETER,
 BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
import circuitpython_csv as csv
import json



#----------------------------------------------

print(time.monotonic())


send_quaternion_on_wi_fi = False
if send_quaternion_on_wi_fi: 
    import socket
    socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

buzzer_pin = board.GP22
hot_wire_pin = board.GP18

buzzer = digitalio.DigitalInOut(buzzer_pin)
buzzer.direction = digitalio.Direction.OUTPUT
buzzer.value = False

hot_wire = digitalio.DigitalInOut(hot_wire_pin)
hot_wire.direction = digitalio.Direction.OUTPUT
hot_wire.value = False




uart_gps = busio.UART(board.GP4, board.GP5, baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart_gps, debug=False)

gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,1000')

gps_covariance_matrix = np.diag([6.25,6.25,6.25,0.01,0.01,0.01])

lora = busio.UART(board.GP16, board.GP17, baudrate=9600, bits=8, parity=None, stop=1, timeout=1)




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


def get_absolute_accelerations(Q, acc):
    rotation_matrix = quaternion_to_rotation_matrix(Q)
    #rotation_matrix = np.transpose(rotation_matrix)
    abs_acc = np.dot(rotation_matrix,acc)
    return abs_acc


#unused !!!!
def calibrate_accelerometer_mean():
    print("Calibrating accelerometer put the rocket nose up on a stable and flat surface")
    offsets = [0.0, 0.0, 0.0]  # Offsets for x, y, z axes
    
    z_gravity = 9.81
    NUM_SAMPLES = 500
    # Collect data for calibration
    for _ in range(NUM_SAMPLES):
        # Read raw acceleration data
        x,y,z = get_accel() 
        
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
    
def apply_calibration(raw_data,offsets):
    result = [0,0,0]
    for i in range(3):
        result[i] = raw_data[i] - offsets[i]
    return result

    
def get_gyro():
    gx,gy,gz = gyroscope.read()
    return gx,gy,gz
def get_accel():  
    ax,ay,az = bno.acceleration # read and convert accel data  
    return ax,ay,az
def get_mag():
    mx,my,mz = magnetometer.magnetic
    return mx,my,mz


class KF:
    def __init__(self):
        
        self.x = np.array([0.,0.,gps.altitude,gps.speed[0],gps.speed[1],gps.speed[2]]).T
        self.t_prev = time.time()
    def predict(self,acc):
        DT = self.compute_DT()
        F = np.array([[1,0,0,DT,0,0],   #state transition matrix
                            [0,1,0,0,DT,0],
                            [0,0,1,0,0,DT],
                            [0,0,0,1,0,0],
                            [0,0,0,0,1,0],
                            [0,0,0,0,0,1]])
        B = np.array([[(DT**2)/2,0,0],    #control transition matrix
                             [0,(DT**2)/2,0],
                             [0,0,(DT**2)/2],
                             [DT,0,0],
                             [0,DT,0],
                             [0,0,DT]])
        self.x = F @ self.x + B @ acc # acc shape = 3X1
        
    def update(self,x):
        self.t_prev=time.time()
        self.x = x
    
    def compute_DT(self):
        t_curr = time.time()
        DT = t_curr-self.t_prev
        self.t_prev=t_curr
        return DT
    
class Csv_writer:
    def __init__(self, column_names):
        self.file = open("flight_data.csv","w",newline="")
        self.writer = csv.DictWriter(self.file, fieldnames = column_names)
        self.writer.writeheader()
    def write_telemetry(self,row):
        self.writer.writerow(row)
    def close(self):
        self.file.close()

class Json_writer:
    def __init__(self):
        self.params = dict()
        return
    def write(self,new_params):
        self.params.update(new_params)
        with open("flight_params.json","w") as file:
            json.dump(self.params,file)
        return



t0 = time.time()
start_bool = False # if IMU start fails - stop calibration


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
        
        #buffer cleaning
        for i in range(50):
            get_accel()
            bno.quaternion
         
        break
    except:
        continue
    
if not start_bool:  
    print("IMU not Started")
    while True:
        pass
    


buzzer_pin.value = 1
time.sleep(0.5)
buzzer_pin.value = 0


    
    
active_calibration = True 
if active_calibration:
    offsets = calibrate_accelerometer_mean()
    print("calibrating magnetometer")
    bno.begin_calibration()
    if not bno.calibration_status:
        print("rotate the electronics")
        time.sleep(1)
else:
    offsets = [0,0,0]




wait_for_gps=False
if wait_for_gps:
    print("waiting for gps first packet to initialize")
    while gps.altitude == 0.0 or gps.latitude == [0, 0.0, 'N'] or gps.longitude == [0, 0.0, 'W']:
        gps.update()
    





need_start_from_console = False
if need_start_from_console:
    print("waiting for start command")
    while True:
        gps.update()
        if lora.in_waiting:
            message_received = lora.read(lora.in_waiting).decode()
            print(message_received)
            if "sr" in message_received:
                print("ollare")
                lora.write(b"sr_ok")
                break
        time.sleep(0.1)
        
json_writer.write({"start_time":""})


c=0

#kalman filter intializzation
previus_position = (convert_cordinates(gps.latitude),convert_cordinates(gps.longitude))
previus_gps_reading = (gps.altitude,gps.latitude,gps.longitude,gps.speed)
kalman = KF()


columns_name = ["time","x_rel_acc","y_rel_acc","z_rel_acc","x_abs_acc","y_abs_acc","z_abs_acc","x_pos","y_pos","z_pos"
                         "x_speed","y_speed","z_speed"]
csv_writer = Csv_writer(columns_name)
json_writer = Json_writer()
json_writer.write({"accelerometer_mean":offsets})

messages=""
start_time = time.time()
lora_time=start_time+0.2


print("started succesfully")
try:
    while True:
        
        try:
            unc_acc = get_accel()
            Q = bno.quaternion
            print(np.linalg.norm(Q))
        except:
            print("imu error")
            continue
        
        acc = np.array(apply_calibration(unc_acc,offsets))
        abs_acc = get_absolute_accelerations(Q,acc)
        #print("unc acc",acc[0],"  ",acc[1],"  ",acc[2])
        #print("rel acc",acc[0],"  ",acc[1],"  ",acc[2])
        #print("abs_acc",abs_acc[0],"  ",abs_acc[1],"  ",abs_acc[2])
        
            

        
        
        #quaterninon on socket
        if send_quaternion_on_wi_fi:
            message = "w" + str(Q[0]) + "w" + "a" + str(Q[1]) + "a" + "b" + str(Q[2]) + "b" + "c" + str(Q[3]) + "c"
            socket.sendto(message.encode("utf-8"),("192.168.1.204",5005))
        
        
        
       
        gps.update()
        if not(gps.altitude == previus_gps_reading[0] and gps.latitude == previus_gps_reading[1] and gps.longitude == previus_gps_reading[2] and gps.speed == previus_gps_reading[3]):
            actual_position = convert_cordinates(gps.latitude),convert_cordinates(gps.longitude)
            positionxy = cordinates_distance(previus_position, actual_position)
            previus_position = actual_position
            previus_gps_reading = (gps.altitude,gps.latitude,gps.longitude,gps.speed)
            z = np.array([positionxy[0],positionxy[1],gps.altitude,gps.speed[0],gps.speed[1],gps.speed[2]]).T
            #print("z:",z)
            kalman.update(z)
        else:
            #print("predict") 
            kalman.predict(np.transpose(abs_acc))
        
        # print(kalman.x)
        row = [time.time()- start_time]+ list(acc) + list(abs_acc) + list(kalman.x)
        row = [round(n,2) for n in row]
        print(row)
        csv_writer.write_telemetry({key:value for key, value in zip(columns_name,row)})
        #print(row)
        
        
        
        #send altitude,vaertical velocity,position by the antenna
        if lora_time<time.time() and time.time() > start_time+2:
            lora_time = time.time()+0.2
            c+=1
            lora_message = "a"+str(round(kalman.x[2],1))+"b"+str(round(kalman.x[5],1))+"c"+str(round(kalman.x[3],2))+"d"+str(round(kalman.x[4],2))+"z"
            messages+=lora_message
            if c==10:
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
except Exception as e:
    raise e
finally:
    json_writer.write({"error":str(e)})
    csv_writer.close()

