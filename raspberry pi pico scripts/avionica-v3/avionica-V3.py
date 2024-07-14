import time
import board
import busio
import digitalio
from ulab import numpy as np
import adafruit_gps
from adafruit_bno08x import (
 BNO_REPORT_ACCELEROMETER,
# BNO_REPORT_GYROSCOPE,
# BNO_REPORT_MAGNETOMETER,
 BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C
import circuitpython_csv as csv
import json
from sensors import Buzzer,IMU, HotWire
from KalmanFilter import KF



#----------------------------------------------


t0 = time.monotonic()

buzzer = Buzzer(board.GP12)
buzzer.startupBuzz()

hot_wire = HotWire(board.GP18)

imu = IMU(board.GP11, board.GP10, False )


lora = busio.UART(board.GP16, board.GP17, baudrate=9600, bits=8, parity=None, stop=1, timeout=1)




uart_gps = busio.UART(board.GP4, board.GP5, baudrate=9600, timeout=10)
gps = adafruit_gps.GPS(uart_gps, debug=False)

gps.send_command(b'PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
gps.send_command(b'PMTK220,1000')

gps_covariance_matrix = np.diag([6.25,6.25,6.25,0.01,0.01,0.01])

buzzer.calibrationCompleteBuzz()

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
    
    

    
class Csv_writer:
    def __init__(self,write_mode, column_names):
        self.write_mode = write_mode
        self.column_names = column_names
        if not self.write_mode: return
        self.file = open("flight_data.csv", "w")
        self.file.write(",".join(column_names) + "\n")  # Write header
        

    def write_telemetry(self, row):
        if not self.write_mode: return
        values = [str(row.get(col, "")) for col in self.column_names]
        self.file.write(",".join(values) + "\n")

    def close(self):
        if not self.write_mode: return
        self.file.close()

class Json_writer:
    def __init__(self,write_mode):
        self.write_mode = write_mode
        self.params = dict()
        return
    def write(self,new_params):
        self.params.update(new_params)
        if not self.write_mode: return
        with open("flight_params.json","w") as file:
            json.dump(self.params,file)
        return







write_mode = False
json_writer = Json_writer(write_mode)
csv_writer = Csv_writer(write_mode,["time","x_abs_acc","y_abs_acc","z_abs_acc","x_pos","y_pos","z_pos","x_speed","y_speed","z_speed"])
json_writer.write({"accelerometer_mean":imu.offsets})


send_quaternion_on_wi_fi = False
if send_quaternion_on_wi_fi: 
    import socket
    socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

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
        
wait_for_gps=False
if wait_for_gps:
    print("waiting for gps first packet to initialize")
    while gps.altitude_m == None or gps.latitude == None or gps.longitude == None:
        gps.update()
        
json_writer.write({"started":True})


c=0

#kalman filter intializzation
previus_position = None
previus_gps_reading = (None,None,None,None)
kalman = KF()





messages=""
start_time = time.time()
lora_time=start_time+0.2


print("started succesfully")
try:
    while True:
        try:
            abs_acc,Q=imu.get_absolute_accelerations_and_quaternion()
        except:
            print("imu error")
            continue
     
        
        #quaterninon on socket
        if send_quaternion_on_wi_fi:
            message = "w" + str(Q[0]) + "w" + "a" + str(Q[1]) + "a" + "b" + str(Q[2]) + "b" + "c" + str(Q[3]) + "c"
            socket.sendto(message.encode("utf-8"),("192.168.1.204",5005))
        
        
        
       
        gps.update()
        if gps.has_fix and previus_position:
            actual_position = convert_cordinates(gps.latitude),convert_cordinates(gps.longitude)
            positionxy = cordinates_distance(previus_position, actual_position)
            previus_position = actual_position
            previus_gps_reading = (gps.altitude_m,gps.latitude,gps.longitude,gps.speed)
            z = np.array([positionxy[0],positionxy[1],gps.altitude_m,gps.speed[0],gps.speed[1],gps.speed[2]]).T
            #print("z:",z)
            kalman.update(z)
        else:
            #print("predict") 
            kalman.predict(abs_acc.T)
        
        # print(kalman.x)
        row = [time.time()- start_time] + list(abs_acc) + list(kalman.x)
        row = [round(n,2) for n in row]
        print(row)
        csv_writer.write_telemetry({key:value for key, value in zip(csv_writer.column_names,row)})
        #print(row)
        
        
        
        #send altitude_m,vaertical velocity,position trought the antenna
        if lora_time<time.time() and time.time() > start_time+2:
            
            lora_time = time.time()+0.2
            c+=1
            lora_message = "a"+str(round(kalman.x[2],1))+"b"+str(round(kalman.x[5],1))+"c"+str(round(kalman.x[3],2))+"d"+str(round(kalman.x[4],2))+"z"
            messages+=lora_message
            
            if c==5:
                
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
    json_writer.write({"error":str(e)})
    raise e
finally:
    csv_writer.close()

