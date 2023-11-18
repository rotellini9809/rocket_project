import serial
from collections import deque
import time


class Serial_parser:
    def __init__(self,port = "COM3") -> None:
        self.ser = serial.Serial(port) 
        self.remaining_message = ""
        self.all_data = deque()
        self.message=""

    #def reset_bool():


    def parse(self):
        n = self.ser.in_waiting
        if n > 0:
            try:
                self.message = self.remaining_message + str(self.ser.readline().decode()).replace('\r\n', '')
            except:
                self.remaining_message = ""
                return None
            c = 0
            ia = 0
            ib = 0
            ic = 0
            id = 0
            iz = 0
            bool_a = False
            bool_b = False
            bool_c = False
            bool_d = False
            bool_z = False
            
            for i,character in enumerate(self.message):
                if character=="a":
                    if not bool_a:
                        ia=i
                        bool_a = True
                    else:
                        bool_a = False
                        bool_b = False
                        bool_c = False
                        bool_d = False
                        bool_z = False
                    
                if bool_a and character=="b":
                    if not bool_b:
                        ib=i
                        bool_b = True
                    else:
                        bool_a = False
                        bool_b = False
                        bool_c = False
                        bool_d = False
                        bool_z = False
                if bool_b and character=="c":
                    if not bool_c:
                        ic=i                        
                        bool_c = True
                    else:
                        bool_a = False
                        bool_b = False
                        bool_c = False
                        bool_d = False
                        bool_z = False
                if bool_c and character=="d":
                    if not bool_d:
                        id=i                    
                        bool_d = True
                    else:
                        bool_a = False
                        bool_b = False
                        bool_c = False
                        bool_d = False
                        bool_z = False
                if bool_d and not bool_z and character=="z":
                    iz=i
                    bool_z=True
                    self.remaining_message = self.message[iz+1:]
                    try:
                        data = float(self.message[ia+1:ib]),float(self.message[ib+1:ic]),float(self.message[ic+1:id]),float(self.message[id+1:iz])
                        self.all_data.append(data)
                    except:
                        pass
                    bool_a = False
                    bool_b = False
                    bool_c = False
                    bool_d = False
                    bool_z = False

            if len(self.all_data)>0:
                return self.all_data.popleft()
                
            
        else:
            self.remaining_message = ""
            if len(self.all_data)>0:
                return self.all_data.popleft()
            

    def start_rocket_cond(self):
        self.ser.write(b"sr\n")
        end = time.time()+1
        while time.time()<end:
            time.sleep(0.15)
            if "sr_ok" in self.message or self.parse() != None:
                return True
        return False


    
    def open_chute_cond(self):
        self.ser.write(b"oc\n")
        end = time.time()+1
        while time.time()<end:
            time.sleep(0.15)
            if "oc_ok" in self.message:
                return True
        return False

if __name__ == "__main__":
    parser = Serial_parser()
    while True:
        message = parser.parse()
        if message:
            print(message)


#altitude, vertical velocity, position
#print(serial.tools.list_ports())