import tkinter as tk
import threading
import random
import time
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np
from tkinter import ttk
from serial_parser import Serial_parser 

class AltitudeApp:
    def __init__(self, root):
        self.serial_parser = Serial_parser()

        self.root = root
        self.root.title("rocket gui")
        self.root.geometry("900x350")
        

        self.altitude_value = 0
        self.max_altitude_value = 0

        self.rely = 0.1

        self.font = ("Helvetica", 16)

        self.altitude_label = tk.Label(self.root, text="Altitude: {}".format(self.altitude_value), font = self.font)
        self.altitude_label.place(relx=0.60, rely=self.rely, anchor=tk.CENTER)

        self.max_altitude_label = tk.Label(self.root, text="Max Altitude: {}".format(self.max_altitude_value), font = self.font)
        self.max_altitude_label.place(relx=0.80, rely=self.rely, anchor=tk.CENTER)

        style = ttk.Style()
        style.map("C.TButton",
            foreground=[("disabled", "black")],
            background=[("active", "green"), ("disabled", "red")],
            font=[("disabled", self.font),("active", self.font),("!active", self.font)])

        self.start_rocket_button = ttk.Button(self.root, text="Start Rocket", command=self.start_rocket, style="C.TButton")
        self.start_rocket_button.place(relx=0.20, rely=self.rely, anchor=tk.CENTER)

        self.open_chute_button = ttk.Button(self.root, text="Open Chute", command=self.open_chute, state=tk.DISABLED,  style="C.TButton")
        self.open_chute_button.place(relx=0.40, rely=self.rely, anchor=tk.CENTER)

        self.connection_status_label = tk.Label(self.root, text="connection stable", font = self.font)
        self.connection_status_label.place(relx=0.5, rely=0.9, anchor=tk.CENTER)
        self.connection_status_label["fg"] = "green"


        
        self.labels = ["vertical velocity", "altitude", "position"]
        self.ylabels = ["m/s","m","north"]
        self.fig = Figure(figsize=(10, 2), dpi=100)
        self.fig.subplots_adjust(wspace=0.5, bottom=0.2)
        self.axes = [self.fig.add_subplot(1, 3, i+1) for i in range(3)]
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.root)
        self.canvas_widget = self.canvas.get_tk_widget()
        self.canvas_widget.place(relx=0.5, rely=0.55, anchor=tk.CENTER)
        


        self.running = True
        self.update_thread = threading.Thread(target=self.update_labels_and_graph_thread)
        self.update_thread.daemon = True
        self.update_thread.start()

        self.start_rocket_thread = threading.Thread(target=self.start_rocket_thread_func)
        self.start_rocket_thread.daemon = True

        self.open_chute_thread = threading.Thread(target=self.open_chute_thread_func)
        self.open_chute_thread.daemon = True

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

        

    def update_labels_and_graph_thread(self):

        
        
        altitudes = []
        vertical_velocities = []
        one_x=0
        x = []

        values_of_position = 50
        positions_x = [0]*values_of_position
        positions_y = [0]*values_of_position

        alphas = np.linspace(0, 1,values_of_position)
        lines = [ax.plot([0], [0])[0] for ax in self.axes]
        point, = self.axes[2].plot([], [], 'ro',alpha=0.8)
        



        

        while self.running:
            data = self.serial_parser.parse()
            if not data:
                self.change_connection_status_label(False)
                continue
            altitude,vertical_velocity,position_x,position_y = data
            #label update
            self.altitude_value = altitude
            self.max_altitude_value = max(self.altitude_value, self.max_altitude_value)
            
            self.altitude_label.config(text="Altitude: {}".format(self.altitude_value))
            self.max_altitude_label.config(text="Max Altitude: {}".format(self.max_altitude_value))

            #graph update
            self.axes[0].set_xticks([])
            self.axes[1].set_xticks([])
            self.axes[2].set_xlabel("west")
            for i,line in enumerate(lines):
                
                self.axes[i].set_title(self.labels[i])  # Set graph title
                self.axes[i].set_ylabel(self.ylabels[i])
            
            one_x += 1
            x.append(one_x)
            altitudes.append(altitude)
            vertical_velocities.append(vertical_velocity)
            for i in range(2):
                
                lines[i].set_xdata(x)
            lines[0].set_ydata(vertical_velocities)
            lines[1].set_ydata(altitudes)

            positions_x.append(position_x)
            positions_y.append(position_y)
            
            positions_x.pop(0)
            positions_y.pop(0)

            lines[2].set_data(positions_x,positions_y)
            point.set_data([position_x,position_y])

            
            for ax in self.axes:
                ax.relim()
                ax.autoscale_view(True, True, True)

            self.canvas.draw()
            self.canvas.flush_events()

            self.change_connection_status_label(True)
            
            time.sleep(0.15)

    
    def change_connection_status_label(self,status):
        if status and self.connection_status_label["fg"] == "red":
            self.connection_status_label["fg"] = "green"
            self.connection_status_label["text"] = "connection stable"
        elif not status and self.connection_status_label["fg"] == "green":
           self.connection_status_label["fg"] = "red"
           self.connection_status_label["text"] = "connection lost"


        

    def open_chute(self):
        self.open_chute_button.config(state=tk.DISABLED, text="chute opened")
        self.open_chute_thread.start()
        
    
    def open_chute_thread_func(self):
        while self.running:
            if self.serial_parser.open_chute_cond():
                break
            time.sleep(0.2)
        print("chute opened")

    def start_rocket(self):
        self.start_rocket_button.config(state=tk.DISABLED, text="rkt started")
        self.start_rocket_thread.start()

    def start_rocket_thread_func(self):
        while self.running:
            if self.serial_parser.start_rocket_cond():
                self.open_chute_button.config(state=tk.NORMAL)
                break
            


    def on_closing(self):
        self.running = False
        self.root.destroy()
        

    

if __name__ == "__main__":
    root = tk.Tk()
    app = AltitudeApp(root)
    root.mainloop()


#start rocket, open chute, 
#altitude, vertical velocity, position