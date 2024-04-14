
import smbus2 as smbus
import os
import asyncio
import time
import serial
import numpy as np

import threading

from controller import Controller
from bar02 import Bar02
from deadmansswitch import DeadmansSwitch
from initialize import *
from thrusters import control_thrusters

def main():
 
    setpoint = 0
   
    # Initialize PID parameters and setpoint
    Kp = 0.5
    Ki = 0.1
    Kd = 0.2

    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    
    filename = f"RunData_{timestamp}.csv"

    bar02_output=Bar02()

    pid_controller = Controller(Kp, Ki, Kd, setpoint)

    deadmans_switch = DeadmansSwitch(4)

    init=Initialize()

    init.init_gpio()
    
    depth=bar02_output.read_depth()

    data= np.array([depth, pid_controller.compute(), setpoint])

    def operation_AUV():
        # Check if Deadman's switch is active
        while True:
            # Read depth/pressure from Bar02 sensor
            depth=bar02_output.read_depth()
            
            new_data = np.array([depth, pid_output, setpoint])
            
            data= np.append(data, new_data, axis=0)
            
            np.savetxt(filename, data, delimiter=',')
           
            if deadmans_switch.is_active():
                
                # Compute control signal using PID controller
                pid_output = Controller.compute(depth)

                # Control thrusters based on PID output
                control_thrusters(pid_output)
                                
            else:
                # Shutdown AUV if Deadman's switch is not active
                break

    thread = threading.Thread(target=operation_AUV)
    thread.deamon = True
    thread.start()

    while True:
        def depth_request():
            user_input = input("Enter the desired depth: ")
            setpoint = user_input
            pid_controller = Controller(Kp, Ki, Kd, setpoint)
    
    




   

if __name__ == "__main__":
    main()


