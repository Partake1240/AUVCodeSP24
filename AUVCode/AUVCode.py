
import smbus2 as smbus
import os
import asyncio
import time
import serial
import numpy as np
import RPi.GPIO as GPIO  # Import GPIO library for Raspberry Pi
import threading
import ms5837
import controller
import car02
import deadmansSwitch
from initialize import *
from thrusters import control_thrusters

def main():
 
    setpoint = 0

    bar02_output=Bar02()

    pid_controller = Controller(Kp, Ki, Kd, setpoint)

    deadmans_switch = DeadmansSwitch(4)

    Initialize()

    init_gpio()

    data= np.array([depth, pid_output, setpoint])

    def operation_AUV():
        while True:
            # Check if Deadman's switch is active
            new_data = np.array([depth, pid_output, setpoint])
            
            data= np.append(data, new_data, axis=0)
           
            if deadmans_switch.is_active():
                # Read depth/pressure from Bar02 sensor
                depth=bar02_output.read_depth()

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
    
    timestamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    filename = f"RunData_{timestamp}.csv"
    np.savetxt(filename, my_matrix, delimiter=',')




   

if __name__ == "__main__":
    main()


