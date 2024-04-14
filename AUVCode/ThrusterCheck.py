import smbus2 as smbus
import os
import asyncio
import time
import serial
from distutils.command import check
import numpy as np

import threading

from controller import Controller
from bar02 import Bar02
from deadmansswitch import DeadmansSwitch
from initialize import *
from thrusters import control_thrusters

def main():
    
    init=Initialize()

    init.init_gpio()

    check_value=0.5
    
    control_thrusters(check_value)
    
    time.sleep(2)

    check_value = 0
    
    control_thrusters(check_value)
    
    time.sleep(.5)
    
    check_value= -0.5
    
    control_thrusters(check_value)
    
    time.sleep(2)
    
    check_value = 0
    
    control_thrusters(check_value)
    
    time.sleep(.5)
    
    exit()
    
    if __name__ == "__main__":
        main()
        