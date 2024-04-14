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
    
    bar02_output=Bar02()
    
    depth=bar02_output.read_depth()
    
    print("Depth: ", depth)
    
    time.sleep(3)
    
    depth=bar02_output.read_depth()
    
    print("Depth: ", depth)
    
    time.sleep(3)
    
    depth=bar02_output.read_depth()
    
    print("Depth: ", depth)
    
    time.sleep(3)
    
    exit