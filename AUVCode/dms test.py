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
import dms_2
from initialize import *
from thrusters import control_thrusters

def main():

    deadmans_switch = Dead_mans_switch()

    if deadmans_switch.is_active():
        print("Deadman's switch is active")
        time.sleep(5)

        else:
            break

if __name__ == "__main__":
    main()