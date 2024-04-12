#This will create a class that initializes and reads the Bar02 Pressure Sensor
class Bar02:
    def __init__(self,fluid_density):
        
        sensor = ms5837.MS5837_02BA() 
        
        # We must initialize the sensor before reading it
        if not sensor.init():
        print("Sensor could not be initialized")
        exit(1)


        # Define the I2C bus number (usually 1 on Raspberry Pi)
        self.bus = smbus.SMBus(1)
        # Bar02 sensor address
        self.BAR02_ADDRESS = 0x77

        sensor.setFluidDensity(fluid_density) # kg/m^3

    # Send command to Bar02 sensor to request pressure measurement
    def read_depth(self):
    
        # We have to read values from sensor to update pressure and temperature
        if not sensor.read():
            print("Sensor read failed!")
            exit(1)
    
        time.sleep(0.05)
       

        return sensor.depth()