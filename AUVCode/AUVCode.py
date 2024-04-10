
import smbus
import time
import RPi.GPIO as GPIO  # Import GPIO library for Raspberry Pi
import serial

# Define a PIDController class
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired depth

        self.prev_error = 0  # Previous error for computing derivative term
        self.integral = 0  # Integral term for accumulated error

    def compute(self, feedback_value):
        error = self.setpoint - feedback_value  # Compute error
        self.integral += error  # Accumulate error for integral term
        derivative = error - self.prev_error  # Compute derivative term
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative  # Compute PID output
        self.prev_error = error  # Update previous error for next iteration
        return output

# Define a Deadman's switch class
class DeadmansSwitch:
    def __init__(self, max_checks=4):
        self.check_counter = 0  # Initialize check counter
        self.max_checks = max_checks  # Maximum number of checks allowed

    def check_connection(self):
        try:
            # Send a test command and wait for a response
            self.serial.write(b'ping\n')
            response = self.serial.readline().strip()

            # Check if response is as expected
            if response == b'pong':
                return True  # Connection successful
            else:
                return False  # Unexpected response, connection failed
        except serial.SerialException:
            return False  # Serial communication error, connection failed

    def is_active(self):
        if self.check_connection():
            self.check_counter = 0  # Reset counter if connection is successful
            return True
        else:
            self.check_counter += 1  # Increment counter if connection fails
            if self.check_counter >= self.max_checks:
                print("Connection failed {} times. Shutting down AUV.".format(self.max_checks))
                return False  # Trigger shutdown if connection fails consecutively
            else:
                return True

# Define GPIO pins for controlling ESCs (adjust pin numbers as needed)
ESC_PWM_PINS = [18, 19, 12, 13]

# Initialize GPIO
def init_gpio():
    GPIO.setmode(GPIO.BCM)
    for pin in ESC_PWM_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

# Function to read pressure from Bar02 sensor
def read_pressure():
    # Define the I2C bus number (usually 1 on Raspberry Pi)
    bus = smbus.SMBus(1)

    # Bar02 sensor address
    BAR02_ADDRESS = 0x77

    # Send command to Bar02 sensor to request pressure measurement
    bus.write_byte(BAR02_ADDRESS, 0x50)  # 0x50 is the command for pressure measurement

    # Wait for measurement to be ready (takes a short time)
    time.sleep(0.05)

    # Read pressure data (3 bytes)
    data = bus.read_i2c_block_data(BAR02_ADDRESS, 0x00, 3)

    # Convert data to pressure value (in mbar)
    pressure = (data[0] << 16 | data[1] << 8 | data[2]) / 100.0  # Pressure in mbar
    return pressure

# Function to control thrusters based on PID output
def control_thrusters(pid_output):
    for i, pin in enumerate(ESC_PWM_PINS):
        # Scale PID output to ESC signal range (adjust as needed)
        throttle = int(pid_output * 1000 / 2 + 1500)

        # Limit throttle within range (adjust as needed)
        throttle = max(min(throttle, 2000), 1000)

        # Send PWM signal to ESC
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(throttle / 1000000.0)
        GPIO.output(pin, GPIO.LOW)

# Example usage
if __name__ == "__main__":
    # Initialize GPIO
    init_gpio()

    # Initialize serial connection
    serial_port = '/dev/ttyUSB0'  # Adjust port as needed
    serial_baudrate = 9600  # Adjust baudrate as needed
    serial_timeout = 1  # Adjust timeout as needed
    serial = serial.Serial(serial_port, serial_baudrate, timeout=serial_timeout)

    # Initialize PID parameters and setpoint
    Kp = 0.5
    Ki = 0.1
    Kd = 0.2
    setpoint = 10  # Desired depth in meters (adjust as needed)

    # Create PID controller object
    pid_controller = PIDController(Kp, Ki, Kd, setpoint)

    # Create Deadman's switch object with maximum checks set to 4
    deadmans_switch = DeadmansSwitch(max_checks=4)

    # Simulate AUV control loop
    while True:
        # Check if Deadman's switch is active
        if deadmans_switch.is_active():
            # Read depth/pressure from Bar02 sensor
            depth_data = read_pressure()

            # Compute control signal using PID controller
            control_signal = pid_controller.compute(depth_data)

            # Control thrusters based on PID output
            control_thrusters(control_signal)

            # Simulate delay or computation time before next iteration
            time.sleep(1)  # Adjust this delay as needed
        else:
            # Shutdown AUV if Deadman's switch is not active
            break
