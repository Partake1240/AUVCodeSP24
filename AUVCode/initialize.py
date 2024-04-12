 def Initialize()
 
     # Initialize PID parameters and setpoint
        Kp = 0.5
        Ki = 0.1
        Kd = 0.2

    # Define GPIO pins for controlling ESCs (adjust pin numbers as needed)
    #CW pins should be the first 2, CCW pins should be last 2
    ESC_PWM_PINS_CW = [18, 12]
    ESC_PWM_PINS_CCW = [13,19]

#This outputs a duty cycle for the ESCs based on the requested pulse width
def pw_to_dc(pulse_width)
        return (pulse_width/10^6)*100*frequency

# Initialize GPIO
def init_gpio():
        GPIO.setmode(GPIO.BCM)
        for pin in ESC_PWM_PINS_CW:
            GPIO.setup(pin, GPIO.OUT)
            frequency = 50  # Set PWM frequency to 50 Hz
            pwm_CW= GPIO.PWM(pin, frequency)
            NEUTRAL_DUTY_CYCLE = pw_to_dc(1500)
            pwm_CW.start(NEUTRAL_DUTY_CYCLE)  # Start PWM signal with neutral throttle

        for pin in ESC_PWM_PINS_CW:
            GPIO.setup(pin, GPIO.OUT)
            frequency = 50  # Set PWM frequency to 50 Hz
            pwm_CCW= GPIO.PWM(pin, frequency)
            NEUTRAL_DUTY_CYCLE = pw_to_dc(1500)
            pwm_CCW.start(NEUTRAL_DUTY_CYCLE)  # Start PWM signal with neutral throttle
        
    
