def control_thrusters(pid_out):
    for i, pin in enumerate(ESC_PWM_PINS):
        
        # Scale PID output to ESC signal range (adjust as needed)
        Pulse_Width_Delta = int(pid_out * 1000 / 2)

        Pulse_Width_CW = 1500 + Pulse_Width_Delta
        Pulse_Width_CCW = 1500 - Pulse_Width_Delta

        # Limit throttle within range
        Pulse_Width_CW = max(min(Pulse_Width_CW, 1900), 1100)
        Pulse_Width_CCW = max(min(Pulse_Width_CCW, 1900), 1100)
        
        # Send PWM signal to ESC
        pwm_CW.ChangeDutyCycle(pw_to_dc(Pulse_Width_CW))
        pwm_CCW.ChangeDutyCycle(pw_to_dc(Pulse_Width_CCW))
