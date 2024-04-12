# Define a PID Controller class
class Controller:
    #Each Time the Controller is initialized, the following parameters are passed.
    #When the setpoint is changed, the controller is reinitialized.
    
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp  # Proportional gain
        self.Ki = Ki  # Integral gain
        self.Kd = Kd  # Derivative gain
        self.setpoint = setpoint  # Desired depth
        self.prev_error = 0  # Previous error for computing derivative term
        self.integral = 0  # Integral term for accumulated error

    def compute(self, depth):
        #Compute error
        error = self.setpoint - depth 

        #Proportional Term
        P=self.Kp*error

        #Integral Term
        self.integral += error  # Accumulate error for integral term
        I=self.Ki*self.integral

        #Derivative Term
        D=self.Kd*(error - self.prev_error)
        #INPUT FILTERING

        #Update previous error for next iteration
        self.prev_error = error  
        
        return P+I+D