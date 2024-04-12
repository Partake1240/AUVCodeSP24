# Define a Deadman's switch class
class DeadmansSwitch:
    def __init__(self, max_checks):
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
