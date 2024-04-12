class Dead_mans_switch():
  
    def __init__(self):
        # IP address of the device you want to check
        TARGET_IP = '192.168.1.164'
        # Number of allowed failed pings before triggering the action
        MAX_FAILED_ATTEMPTS = 3
        # Time between pings in seconds
        PING_INTERVAL = 5
        # Counter for failed attempts
        failed_attempts = 0

    async def is_active(self):
            while True:
            # Send a ping to the target IP
            response = os.system(f'ping -c 1 {TARGET_IP}')
    
            # Check the response
            if response == 0:
                print(f'Successfully pinged {TARGET_IP}')
                failed_attempts = 0
            else:
                print(f'Failed to ping {TARGET_IP}')
                failed_attempts += 1
        
                # If the number of failed attempts reaches the threshold, trigger the action
                if failed_attempts >= MAX_FAILED_ATTEMPTS:
                    print('Threshold reached. Triggering deadman\'s switch action.')
                    # Insert code to perform the action here
                    # For example, shut down the system: os.system('sudo shutdown now')
                    return False
                else:
                    return True
    
            # Wait for the next interval
            asyncio.sleep(PING_INTERVAL)
