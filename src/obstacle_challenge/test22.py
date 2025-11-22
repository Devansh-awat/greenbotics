import os
import time
from src.motors import servo, motor
try:
        servo.initialize()
        #time.sleep(0.5)
        motor.initialize()
        #time.sleep(0.5)
        #servo.cleanup()
        #time.sleep(0.5)
        #smotor.cleanup()
except Exception as e:
    print(f"An error occurred: {e}")