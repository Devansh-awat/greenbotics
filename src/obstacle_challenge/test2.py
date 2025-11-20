from src.motors import servo, motor
import time
servo.initialize()
motor.initialize()
servo.set_angle(0)
motor.forward(100)
time.sleep(1.5)
servo.cleanup()
motor.cleanup()