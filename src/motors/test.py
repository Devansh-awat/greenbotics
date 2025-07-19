from src.motors import motor, servo
import time

motor.initialize()
servo.initialize()
servo.set_angle(17)
time.sleep(5)
servo.cleanup()
motor.cleanup()
