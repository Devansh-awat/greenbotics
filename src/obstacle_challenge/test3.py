from src.motors import motor,servo
import time
motor.initialize()
servo.initialize()
servo.set_angle(9)
motor.forward(100)
time.sleep(1)
servo.set_angle(-45)
time.sleep(1)
motor.cleanup()
servo.cleanup()
