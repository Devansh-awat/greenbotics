import time
from src.sensors import bno055
from src.motors import servo,motor
from src.obstacle_challenge.main import get_angular_difference
bno055.initialize()
servo.initialize()
motor.initialize()
motor.reverse(30)
time.sleep(1)
motor.brake()
motor.cleanup()
servo.cleanup()
bno055.cleanup()