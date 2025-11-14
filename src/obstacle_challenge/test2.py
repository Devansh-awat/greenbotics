import time
from src.sensors import bno055
from src.motors import servo,motor
from src.obstacle_challenge.main import get_angular_difference
bno055.initialize()
servo.initialize()
motor.initialize()
motor.forward(50)
servo.set_angle_unlimited(-60)
while get_angular_difference(bno055.get_heading(), 270)>5:
    time.sleep(0.01)
print(bno055.get_heading())
motor.reverse(50)
while get_angular_difference(bno055.get_heading(), 5)>5:
    time.sleep(0.01)
print(bno055.get_heading())
motor.brake()
print(bno055.get_heading())
time.sleep(5)
print(bno055.get_heading())
motor.cleanup()
print(bno055.get_heading())
servo.cleanup()
print(bno055.get_heading())
bno055.cleanup()