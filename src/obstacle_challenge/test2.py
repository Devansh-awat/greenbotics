import cv2
from src.sensors import camera
from src.motors import motor, servo
from src.obstacle_challenge.test3 import process_video_frame,annotate_video_frame
camera.initialize()
motor.initialize()
servo.initialize()
motor.forward(55)
try:
    while True:
        frame = camera.capture_frame()
        detections = process_video_frame(frame)
        detected_walls = detections['detected_walls']
        left_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_left')
        servo.set_angle((left_pixel_size-13000)*0.005)
        cv2.imshow('img',annotate_video_frame(frame,detections,'clockwise'))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    camera.cleanup()
    servo.cleanup()
    motor.cleanup()