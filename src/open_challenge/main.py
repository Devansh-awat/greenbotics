from collections import deque
import time
import threading
import cProfile
import cv2
import numpy as np
from gpiozero import Button

# Import all settings from the new config file
from src.obstacle_challenge.main import get_angular_difference
from src.obstacle_challenge.test3 import steer_with_gyro
import src.open_challenge.config as config

# Import hardware control modules
from src.sensors import bno055, camera, distance
from src.motors import motor, servo


class CameraThread(threading.Thread):
    def __init__(self, camera_instance):
        super().__init__()
        self.camera = camera_instance
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = True
        self.daemon = True

    def run(self):
        while self.running:
            frame = self.camera.capture_frame()
            with self.lock:
                self.latest_frame = frame

    def get_frame(self):
        with self.lock:
            if self.latest_frame is not None:
                return self.latest_frame.copy()
            return None

    def stop(self):
        self.running = False

class SensorThread(threading.Thread):
    def __init__(self, bno, dist, init_event):
        super().__init__()
        self.bno = bno
        self.dist = dist
        self.initialization_complete = init_event
        self.lock = threading.Lock()
        self.running = True
        self.daemon = True
        self.heading = None
        self.distance_left = None
        self.distance_right = None
        self.distance_back = None
        self.distance_center = None

    def run(self):
        try:
            self.bno.initialize()
            print("SensorThread: Initializing distance sensors...")
            self.dist.initialise()
            print("SensorThread: Distance sensors initialized.")
            self.initialization_complete.set()
            while self.running:
                heading = self.bno.get_heading()
                dist_left = self.dist.get_distance(0)
                dist_center = self.dist.get_distance(2)
                dist_right = self.dist.get_distance(3)
                dist_back = self.dist.get_distance(-1)
                with self.lock:
                    self.heading = heading
                    self.distance_left = dist_left
                    self.distance_center = dist_center
                    self.distance_right = dist_right
                    self.distance_back = dist_back
        finally:
            print("SensorThread: Cleaning up distance sensors...")
            self.dist.cleanup()
            self.bno.cleanup()
            print("SensorThread: Distance sensor cleanup complete.")

    def get_readings(self):
        with self.lock:
            return {
                'heading': self.heading,
                'distance_left': self.distance_left,
                'distance_center': self.distance_center,
                'distance_right': self.distance_right,
                'distance_back' : self.distance_back
            }

    def stop(self):
        self.running = False


def process_video_frame(frame):
    processed_data = {
        'detected_walls': [],
        'detected_orange': [],
        'detected_close_black': []
    }
    
    frame = cv2.GaussianBlur(frame,(1,7),0)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_black = cv2.inRange(hsv_frame, config.LOWER_BLACK, config.UPPER_BLACK)
    mask_orange = cv2.inRange(hsv_frame, config.LOWER_ORANGE, config.UPPER_ORANGE)

    # Apply ROI masks to get detections only in the desired areas
    final_mask_walls = cv2.bitwise_and(mask_black, config.roi_mask_walls)
    final_mask_orange = cv2.bitwise_and(mask_orange, config.roi_mask_orange)
    final_mask_close_black = cv2.bitwise_and(mask_black, config.roi_mask_close_black)

    # Detect orange line for turn counting
    contours, _ = cv2.findContours(final_mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        biggest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest_contour)
        if area > config.ORANGE_MIN_AREA:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                processed_data['detected_orange'].append({'area': area, 'centroid': (cx, cy)})

    # Detect "close black" walls for sharp turn avoidance
    contours, _ = cv2.findContours(final_mask_close_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > config.WALL_MIN_AREA:
            processed_data['detected_close_black'].append({'area': area})

    # Detect side walls for steering
    wall_contours_by_roi = {job['type']: [] for job in [config.left_side_job, config.right_side_job, config.inner_left_side_job, config.inner_right_side_job]}
    contours, _ = cv2.findContours(final_mask_walls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) > config.WALL_MIN_AREA:
            M = cv2.moments(c)
            if M["m00"] == 0: continue
            cx = int(M["m10"] / M["m00"])

            # Determine which ROI the contour belongs to
            job_type = 'unknown'
            if config.left_side_job['roi'][0] <= cx < config.left_side_job['roi'][0] + config.left_side_job['roi'][2]: job_type = config.left_side_job['type']
            elif config.right_side_job['roi'][0] <= cx < config.right_side_job['roi'][0] + config.right_side_job['roi'][2]: job_type = config.right_side_job['type']
            elif config.inner_left_side_job['roi'][0] <= cx < config.inner_left_side_job['roi'][0] + config.inner_left_side_job['roi'][2]: job_type = config.inner_left_side_job['type']
            elif config.inner_right_side_job['roi'][0] <= cx < config.inner_right_side_job['roi'][0] + config.inner_right_side_job['roi'][2]: job_type = config.inner_right_side_job['type']

            if job_type != 'unknown':
                wall_contours_by_roi[job_type].append(c)

    # Find the biggest contour in each ROI and add it to the processed data
    for job_type, contour_list in wall_contours_by_roi.items():
        if contour_list:
            biggest_contour = max(contour_list, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            processed_data['detected_walls'].append({'type': job_type, 'area': area, 'contour': biggest_contour})
    
    return processed_data

def annotate_video_frame(frame, detections, debug_info=""):
    annotated_frame = frame.copy()
    light_blue = (255, 255, 0)

    # Define all ROIs for drawing
    all_rois = [
        config.left_side_job['roi'], config.right_side_job['roi'],
        config.inner_left_side_job['roi'], config.inner_right_side_job['roi'],
        (config.orange_roi_x, config.orange_roi_y, config.orange_roi_w, config.orange_roi_h),
        (config.close_x, config.close_y, config.close_w, config.close_h),
    ]
    # Draw ROIs on the frame
    for x, y, w, h in all_rois:
        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), light_blue, 2)

    # Draw detected walls
    for wall in detections['detected_walls']:
        cv2.drawContours(annotated_frame, [wall['contour']], -1, (255, 0, 0), 2)

    # Draw detected orange line
    if detections['detected_orange']:
        cv2.circle(annotated_frame, detections['detected_orange'][0]['centroid'], 5, (0, 165, 255), -1)

    # Add debug text to the frame
    cv2.putText(annotated_frame, str(debug_info), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    
    return annotated_frame


if __name__ == "__main__":
    camera.initialize()
    motor.initialize()
    servo.initialize()
    button = Button(23)
    
    profiler = cProfile.Profile()
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('recording.mp4', fourcc, 30, (config.FRAME_WIDTH, config.FRAME_HEIGHT))

    # Deque to track recent orange line detections for debouncing
    orange_detection_history = deque(maxlen=5)
    orange_detection_history.append(False)

    final_run_initiated = False
    final_run_start_time = None
    turn_counter = 0
    
    prev_angle = 0

    # Start camera and sensor threads
    sensors_initialized_event = threading.Event()
    camera_thread = CameraThread(camera)
    camera_thread.start()
    sensor_thread = SensorThread(bno055, distance, sensors_initialized_event)
    sensor_thread.start()
    
    print("MainThread: Waiting for sensors to initialize...")
    sensors_initialized_event.wait()
    print("MainThread: Sensors are ready. Proceeding with main logic.")    
    
    try:
        profiler.enable()
        run_start_time = time.monotonic()
        motor.forward(config.MOTOR_SPEED)

        while True:
            angle = 0
            debug = []
            frame = camera_thread.get_frame()
            if frame is None:
                continue
            
            # Process frame to find walls and orange lines
            detections = process_video_frame(frame)
            
            # --- Turn Counting Logic ---
            orange_detected_this_frame = bool(detections['detected_orange'])
            orange_detection_history.append(orange_detected_this_frame)
            
            # Check for a rising edge in detection: False -> True
            if len(orange_detection_history) >= 2:
                if orange_detection_history[-1] and not orange_detection_history[-2]:
                    turn_counter += 1
                    print(f"Orange line detected. Turn counter is now: {turn_counter}")

            # --- Steering Logic ---
            # Balance pixel areas of left vs. right walls
            left_pixel_size = sum(obj['area'] for obj in detections['detected_walls'] if 'left' in obj['type'])
            right_pixel_size = sum(obj['area'] for obj in detections['detected_walls'] if 'right' in obj['type'])

            # If one side wall disappears, turn sharply towards the other side
            if left_pixel_size < 100 and right_pixel_size > 100:
                right_pixel_size += 25000
            elif right_pixel_size < 100 and left_pixel_size > 100:
                left_pixel_size += 25000
            
            # Proportional steering based on the difference in wall area
            angle = ((left_pixel_size - right_pixel_size) * 0.0005) + 1
            
            # Override with sharp turn if a wall is detected directly in front
            close_black_area = sum(obj['area'] for obj in detections.get('detected_close_black', []))
            if close_black_area > 3000:
                # Decide turn direction based on which side has more "space" (fewer pixels)
                if left_pixel_size < right_pixel_size:
                    angle = -35  # Turn left
                else:
                    angle = 35   # Turn right
            
            # --- Angle Smoothing and Clamping ---
            # Limit the rate of change of the servo to prevent jerky movements
            angle = np.clip(angle, prev_angle - 10, prev_angle + 10)
            # Clamp the final angle to the servo's limits
            angle = np.clip(angle, -40, 40)
            servo.set_angle(angle)
            prev_angle = angle

            # --- Annotation and Video Recording ---
            debug.extend([f"L:{int(left_pixel_size)}", f"R:{int(right_pixel_size)}"])
            debug.append(f"Angle:{int(angle)}")
            debug.append(f"Turns:{turn_counter}")
            annotated_frame = annotate_video_frame(frame, detections, debug_info=str(debug))
            out.write(annotated_frame)
            
            # --- Exit Conditions ---
            if button.is_pressed:
                print("Button pressed. Stopping.")
                break
            
            if turn_counter >= 12 and not final_run_initiated:
                print("12 turns reached. Stopping in 1.5 seconds")
                final_run_initiated = True
                final_run_start_time = time.monotonic()

            # 2. If the final run has been initiated, check if the timer is up
            if final_run_initiated and (time.monotonic() - final_run_start_time >= 1.5):
                print("1.5 second complete. Stopping.")
                break

    finally:
        run_end_time = time.monotonic()
        run_time = run_end_time - run_start_time 
        print(f'Run completed in: {run_time:.2f} seconds.')
        # --- Cleanup ---
        profiler.disable()
        motor.brake()
        print("MainThread: Signaling threads to stop...")
        camera_thread.stop()
        sensor_thread.stop()

        print("MainThread: Waiting for threads to complete...")
        camera_thread.join()
        sensor_thread.join()
        print("MainThread: All threads have completed.")
        
        out.release()
        print("Stopping profiler and saving stats...")
        profiler.dump_stats("open_challenge.pstats")
        
        camera.cleanup()
        servo.set_angle(0)
        servo.cleanup()
        motor.cleanup()
        cv2.destroyAllWindows()
        print("Program finished.")
