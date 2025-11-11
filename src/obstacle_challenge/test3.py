from collections import deque
import time
from src.obstacle_challenge.config import LOWER_RED_1, UPPER_RED_1, LOWER_RED_2, UPPER_RED_2, LOWER_GREEN, UPPER_GREEN
from src.obstacle_challenge.main import get_angular_difference
from src.sensors import bno055, camera, distance
from src.motors import motor, servo
import numpy as np
import cv2
import cProfile
import threading
from gpiozero import Button

MOTOR_SPEED = 90
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
FRAME_MIDPOINT_X = FRAME_WIDTH // 2

LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 255, 110])
LOWER_ORANGE = np.array([6, 70, 20])
UPPER_ORANGE = np.array([26, 255, 255])
LOWER_MAGENTA = np.array([158, 73, 64])
UPPER_MAGENTA = np.array([173, 255, 223])
target=0
detection_params = {'min_area': 300, 'return_rule': 'biggest_in_job', 'return_mask': True}
WALL_MIN_AREA = detection_params['min_area']
BLOCK_MIN_AREA = 500
MAGENTA_MIN_AREA = 500
CLOSE_BLOCK_MIN_AREA = 15

left_roi_x, left_roi_y, left_roi_w, left_roi_h = 0, 140, 135, 150
right_roi_x, right_roi_y, right_roi_w, right_roi_h = 505, 140, 135, 150
inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = 140, 165, 100, 100
inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = 400, 165 , 100, 100
orange_roi_x, orange_roi_y, orange_roi_w, orange_roi_h = 280, 200, 80, 40
close_x,close_y,close_w,close_h = 140,120,360,10
full_frame_roi = (0, 100, 640, 165)
close_block_roi = (250, 230, 140, 10)

left_side_job = {'roi': (left_roi_x, left_roi_y, left_roi_w, left_roi_h), 'type': 'wall_left'}
right_side_job = {'roi': (right_roi_x, right_roi_y, right_roi_w, right_roi_h), 'type': 'wall_right'}
inner_left_side_job = {'roi': (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h), 'type': 'wall_inner_left'}
inner_right_side_job = {'roi': (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h), 'type': 'wall_inner_right'}

roi_mask_walls = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
for job in [left_side_job, right_side_job, inner_left_side_job, inner_right_side_job]:
    x, y, w, h = job['roi']
    cv2.rectangle(roi_mask_walls, (x, y), (x + w, y + h), 255, -1)

roi_mask_main_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
x, y, w, h = full_frame_roi
cv2.rectangle(roi_mask_main_blocks, (x, y), (x + w, y + h), 255, -1)

roi_mask_close_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
x, y, w, h = close_block_roi
cv2.rectangle(roi_mask_close_blocks, (x, y), (x + w, y + h), 255, -1)

roi_mask_orange = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
cv2.rectangle(roi_mask_orange, (orange_roi_x, orange_roi_y), (orange_roi_x + orange_roi_w, orange_roi_h + orange_roi_y), 255, -1)

roi_mask_magenta = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")

roi_mask_close_black = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
cv2.rectangle(roi_mask_close_black, (close_x, close_y), (close_x + close_w, close_y + close_h), 255, -1)

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
        'detected_blocks': [],
        'detected_walls': [],
        'detected_orange': [],
        'detected_magenta': [],
        'detected_close_black': []
    }
    
    frame = cv2.GaussianBlur(frame,(1,7),0)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    mask_black = cv2.inRange(hsv_frame, LOWER_BLACK, UPPER_BLACK)
    mask_red1 = cv2.inRange(hsv_frame, LOWER_RED_1, UPPER_RED_1)
    mask_red2 = cv2.inRange(hsv_frame, LOWER_RED_2, UPPER_RED_2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)
    mask_green = cv2.inRange(hsv_frame, LOWER_GREEN, UPPER_GREEN)
    mask_orange = cv2.inRange(hsv_frame, LOWER_ORANGE, UPPER_ORANGE)
    mask_magenta = cv2.inRange(hsv_frame, LOWER_MAGENTA, UPPER_MAGENTA)

    mask_red_or_green = cv2.bitwise_or(mask_red, mask_green)
    pure_black_mask = cv2.bitwise_and(mask_black, cv2.bitwise_not(mask_red_or_green))

    final_mask_walls = cv2.bitwise_and(pure_black_mask, roi_mask_walls)
    final_mask_main_red = cv2.bitwise_and(mask_red, roi_mask_main_blocks)
    final_mask_main_green = cv2.bitwise_and(mask_green, roi_mask_main_blocks)
    final_mask_close_red = cv2.bitwise_and(mask_red, roi_mask_close_blocks)
    final_mask_close_green = cv2.bitwise_and(mask_green, roi_mask_close_blocks)
    final_mask_orange = cv2.bitwise_and(mask_orange, roi_mask_orange)
    final_mask_magenta = cv2.bitwise_and(mask_magenta, roi_mask_magenta)
    final_mask_close_black = cv2.bitwise_and(pure_black_mask, roi_mask_close_black)
    final_mask_close_magenta = cv2.bitwise_and(mask_magenta, roi_mask_close_blocks)

    contours, _ = cv2.findContours(final_mask_magenta, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        biggest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest_contour)

        if area > MAGENTA_MIN_AREA:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                leftmost_x = biggest_contour[:, 0, 0].min()
                rightmost_x = biggest_contour[:, 0, 0].max()

                dist_to_center_left = abs(leftmost_x - FRAME_MIDPOINT_X)
                dist_to_center_right = abs(rightmost_x - FRAME_MIDPOINT_X)

                if dist_to_center_left <= dist_to_center_right:
                    target_x = leftmost_x
                else:
                    target_x = rightmost_x
                
                processed_data['detected_magenta'].append({
                    'type': 'magenta_block',
                    'area': area,
                    'centroid': (cx, cy),
                    'contour': biggest_contour,
                    'target_x': target_x,
                    'target_y': cy
                })

    all_detected_blocks = []
    block_masks = {
        'main_red': final_mask_main_red, 'main_green': final_mask_main_green,
        'close_red': final_mask_close_red, 'close_green': final_mask_close_green,
        'close_magenta': final_mask_close_magenta
    }
    block_types = {
        'main_red': ('block', 'red', BLOCK_MIN_AREA),
        'main_green': ('block', 'green', BLOCK_MIN_AREA),
        'close_red': ('close_block', 'red', CLOSE_BLOCK_MIN_AREA),
        'close_green': ('close_block', 'green', CLOSE_BLOCK_MIN_AREA),
        'close_magenta': ('close_block', 'magenta', CLOSE_BLOCK_MIN_AREA)
    }
    for name, mask in block_masks.items():
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            b_type, b_color, min_area = block_types[name]
            if area > min_area:
                M = cv2.moments(biggest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    all_detected_blocks.append({'type': b_type, 'color': b_color, 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour})

    main_blocks = [b for b in all_detected_blocks if b['type'] == 'block']
    other_blocks = [b for b in all_detected_blocks if b['type'] != 'block']
    if len(main_blocks) > 1:
        lowest_main_block = max(main_blocks, key=lambda b: b['centroid'][1])
        main_blocks = [lowest_main_block]
    processed_data['detected_blocks'] = main_blocks + other_blocks
    
    contours, _ = cv2.findContours(final_mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        biggest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest_contour)
        if area > 20:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                processed_data['detected_orange'].append({'type': 'orange_block', 'color': 'orange', 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour})

    contours, _ = cv2.findContours(final_mask_close_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        area = cv2.contourArea(contour)
        if area > WALL_MIN_AREA: # Check each contour's area
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                processed_data['detected_close_black'].append({
                    'type': 'close_black', 
                    'color': 'black', 
                    'area': area, 
                    'centroid': (cx, cy), 
                    'contour': contour # Add the current contour to the list
                })

    wall_contours_by_roi = {job['type']: [] for job in [left_side_job, right_side_job, inner_left_side_job, inner_right_side_job]}
    contours, _ = cv2.findContours(final_mask_walls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        if cv2.contourArea(c) > WALL_MIN_AREA:
            M = cv2.moments(c)
            if M["m00"] == 0: continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            job_type = 'unknown'
            if left_side_job['roi'][0] <= cx < left_side_job['roi'][0] + left_side_job['roi'][2]: job_type = left_side_job['type']
            elif right_side_job['roi'][0] <= cx < right_side_job['roi'][0] + right_side_job['roi'][2]: job_type = right_side_job['type']
            elif inner_left_side_job['roi'][0] <= cx < inner_left_side_job['roi'][0] + inner_left_side_job['roi'][2]: job_type = inner_left_side_job['type']
            elif inner_right_side_job['roi'][0] <= cx < inner_right_side_job['roi'][0] + inner_right_side_job['roi'][2]: job_type = inner_right_side_job['type']

            if job_type != 'unknown':
                wall_contours_by_roi[job_type].append(c)

    for job_type, contour_list in wall_contours_by_roi.items():
        if contour_list:
            biggest_contour = max(contour_list, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                processed_data['detected_walls'].append({'type': job_type, 'color': 'black', 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour})
    
    return processed_data

def annotate_video_frame(frame, detections, driving_direction, debug_info="", block_target=None):
    annotated_frame = frame.copy()
    light_blue = (255, 255, 0)
    target_line_color = (255, 0, 255)
    midpoint_color = (0, 255, 255)
    magenta_target_color = (255, 255, 255)

    all_rois = [
        (left_roi_x, left_roi_y, left_roi_w, left_roi_h),
        (right_roi_x, right_roi_y, right_roi_w, right_roi_h),
        (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h),
        (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h),
        (orange_roi_x, orange_roi_y, orange_roi_w, orange_roi_h),
        (close_x, close_y, close_w, close_h),
        full_frame_roi,
        close_block_roi
    ]
    for x, y, w, h in all_rois:
        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), light_blue, 2)

    for wall in detections['detected_walls']:
        cv2.drawContours(annotated_frame, [wall['contour']], -1, (255, 0, 0), 2)

    for block in detections['detected_blocks']:
        draw_color = (255, 255, 255)
        if block['color'] == 'red':
            draw_color = (0, 0, 255)
        elif block['color'] == 'green':
            draw_color = (0, 255, 0)
        elif block['color'] == 'magenta':
            draw_color = (255, 0, 255)
        cv2.drawContours(annotated_frame, [block['contour']], -1, draw_color, 2)

    for orange_obj in detections['detected_orange']:
        cv2.drawContours(annotated_frame, [orange_obj['contour']], -1, (0, 165, 255), 2)

    for black_obj in detections.get('detected_close_black', []):
        cv2.drawContours(annotated_frame, [black_obj['contour']], -1, (255, 0, 0), 2)

    for magenta_obj in detections['detected_magenta']:
        cv2.drawContours(annotated_frame, [magenta_obj['contour']], -1, (255, 0, 255), 2)
        target_x = magenta_obj['target_x']
        cy = magenta_obj['centroid'][1]
        cv2.circle(annotated_frame, (target_x, cy), 7, (255, 255, 255), -1)

    main_blocks = [b for b in detections['detected_blocks'] if b['type'] == 'block']
    
    magenta_is_valid_target = False
    if main_blocks and detections['detected_magenta']:
        block_color = main_blocks[0]['color']
        magenta_target_x = detections['detected_magenta'][0]['target_x']
        
        use_magenta_for_red = (block_color == 'red' and driving_direction == 'counter-clockwise')
        use_magenta_for_green = (block_color == 'green' and driving_direction == 'clockwise')
        is_in_deadzone = 240 < magenta_target_x < 400

        if (use_magenta_for_red or use_magenta_for_green) and not is_in_deadzone:
            magenta_is_valid_target = True
    
    if magenta_is_valid_target:
        block_x = main_blocks[0]['centroid'][0]
        magenta_obj = detections['detected_magenta'][0]
        target_x = magenta_obj['target_x']
        midpoint_x = (block_x + target_x) // 2
        
        cv2.line(annotated_frame, (midpoint_x, 0), (midpoint_x, FRAME_HEIGHT), midpoint_color, 2)
        cv2.putText(annotated_frame, "Goal", (midpoint_x + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, midpoint_color, 2)

        cy = magenta_obj['centroid'][1]
        cv2.circle(annotated_frame, (target_x, cy), 7, magenta_target_color, -1)

    elif main_blocks:
        block = main_blocks[0]
        block_color = block['color']
        block_x, block_y = block['centroid']
        original_target_x = 0        
        if block_target is not None:
            if block_color == 'red':
                original_target_x = 320-block_target
            else:
                original_target_x = 320+block_target
        if original_target_x > 0:
            cv2.line(annotated_frame, (original_target_x, 0), (original_target_x, FRAME_HEIGHT), target_line_color, 2)

    cv2.putText(annotated_frame, str(debug_info), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    
    return annotated_frame

def drive_straight_with_gyro(target_heading, duration, speed, direction='forward'):
    print(f"Driving {direction} with gyro stabilization for {duration}s...")
    
    KP = 0.85

    start_time = time.monotonic()
    
    if direction == 'forward':
        motor.forward(speed)
    else:
        motor.reverse(speed)

    while time.monotonic() - start_time < duration:
        current_heading = sensor_thread.get_readings()['heading']
        if current_heading is None:
            time.sleep(0.01)
            continue

        error = target_heading - current_heading
        
        while error <= -180: error += 360
        while error > 180: error -= 360

        steer_angle = KP * error
        
        steer_angle = np.clip(steer_angle, -45, 45)

        servo.set_angle(steer_angle)
        time.sleep(0.01)

    motor.brake()
    servo.set_angle(0)
    print("Gyro-stabilized drive complete.")

def steer_with_gyro(current_heading: float, 
                    target_heading: float, 
                    kp: float = 0.85, 
                    min_servo_angle: int = -45, 
                    max_servo_angle: int = 45) -> float:
    """
    Calculates the servo steering angle to maintain a target heading using a gyro.

    This function uses a Proportional (P) controller to correct the robot's
    trajectory. It correctly handles the "wrap-around" issue where headings
    jump between 359 and 0 degrees.

    Args:
        current_heading (float): The current direction of the robot in degrees (0-359).
        target_heading (float): The desired direction in degrees (0-359).
        kp (float): The proportional gain constant. This critical value determines
                    how aggressively the robot corrects its course.
                    - Higher value = more aggressive, sharper turns (can lead to oscillation).
                    - Lower value = softer, smoother turns (can be slow to correct).
        min_servo_angle (int): The minimum allowable angle for the servo (e.g., full left).
        max_servo_angle (int): The maximum allowable angle for the servo (e.g., full right).

    Returns:
        float: The calculated servo angle, clamped within the min/max limits.
    """
    # 1. Calculate the error between target and current heading.
    error = target_heading - current_heading

    # 2. Handle the wrap-around issue (e.g., turning from 350 degrees to 10 degrees).
    # The shortest path is not -340 degrees, but +20 degrees.
    if error > 180:
        error -= 360
    elif error < -180:
        error += 360

    # 3. Calculate the steering correction.
    # This is the "Proportional" part of the controller. The steering angle is
    # directly proportional to the error, scaled by the gain 'kp'.
    steer_angle = kp * error

    # 4. Clamp the steering angle to the servo's physical limits.
    # This prevents the code from trying to turn the servo beyond its capabilities.
    clamped_steer_angle = np.clip(steer_angle, min_servo_angle, max_servo_angle)

    return clamped_steer_angle

def perform_initial_maneuver():
    print("--- Executing Full Initial Maneuver ---")

    MANEUVER_SPEED = 43
    SERVO_TURN_ANGLE = 40.0
    SCAN_TRIGGER_ANGLE_DEG = 10.0
    TOTAL_TURN_ANGLE_DEG = 85.0
    HEADING_LOCK_TOLERANCE = 5.0

    DRIVE_FORWARD_DURATION = 1.4
    DRIVE_FORWARD_SHORT_DURATION = 0
    REVERSE_DURATION = 0.4
    
    if driving_direction == "clockwise":
        initial_turn_servo = SERVO_TURN_ANGLE+15
        final_turn_servo = -SERVO_TURN_ANGLE
        direction_modifier = 1
    else:
        initial_turn_servo = -SERVO_TURN_ANGLE
        final_turn_servo = SERVO_TURN_ANGLE
        direction_modifier = -1

    def calculate_target_heading(base, offset):
        return (base + offset + 360) % 360

    scan_heading = calculate_target_heading(INITIAL_HEADING, SCAN_TRIGGER_ANGLE_DEG * direction_modifier)
    ninety_degree_heading = calculate_target_heading(INITIAL_HEADING, TOTAL_TURN_ANGLE_DEG * direction_modifier)
    print(f"Direction: {driving_direction.upper()}")
    print(f"Initial Heading: {INITIAL_HEADING:.1f}°")
    print(f"Scan Will Trigger At: {scan_heading:.1f}°")
    print(f"Full Turn Target: {ninety_degree_heading:.1f}°")

    motor.forward(MANEUVER_SPEED)
    servo.set_angle_unlimited(initial_turn_servo)
    print("Starting initial turn...")

    while get_angular_difference(INITIAL_HEADING, sensor_thread.get_readings()['heading']) < SCAN_TRIGGER_ANGLE_DEG:
        print(sensor_thread.get_readings()['heading'])
    motor.brake()
    print(f"Scan angle reached. Pausing to scan for objects...")
    detected_block_color = None
    scan_start_time = time.monotonic()

    while time.monotonic() - scan_start_time < 1.0:
        frame = camera_thread.get_frame()
        if frame is None: continue

        detections = process_video_frame(frame)
        annotated_frame = annotate_video_frame(frame, detections, driving_direction)
        out.write(annotated_frame)
        main_blocks = [b for b in detections.get('detected_blocks', []) if b['type'] == 'block']
        
        if main_blocks:
            print(main_blocks[0]['area'])
            if main_blocks[0]['area']>1000:
                detected_block_color = main_blocks[0]['color']
                print(f"Block Found! Color: {detected_block_color.upper()}. Ending scan.")
                break
    
    if detected_block_color is None:
        print("Scan complete. No block was detected.")

    servo.set_angle(0)
    print("90-degree turn complete. Performing action based on scan...")
    time.sleep(0.5)

    action_taken = "None"
    drive_target_heading = ninety_degree_heading

    if driving_direction == 'counter-clockwise':
        if detected_block_color == 'green':
            motor.forward(60)
            while get_angular_difference(ninety_degree_heading, sensor_thread.get_readings()['heading']) > 5:
                servo.set_angle(steer_with_gyro(sensor_thread.get_readings()['heading'],ninety_degree_heading))
            action_taken = f"DRIVE_FORWARD_GREEN for {1}s"
            drive_straight_with_gyro(drive_target_heading, 1, 70, 'forward')
        elif detected_block_color == 'red':
            motor.forward(60)
            while get_angular_difference(ninety_degree_heading, sensor_thread.get_readings()['heading']) > 5:
                servo.set_angle(steer_with_gyro(sensor_thread.get_readings()['heading'],ninety_degree_heading))
            action_taken = f"REVERSE_BEFORE_TURN for {REVERSE_DURATION}s"
            drive_straight_with_gyro(drive_target_heading, REVERSE_DURATION, 70, 'reverse')
        else:
            return
            action_taken = f"DRIVE_FORWARD_NONE for {DRIVE_FORWARD_SHORT_DURATION}s"
            drive_straight_with_gyro(drive_target_heading, DRIVE_FORWARD_SHORT_DURATION, 70, 'forward')
            
    else:
        if detected_block_color == 'green':
            motor.forward(60)
            while get_angular_difference(ninety_degree_heading, sensor_thread.get_readings()['heading']) > 5:
                servo.set_angle(steer_with_gyro(sensor_thread.get_readings()['heading'],ninety_degree_heading))
            action_taken = f"REVERSE_FOR_GREEN_CW for {REVERSE_DURATION}s"
            drive_straight_with_gyro(drive_target_heading, REVERSE_DURATION, 70, 'reverse')
        elif detected_block_color == 'red':
            motor.forward(60)
            while get_angular_difference(ninety_degree_heading, sensor_thread.get_readings()['heading']) > 5:
                servo.set_angle(steer_with_gyro(sensor_thread.get_readings()['heading'],ninety_degree_heading))
            action_taken = f"DRIVE_FORWARD_RED_CW for {1}s"
            drive_straight_with_gyro(drive_target_heading, 1, 70, 'forward')
        else:
            return
            action_taken = f"DRIVE_FORWARD_NONE_CW for {DRIVE_FORWARD_SHORT_DURATION}s"
            drive_straight_with_gyro(drive_target_heading, DRIVE_FORWARD_SHORT_DURATION, 70, 'forward')

    motor.brake()
    print(f"Action complete: {action_taken}")
    time.sleep(0.5)

    print(f"Performing final turn to return to {INITIAL_HEADING:.1f}°...")
    motor.forward(70)
    #servo.set_angle(final_turn_servo)

    while get_angular_difference(sensor_thread.get_readings()['heading'], INITIAL_HEADING) > 15:
        servo.set_angle(steer_with_gyro(sensor_thread.get_readings()['heading'],INITIAL_HEADING,2))
    motor.brake()
    servo.set_angle(0)
    motor.reverse(65)
    start_time = time.monotonic()
    while time.monotonic() - start_time < 0.5:
        servo.set_angle(-steer_with_gyro(sensor_thread.get_readings()['heading'],INITIAL_HEADING,1))
    time.sleep(0.5)
    motor.brake()
    print("--- Initial Maneuver Complete. Transitioning to straight driving. ---")

def parking():
    # motor.reverse(60)
    # print(INITIAL_HEADING, sensor_thread.get_readings()['heading'])
    # while get_angular_difference(INITIAL_HEADING, sensor_thread.get_readings()['heading']) > 5:
    #     #print(INITIAL_HEADING, sensor_thread.get_readings()['heading'])
    #     sensor_readings = sensor_thread.get_readings()
    #     servo.set_angle(-steer_with_gyro(sensor_readings['heading'],INITIAL_HEADING, kp=3))
    #     time.sleep(0.01)
    # motor.brake()
    motor.forward(65)
    print('forward')
    print(sensor_thread.get_readings()['distance_center'])
    while True:
        sensor_readings = sensor_thread.get_readings()
        distance_center = sensor_readings.get('distance_center')
        if distance_center is not None and distance_center <= 150:
            print(f"Distance is {distance_center}. Exiting loop.")
            break
        print(distance_center)
        servo.set_angle(steer_with_gyro(sensor_readings['heading'], (INITIAL_HEADING + 15) % 360, kp=1))
        time.sleep(0.01)
    motor.reverse(50)
    #return
    #print(sensor_thread.get_readings()['distance_back'], sensor_readings['heading'])
    while get_angular_difference((INITIAL_HEADING+90)%360, sensor_thread.get_readings()['heading']) > 5 or sensor_thread.get_readings()['distance_back'] is not None and sensor_thread.get_readings()['distance_back'] > 230:
        #print(sensor_thread.get_readings()['distance_back'], sensor_readings['heading'])
        sensor_readings = sensor_thread.get_readings()
        servo.set_angle_unlimited(-steer_with_gyro(sensor_readings['heading'],(INITIAL_HEADING+90)%360, kp=1, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)
    #return
    motor.forward(55) 
    while get_angular_difference((INITIAL_HEADING+170)%360, sensor_thread.get_readings()['heading']) > 5:
            #print(INITIAL_HEADING, sensor_thread.get_readings()['heading'])
            sensor_readings = sensor_thread.get_readings()
            servo.set_angle(steer_with_gyro(sensor_readings['heading'],(INITIAL_HEADING+170)%360, kp=1,min_servo_angle=-40, max_servo_angle=40))    
    motor.brake()
    servo.set_angle(0)
    time.sleep(1)
    first_magenta_line_passed = False
    # This helper flag tracks if we are currently over the first line.
    on_first_line = False

    # Thresholds
    MAGENTA_HIGH_THRESHOLD = 500 # The count needed to detect a line
    MAGENTA_LOW_THRESHOLD = 200
    motor.forward(55) 
    while True:
        frame = camera_thread.get_frame()
        if frame is None:
            print("Failed to get frame, breaking loop.")
            break

        roi = frame[160:, 600:]
        mask = cv2.inRange(cv2.cvtColor(roi, cv2.COLOR_BGR2HSV), LOWER_BLACK, np.array([180, 255, 40]))
        roi[mask == 255] = (0, 0, 0)
        y_coords = np.argmax(mask, axis=0)
        valid_y_coords = y_coords[mask[y_coords, np.arange(roi.shape[1])] > 0]
        
        steering_value = 0.0
        if valid_y_coords.size > 0:
            average_y = np.mean(valid_y_coords)
            error = 25 - average_y
            steering_value = 0.8 * error
            cv2.line(frame, (600, 160 + int(average_y)), (frame.shape[1], 160 + int(average_y)), (0, 0, 255), 2)
            
        servo.set_angle(steering_value)
        
        cv2.line(frame, (600, 160 + 25), (frame.shape[1], 160 + 25), (0, 255, 0), 2)
        # --- 2. Magenta Pixel Detection (remains the same) ---
        roi_stop = frame[330:360, 426:640]
        hsv_stop = cv2.cvtColor(roi_stop, cv2.COLOR_BGR2HSV)
        mask_magenta = cv2.inRange(hsv_stop, LOWER_MAGENTA, UPPER_MAGENTA)
        magenta_pixel_count = cv2.countNonZero(mask_magenta)

        # --- 3. Visualization ---
        cv2.rectangle(frame, (600, 160), (frame.shape[1], frame.shape[0]), (0, 255, 0), 2)
        cv2.putText(frame, f"Servo angle: {steering_value}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        cv2.rectangle(frame, (426, 330), (640, 360), (255, 0, 255), 2)
        cv2.putText(frame, f"Magenta Pixels: {magenta_pixel_count}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        
        # Display the current state on the frame for easy debugging
        state_text = f"Armed to Stop: {first_magenta_line_passed}"
        cv2.putText(frame, state_text, (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        out.write(frame)

        # --- 4. New State-Based Stop Condition ---
        
        # If we are armed (already passed the first line), look for the second line to stop.
        if first_magenta_line_passed:
            if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                print("Second magenta line detected. Stopping.")
                break
        # Otherwise, we are still handling the logic for crossing the first line.
        else:
            # If we have detected the peak of the first line, wait for the count to drop.
            if on_first_line:
                if magenta_pixel_count < MAGENTA_LOW_THRESHOLD:
                    print("First magenta line fully crossed. Now armed to stop on the next one.")
                    first_magenta_line_passed = True
            # If we haven't seen the first line yet, look for its peak.
            else:
                if magenta_pixel_count > MAGENTA_HIGH_THRESHOLD:
                    print("Detected what seems to be the first magenta line.")
                    on_first_line = True
    servo.set_angle(1)
    time.sleep(0.7)
    motor.brake()
    time.sleep(2)
    motor.reverse(45)
    servo.set_angle_unlimited(55)
    while get_angular_difference((INITIAL_HEADING+100)%360, sensor_thread.get_readings()['heading']) > 10:
            pass
    motor.brake()
    time.sleep(2)
    motor.forward(45)
    servo.set_angle(0)
    print('reverse')
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        print(dist)
        if dist is not None and dist > 210:
            break
        
    motor.brake()
    time.sleep(2)
    motor.reverse(40)
    servo.set_angle_unlimited(-60)
    while True:
        dist = sensor_thread.get_readings()['distance_back']
        if dist is not None:
            if dist <= 65:
                break
        if get_angular_difference((INITIAL_HEADING+180)%360, sensor_thread.get_readings()['heading']) < 5:
            break
    motor.brake()
    time.sleep(2)
    motor.forward(40)
    while True:
        if sensor_thread.get_readings()['distance_center'] is not None and sensor_thread.get_readings()['distance_center'] < 80:
            break
        if get_angular_difference(sensor_thread.get_readings()['heading'], (INITIAL_HEADING+180)%360) < 5:
            break
        servo.set_angle(steer_with_gyro(sensor_thread.get_readings()['heading'],(INITIAL_HEADING+180)%360, kp=1.5))
        time.sleep(0.01)
    motor.brake()


def parking2():
    global INITIAL_HEADING
    motor.forward(65)
    print('forward')
    print(sensor_thread.get_readings()['distance_center'])
    while True:
        sensor_readings = sensor_thread.get_readings()
        distance_center = sensor_readings.get('distance_center')
        if distance_center is not None and distance_center <= 150:
            print(f"Distance is {distance_center}. Exiting loop.")
            break
        print(distance_center)
        servo.set_angle(steer_with_gyro(sensor_readings['heading'], (INITIAL_HEADING) % 360, kp=1))
        time.sleep(0.01)
    motor.reverse(50)
    #return
    #print(sensor_thread.get_readings()['distance_back'], sensor_readings['heading'])
    while get_angular_difference((INITIAL_HEADING-90)%360, sensor_thread.get_readings()['heading']) > 5 or sensor_thread.get_readings()['distance_back'] is not None and sensor_thread.get_readings()['distance_back'] > 180:
        #print(sensor_thread.get_readings()['distance_back'], sensor_readings['heading'])
        sensor_readings = sensor_thread.get_readings()
        servo.set_angle_unlimited(-steer_with_gyro(sensor_readings['heading'],(INITIAL_HEADING-90)%360, kp=1, min_servo_angle=-60, max_servo_angle=60))
        time.sleep(0.01)
    #return
    motor.forward(55) 
    while get_angular_difference((INITIAL_HEADING-180)%360, sensor_thread.get_readings()['heading']) > 5:
            #print(INITIAL_HEADING, sensor_thread.get_readings()['heading'])
            sensor_readings = sensor_thread.get_readings()
            servo.set_angle(steer_with_gyro(sensor_readings['heading'],(INITIAL_HEADING-180)%360, kp=1,min_servo_angle=-40, max_servo_angle=40))   
    while True:
        frame = camera_thread.get_frame()
        if frame is None:
            print("Failed to get frame, breaking loop.")
            break
        
        roi_black_line = frame[160:, :40]
        mask_black = cv2.inRange(cv2.cvtColor(roi_black_line, cv2.COLOR_BGR2HSV), LOWER_BLACK, np.array([180, 255, 40]))
        y_coords = np.argmax(mask_black, axis=0)
        valid_y_coords = y_coords[mask_black[y_coords, np.arange(roi_black_line.shape[1])] > 0]
        
        steering_value = 0.0
        if valid_y_coords.size > 0:
            average_y = np.mean(valid_y_coords)
            error = average_y - 15
            steering_value = 0.8 * error
            
        servo.set_angle(steering_value)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_orange = cv2.inRange(hsv_frame, LOWER_ORANGE, UPPER_ORANGE)
        final_mask_orange = cv2.bitwise_and(mask_orange, roi_mask_orange)

        contours, _ = cv2.findContours(final_mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(frame, [biggest_contour], -1, (0, 165, 255), 2)
            if cv2.contourArea(biggest_contour) > 50:
                print(">>> Orange parking line detected! Transitioning to final maneuver. <<<")
                motor.brake()
                
                print(f"Original INITIAL_HEADING was: {INITIAL_HEADING}")
                INITIAL_HEADING = (INITIAL_HEADING + 180) % 360
                print(f"New INITIAL_HEADING for parking() is: {INITIAL_HEADING}")
                
                parking()
                break
        
        out.write(frame)
    
if __name__ == "__main__":
    camera.initialize()
    motor.initialize()
    servo.initialize()
    button = Button(23)
    
    profiler = cProfile.Profile()
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('recording.mp4', fourcc, 30, (640, 360))

    orange_detection_history = deque(maxlen=75)
    front_sensor_values = deque(maxlen=20)
    orange_detection_history.append(False)
    turn_counter = 0
    angle = 0
    prevangle = 0

    sensors_initialized_event = threading.Event()
    camera_thread = CameraThread(camera)
    camera_thread.start()
    sensor_thread = SensorThread(bno055, distance, sensors_initialized_event)
    sensor_thread.start()
    
    print("MainThread: Waiting for sensors to initialize...")
    sensors_initialized_event.wait() 
    print("MainThread: Sensors are ready. Proceeding with main logic.")    
    time.sleep(1)
    dist_left = sensor_thread.get_readings()['distance_left']
    dist_right = sensor_thread.get_readings()['distance_right']
    print(dist_left, dist_right)
    driving_direction = "clockwise"
    if dist_left is not None and dist_right is not None:
        if dist_left < dist_right:
            driving_direction = "clockwise"
        else:
            driving_direction = "counter-clockwise"
    if driving_direction == "clockwise":
        cv2.rectangle(roi_mask_magenta, (0, full_frame_roi[1]), (320, full_frame_roi[1] + full_frame_roi[3]), 255, -1)
    else:
        cv2.rectangle(roi_mask_magenta, (320, full_frame_roi[1]), (640, full_frame_roi[1] + full_frame_roi[3]), 255, -1)
       
    INITIAL_HEADING = None
    while INITIAL_HEADING is None:
        print("MainThread: Waiting for first valid heading reading...")
        readings = sensor_thread.get_readings()
        if readings and readings['heading'] is not None:
            INITIAL_HEADING = readings['heading']
        time.sleep(0.05)
    print(f"MainThread: Initial heading locked: {INITIAL_HEADING}")

    try:
        profiler.enable()
        run_start_time = time.monotonic()
        perform_initial_maneuver()
        motor.forward(MOTOR_SPEED)

        while True:
            angle=0
            debug = []
            frame = camera_thread.get_frame()
            if frame is None:
                continue
            
            sensor_readings = sensor_thread.get_readings()

            detections = process_video_frame(frame)
            detected_blocks = detections['detected_blocks']
            detected_walls = detections['detected_walls']
            detected_orange_object = detections['detected_orange']

            orange_detected_this_frame = bool(detected_orange_object)
            orange_detection_history.append(orange_detected_this_frame)
            if len(orange_detection_history) >= 3:
                if orange_detection_history[-1] and orange_detection_history[-2] and not orange_detection_history[-3]:
                    if not any(list(orange_detection_history)[:-2]):
                        turn_counter += 1
                        print(f"Orange detected after a gap. Turn counter is now: {turn_counter}")

            if detected_blocks:
                is_close_block = False
                for block in detected_blocks:
                    if block['type'] == 'close_block':
                        is_close_block = True
                        if block['color'] == 'magenta' and (time.monotonic()-run_start_time)>20:
                            if driving_direction == 'clockwise':
                                angle = -25
                            else:
                                angle = 30
                        elif block['color'] == 'red':
                            angle = -25
                        elif block['color'] == 'green':
                            angle = 30
                        servo.set_angle(angle)
                        motor.reverse(60)
                        time.sleep(0.3)
                        motor.forward(60)
                        servo.set_angle(-angle)
                        time.sleep(0.2)
                        break
                
                if not is_close_block:
                    motor.forward(MOTOR_SPEED)
                    block = detected_blocks[0]
                    block_color = block['color']
                    block_x, block_y = block['centroid']
                    debug.append((block_x, block_y))
                    
                    if block_color == 'red':
                        wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                        target = 300 if block_y > 170 and 200 < block_x < 440 else 150
                        debug.append(target)
                        if detections['detected_magenta'] and driving_direction == 'counter-clockwise' and not 320<detections['detected_magenta'][0]['target_x']<640:
                            target_x = detections['detected_magenta'][0]['target_x']
                            midpoint_x = (block_x + target_x) // 2
                            angle = ((midpoint_x - FRAME_MIDPOINT_X) * 0.20) + 1
                        else:
                            angle = ((block_x - (320 - target)) * 0.09) + 1
                        if wall_inner_right_size > 3000: angle = np.clip(angle, -45, -10)
                        else: angle = np.clip(angle, -45, 35)
                    
                    elif block_color == 'green':
                        wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                        target = 200 if block_y > 180 and 240 < block_x < 400 else 130
                        if detections['detected_magenta'] and driving_direction == 'clockwise' and not 0<detections['detected_magenta'][0]['target_x']<320 and abs(detections['detected_magenta'][0]['target_y']-block_y)<70:
                            target_x = detections['detected_magenta'][0]['target_x']
                            midpoint_x = (block_x + target_x) // 2
                            angle = ((midpoint_x - FRAME_MIDPOINT_X) * 0.20) + 1
                        else:
                            angle = ((block_x - (320 + target)) * 0.07) + 1
                        if wall_inner_left_size > 3000: angle = np.clip(angle, 15, 45)
                        else: angle = np.clip(angle, -45, 45 )
            else:
                left_pixel_size,right_pixel_size,wall_inner_left_size,wall_inner_right_size,left_distance,right_distance,target=0,0,0,0,0,0,0
                left_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_left')
                right_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_right')
                wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                if left_pixel_size<100 and (right_pixel_size + wall_inner_right_size)>100:
                    right_pixel_size += 25000
                elif right_pixel_size<100 and (left_pixel_size + wall_inner_left_size)>100:
                    left_pixel_size += 25000
                if left_distance is not None and left_distance > 1500 and driving_direction == 'counter-clockwise': right_pixel_size += 25000
                elif right_distance is not None and right_distance > 1500 and driving_direction == 'clockwise': left_pixel_size += 25000
                
                debug.extend([left_pixel_size, right_pixel_size])
                angle = (((left_pixel_size + wall_inner_left_size) - (right_pixel_size + wall_inner_right_size)) * 0.0005) + 1
                close_black_area = sum(obj['area'] for obj in detections.get('detected_close_black', []))
                if close_black_area > 3000:
                    if driving_direction == 'clockwise':
                        angle = 35
                    else:
                        angle = -35
            debug.append(round(angle))
            debug.append(turn_counter)
            
            annotated_frame = annotate_video_frame(frame, detections, driving_direction, debug_info=str(debug),block_target=target)
            
            out.write(annotated_frame)
            angle = np.clip(angle, prevangle-10, prevangle+10)
            angle = np.clip(angle,-40,40)
            servo.set_angle(angle)
            prevangle = angle
            angle = 0
            
            if False:
                cv2.imshow("Robot Live Feed", annotated_frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            if button.is_pressed:
                motor.brake()
                break
            if turn_counter >= 13:
                if driving_direction == 'clockwise':
                    parking()
                else:
                    parking2()
                run_end_time = time.time()
                run_time = run_end_time-run_start_time
                print(run_time)
                motor.brake()
                break

    finally:
        profiler.disable()
        motor.brake()
        print("MainThread: Signaling threads to stop...")
        camera_thread.stop()
        sensor_thread.stop()

        print("MainThread: Waiting for threads to complete...")
        camera_thread.join()
        sensor_thread.join()
        print("MainThread: All threads have completed.")
        motor.brake()
        out.release()
        print("Stopping profiler and saving stats...")
        profiler.dump_stats("loop_performance.pstats")
        camera.cleanup()
        servo.set_angle(0)
        servo.cleanup()
        motor.cleanup()
        cv2.destroyAllWindows()
