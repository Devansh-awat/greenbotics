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
UPPER_BLACK = np.array([180, 255, 90])
LOWER_ORANGE = np.array([6, 70, 20])
UPPER_ORANGE = np.array([26, 255, 255])
LOWER_MAGENTA = np.array([158, 73, 64])
UPPER_MAGENTA = np.array([173, 255, 223])

detection_params = {'min_area': 300, 'return_rule': 'biggest_in_job', 'return_mask': True}
WALL_MIN_AREA = detection_params['min_area']
BLOCK_MIN_AREA = 500
MAGENTA_MIN_AREA = 500
CLOSE_BLOCK_MIN_AREA = 15

left_roi_x, left_roi_y, left_roi_w, left_roi_h = 0, 140, 135, 150
right_roi_x, right_roi_y, right_roi_w, right_roi_h = 505, 140, 135, 150
inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = 140, 190, 100, 100
inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = 400, 190, 100, 100
orange_roi_x, orange_roi_y, orange_roi_w, orange_roi_h = 280, 200, 80, 40
close_x,close_y,close_w,close_h = 140,120,360,10
full_frame_roi = (0, 80, 640, 185)
close_block_roi = (230, 230, 180, 10)

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
    def __init__(self, bno, dist):
        super().__init__()
        self.bno = bno
        self.dist = dist
        self.lock = threading.Lock()
        self.running = True
        self.daemon = True
        self.heading = None
        self.distance_left = None
        self.distance_right = None

    def run(self):
        while self.running:
            heading = self.bno.get_heading()
            dist_left = self.dist.get_distance(0)
            dist_center = self.dist.get_distance(2)
            dist_right = self.dist.get_distance(3)
            with self.lock:
                self.heading = heading
                self.distance_left = dist_left
                self.distance_center = dist_center
                self.distance_right = dist_right
            time.sleep(0.02) 

    def get_readings(self):
        with self.lock:
            return {
                'heading': self.heading,
                'distance_left': self.distance_left,
                'distance_center': self.distance_center,
                'distance_right': self.distance_right
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
        'close_red': final_mask_close_red, 'close_green': final_mask_close_green
    }
    block_types = {
        'main_red': ('block', 'red', BLOCK_MIN_AREA),
        'main_green': ('block', 'green', BLOCK_MIN_AREA),
        'close_red': ('close_block', 'red', CLOSE_BLOCK_MIN_AREA),
        'close_green': ('close_block', 'green', CLOSE_BLOCK_MIN_AREA)
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
        draw_color = (0, 0, 255) if block['color'] == 'red' else (0, 255, 0)
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

    MANEUVER_SPEED = 45
    SERVO_TURN_ANGLE = 40.0
    SCAN_TRIGGER_ANGLE_DEG = 30.0
    TOTAL_TURN_ANGLE_DEG = 85.0
    HEADING_LOCK_TOLERANCE = 5.0

    DRIVE_FORWARD_DURATION = 1
    DRIVE_FORWARD_SHORT_DURATION = 0
    REVERSE_DURATION = 0
    
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
    print(f"Initial Heading: {INITIAL_HEADING:.1f}Â°")
    print(f"Scan Will Trigger At: {scan_heading:.1f}Â°")
    print(f"Full Turn Target: {ninety_degree_heading:.1f}Â°")

    motor.forward(MANEUVER_SPEED)
    servo.set_angle_unlimited(initial_turn_servo)
    print("Starting initial turn...")

    while get_angular_difference(INITIAL_HEADING, sensor_thread.get_readings()['heading']) < SCAN_TRIGGER_ANGLE_DEG:
        time.sleep(0.02)

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
            if main_blocks[0]['area']>3000:
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
            action_taken = f"DRIVE_FORWARD_GREEN for {DRIVE_FORWARD_DURATION}s"
            drive_straight_with_gyro(drive_target_heading, DRIVE_FORWARD_DURATION, 70, 'forward')
        elif detected_block_color == 'red':
            action_taken = f"REVERSE_BEFORE_TURN for {REVERSE_DURATION}s"
            drive_straight_with_gyro(drive_target_heading, REVERSE_DURATION, 70, 'reverse')
        else:
            return
            action_taken = f"DRIVE_FORWARD_NONE for {DRIVE_FORWARD_SHORT_DURATION}s"
            drive_straight_with_gyro(drive_target_heading, DRIVE_FORWARD_SHORT_DURATION, 70, 'forward')
            
    else:
        if detected_block_color == 'green':
            action_taken = f"REVERSE_FOR_GREEN_CW for {REVERSE_DURATION}s"
            drive_straight_with_gyro(drive_target_heading, REVERSE_DURATION, 70, 'reverse')
        elif detected_block_color == 'red':
            
            action_taken = f"DRIVE_FORWARD_RED_CW for {DRIVE_FORWARD_DURATION}s"
            drive_straight_with_gyro(drive_target_heading, DRIVE_FORWARD_DURATION+0.5, 70, 'forward')
        else:
            return
            action_taken = f"DRIVE_FORWARD_NONE_CW for {DRIVE_FORWARD_SHORT_DURATION}s"
            drive_straight_with_gyro(drive_target_heading, DRIVE_FORWARD_SHORT_DURATION, 70, 'forward')

    motor.brake()
    print(f"Action complete: {action_taken}")
    time.sleep(0.5)

    print(f"Performing final turn to return to {INITIAL_HEADING:.1f}Â°...")
    motor.forward(70)
    servo.set_angle(final_turn_servo)

    while get_angular_difference(sensor_thread.get_readings()['heading'], INITIAL_HEADING) > 55:
        pass
    motor.brake()
    servo.set_angle(0)
    print("--- Initial Maneuver Complete. Transitioning to straight driving. ---")

def parking():
    motor.reverse(60)
    print(INITIAL_HEADING, sensor_thread.get_readings()['heading'])
    while get_angular_difference(INITIAL_HEADING, sensor_thread.get_readings()['heading']) > 5:
        print(INITIAL_HEADING, sensor_thread.get_readings()['heading'])
        sensor_readings = sensor_thread.get_readings()
        servo.set_angle(-steer_with_gyro(sensor_readings['heading'],INITIAL_HEADING, kp=3))
        time.sleep(0.01)
    motor.brake()
    motor.forward(65)
    print(sensor_thread.get_readings()['distance_center'])
    if sensor_thread.get_readings()['distance_center'] < 500:
        while sensor_thread.get_readings()['distance_center'] > 500:
            if driving_direction == 'clockwise':
                servo.set_angle((500-sensor_thread.get_readings()['distance_left'])*0.05)
                print(sensor_thread.get_readings()['distance_left'])
            else:
                servo.set_angle((sensor_thread.get_readings()['distance_right']-500)*0.05)
                print(sensor_thread.get_readings()['distance_right'])
            time.sleep(0.01)
    else:
        while sensor_thread.get_readings()['distance_center'] > 500:
            print(INITIAL_HEADING, sensor_thread.get_readings()['heading'])
            sensor_readings = sensor_thread.get_readings()
            servo.set_angle(steer_with_gyro(sensor_readings['heading'],INITIAL_HEADING, kp=3))
            time.sleep(0.01)
    if driving_direction == 'clockwise':
        servo.set_angle(-45)
    else:
        servo.set_angle(45)
    time.sleep(0.5)
    while get_angular_difference((INITIAL_HEADING+180)%360, sensor_thread.get_readings()['heading']) > 5:
        print(INITIAL_HEADING, sensor_thread.get_readings()['heading'])
        sensor_readings = sensor_thread.get_readings()
        servo.set_angle(steer_with_gyro(sensor_readings['heading'],(INITIAL_HEADING+180)%360, kp=1))
        #time.sleep(0.01)

if __name__ == "__main__":
    camera.initialize()
    motor.initialize()
    servo.initialize()
    distance.initialise()
    bno055.initialize()
    button = Button(23)
    
    profiler = cProfile.Profile()
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('recording.mp4', fourcc, 30, (640, 360))

    orange_detection_history = deque(maxlen=75)
    orange_detection_history.append(False)
    turn_counter = 0
    angle = 0
    prevangle = 0

    camera_thread = CameraThread(camera)
    camera_thread.start()
    sensor_thread = SensorThread(bno055, distance)
    sensor_thread.start()
    
    time.sleep(1)

    dist_left = distance.get_distance(0)
    dist_right = distance.get_distance(3)
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
       
    INITIAL_HEADING = bno055.get_heading()

    try:
        profiler.enable()
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
                        if block['color'] == 'red':
                            angle = -25
                        elif block['color'] == 'green':
                            angle = 30
                        servo.set_angle(angle)
                        motor.reverse(60)
                        time.sleep(0.6)
                        motor.forward(60)
                        servo.set_angle(-angle)
                        time.sleep(0.4)
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
                        if detections['detected_magenta'] and driving_direction == 'counter-clockwise' and not 240<detections['detected_magenta'][0]['target_x']<400:
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
                        if detections['detected_magenta'] and driving_direction == 'clockwise' and not 240<detections['detected_magenta'][0]['target_x']<400 and abs(detections['detected_magenta'][0]['target_y']-block_y)<70:
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
                if left_pixel_size<100 and right_pixel_size>100:
                    right_pixel_size += 25000
                elif right_pixel_size<100 and left_pixel_size>100:
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
            print(sensor_readings['distance_center'])
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
                parking()
                motor.brake()
                break

    finally:
        profiler.disable()
        camera_thread.stop()
        sensor_thread.stop()
        motor.brake()
        out.release()
        print("Stopping profiler and saving stats...")
        profiler.dump_stats("loop_performance.pstats")
        camera.cleanup()
        servo.set_angle(0)
        servo.cleanup()
        motor.cleanup()
        distance.cleanup()
        cv2.destroyAllWindows()
