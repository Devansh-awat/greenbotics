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

# --- Constants and Configuration ---
MOTOR_SPEED = 85
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
FRAME_MIDPOINT_X = FRAME_WIDTH // 2

# --- Color Definitions ---
LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 255, 50])
LOWER_ORANGE = np.array([6, 70, 20])
UPPER_ORANGE = np.array([26, 255, 255])
LOWER_MAGENTA = np.array([158, 73, 64])
UPPER_MAGENTA = np.array([173, 255, 223])

# --- Detection Parameters ---
detection_params = {'min_area': 300, 'return_rule': 'biggest_in_job', 'return_mask': True}
WALL_MIN_AREA = detection_params['min_area']
BLOCK_MIN_AREA = 500
MAGENTA_MIN_AREA = 500
CLOSE_BLOCK_MIN_AREA = 1

# --- Region of Interest (ROI) Definitions ---
left_roi_x, left_roi_y, left_roi_w, left_roi_h = 0, 90, 100, 200
right_roi_x, right_roi_y, right_roi_w, right_roi_h = 540, 90, 100, 200
inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = 140, 150, 100, 100
inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = 400, 150, 100, 100
orange_roi_x, orange_roi_y, orange_roi_w, orange_roi_h = 280, 160, 80, 20
full_frame_roi = (0, 40, 640, 180)
close_block_roi = (300, 0, 1, 1)

# --- Detection Job Definitions ---
left_side_job = {'roi': (left_roi_x, left_roi_y, left_roi_w, left_roi_h), 'type': 'wall_left'}
right_side_job = {'roi': (right_roi_x, right_roi_y, right_roi_w, right_roi_h), 'type': 'wall_right'}
inner_left_side_job = {'roi': (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h), 'type': 'wall_inner_left'}
inner_right_side_job = {'roi': (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h), 'type': 'wall_inner_right'}

# --- Pre-computed ROI Masks ---
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


# --- Threading Classes ---
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
            dist_right = self.dist.get_distance(3)
            with self.lock:
                self.heading = heading
                self.distance_left = dist_left
                self.distance_right = dist_right
            time.sleep(0.02) # Read sensors at approx 50Hz

    def get_readings(self):
        with self.lock:
            return {
                'heading': self.heading,
                'distance_left': self.distance_left,
                'distance_right': self.distance_right
            }

    def stop(self):
        self.running = False


# --- Core Functions ---
def process_video_frame(frame):
    processed_data = {
        'detected_blocks': [],
        'detected_walls': [],
        'detected_orange': [],
        'detected_magenta': []
    }
    
    #frame = frame[426:946, 164:1124]
    frame = cv2.GaussianBlur(frame,(1,7),0)
    #frame = cv2.bilateralFilter(frame, d=9, sigmaColor=75, sigmaSpace=75)
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #h, s, v = cv2.split(hsv_frame)
    #s = np.clip(s*1.5,0,255).astype(np.uint8)
    #v = np.clip(v*1.5,0,255).astype(np.uint8)
    #hsv_frame = cv2.merge((h,s,v))

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

    # --- Magenta Detection ---
    contours, _ = cv2.findContours(mask_magenta, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        # Find the single biggest magenta contour in the frame
        biggest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest_contour)

        # Proceed only if the biggest contour is larger than the minimum area
        if area > MAGENTA_MIN_AREA:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                # Calculate centroid (center of the object)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # Find the x-coordinate of the leftmost and rightmost pixels of the contour
                leftmost_x = biggest_contour[:, 0, 0].min()
                rightmost_x = biggest_contour[:, 0, 0].max()

                # Calculate the distance of these points from the center of the frame
                dist_to_center_left = abs(leftmost_x - FRAME_MIDPOINT_X)
                dist_to_center_right = abs(rightmost_x - FRAME_MIDPOINT_X)

                # Determine which x-coordinate is closer to the center
                if dist_to_center_left <= dist_to_center_right:
                    target_x = leftmost_x
                else:
                    target_x = rightmost_x
                
                # Store all the calculated data in the processed_data dictionary
                processed_data['detected_magenta'].append({
                    'type': 'magenta_block',
                    'area': area,
                    'centroid': (cx, cy),
                    'contour': biggest_contour,
                    'target_x': target_x  # The special x-coordinate you calculated
                })

    # Block Detection
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
        biggest_main_block = max(main_blocks, key=lambda b: b['area'])
        main_blocks = [biggest_main_block]
    processed_data['detected_blocks'] = main_blocks + other_blocks
    
    # Orange Detection
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

    # Wall Detection
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

def annotate_video_frame(frame, detections, debug_info=""):
    """
    Annotates the video frame with ROIs, detected object contours, and the target x-coordinate.

    Args:
        frame: The original video frame to annotate.
        detections: A dictionary containing lists of detected walls, blocks, and orange objects.
        debug_info: A string containing debug information to display on the frame.

    Returns:
        The annotated video frame.
    """
    annotated_frame = frame.copy()
    light_blue = (255, 255, 0)  # BGR color for light blue
    target_line_color = (255, 0, 255) # BGR color for magenta
    midpoint_color = (0, 255, 255)      # Yellow for the midpoint goal
    magenta_target_color = (255, 255, 255) # White for the magenta edge target

    # --- Draw all ROIs in light blue ---
    all_rois = [
        (left_roi_x, left_roi_y, left_roi_w, left_roi_h),
        (right_roi_x, right_roi_y, right_roi_w, right_roi_h),
        (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h),
        (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h),
        (orange_roi_x, orange_roi_y, orange_roi_w, orange_roi_h),
        full_frame_roi,
        close_block_roi
    ]
    for x, y, w, h in all_rois:
        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), light_blue, 2)

    # --- Draw detected object contours ---
    for wall in detections['detected_walls']:
        cv2.drawContours(annotated_frame, [wall['contour']], -1, (255, 0, 0), 2)

    for block in detections['detected_blocks']:
        draw_color = (0, 0, 255) if block['color'] == 'red' else (0, 255, 0)
        cv2.drawContours(annotated_frame, [block['contour']], -1, draw_color, 2)

    for orange_obj in detections['detected_orange']:
        cv2.drawContours(annotated_frame, [orange_obj['contour']], -1, (0, 165, 255), 2)

    for magenta_obj in detections['detected_magenta']:
        # Draw the contour of the magenta object
        cv2.drawContours(annotated_frame, [magenta_obj['contour']], -1, (255, 0, 255), 2)

        # Draw a small circle at the calculated target_x to visualize it
        target_x = magenta_obj['target_x']
        cy = magenta_obj['centroid'][1] # Use the centroid's y for plotting
        cv2.circle(annotated_frame, (target_x, cy), 7, (255, 255, 255), -1) # White circle

    # --- Determine and draw visualization lines ---
    main_blocks = [b for b in detections['detected_blocks'] if b['type'] == 'block']

    # --- Case 1: Both magenta and a main block are detected ---
    if main_blocks and detections['detected_magenta']:
        # Get all the necessary coordinates
        block_x = main_blocks[0]['centroid'][0]
        magenta_obj = detections['detected_magenta'][0]
        target_x = magenta_obj['target_x']
        midpoint_x = (block_x + target_x) // 2

        # DRAW THE MIDPOINT (The robot's actual steering goal)
        # A vertical yellow line shows where the robot wants the midpoint to be
        cv2.line(annotated_frame, (midpoint_x, 0), (midpoint_x, FRAME_HEIGHT), midpoint_color, 2)
        cv2.putText(annotated_frame, "Goal", (midpoint_x + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, midpoint_color, 2)

        # DRAW THE MAGENTA TARGET_X (The edge point used in the calculation)
        # A white circle shows exactly which point on the magenta object is being tracked
        cy = magenta_obj['centroid'][1]
        cv2.circle(annotated_frame, (target_x, cy), 7, magenta_target_color, -1)


    # --- Case 2: Only a main block is detected (your original fallback logic) ---
    elif main_blocks:
        block = main_blocks[0]
        block_color = block['color']
        block_x, block_y = block['centroid']
        original_target_x = 0
        target = 0

        if block_color == 'red':
            target = 300 if block_y > 130 and 240 < block_x < 400 else 150
            original_target_x = 320 - target
        
        elif block_color == 'green':
            target = 200 if block_y > 140 and 240 < block_x < 400 else 100
            original_target_x = 320 + target

        if original_target_x > 0:
            cv2.line(annotated_frame, (original_target_x, 0), (original_target_x, FRAME_HEIGHT), target_line_color, 2)

    # --- Draw debug info ---
    cv2.putText(annotated_frame, str(debug_info), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
    
    return annotated_frame

# --- Main Execution ---
if __name__ == "__main__":
    # --- Initialization ---
    camera.initialize()
    motor.initialize()
    servo.initialize()
    distance.initialise()
    bno055.initialize()
    button = Button(23)
    
    profiler = cProfile.Profile()
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('recording.mp4', fourcc, 30, (640, 360))

    # --- State Variables ---
    orange_detection_history = deque(maxlen=30)
    turn_counter = 0
    angle = 0
    prevangle = 0

    # --- Start Threads ---
    camera_thread = CameraThread(camera)
    camera_thread.start()
    sensor_thread = SensorThread(bno055, distance)
    sensor_thread.start()
    
    time.sleep(1) # Allow threads to start and get initial readings

    # --- Initial Setup ---
    dist_left = distance.get_distance(0)
    dist_right = distance.get_distance(3)
    print(dist_left, dist_right)
    driving_direction = "clockwise"
    if dist_left is not None and dist_right is not None:
        if dist_left < dist_right:
            driving_direction = "clockwise"
        else:
            driving_direction = "counter-clockwise"
            
    INITIAL_HEADING = bno055.get_heading()

    try:
        profiler.enable()
        motor.forward(60)
        if True:
            if driving_direction == "clockwise":
                servo.set_angle_unlimited(45)
                print(INITIAL_HEADING, bno055.get_heading())
                while not(get_angular_difference(INITIAL_HEADING, bno055.get_heading())>= 30):
                    print(INITIAL_HEADING, bno055.get_heading())
                    time.sleep(0.05)
            else:
                servo.set_angle_unlimited(-45)
                print(INITIAL_HEADING, bno055.get_heading())
                while not(get_angular_difference(INITIAL_HEADING, bno055.get_heading())>= 30):
                    print(INITIAL_HEADING, bno055.get_heading())
                    time.sleep(0.05)
        motor.forward(MOTOR_SPEED)

        # --- Main Loop ---
        while True:
            debug = []
            frame = camera_thread.get_frame()
            if frame is None:
                continue
            
            sensor_readings = sensor_thread.get_readings()

            # Process the frame to get detections
            detections = process_video_frame(frame)
            detected_blocks = detections['detected_blocks']
            detected_walls = detections['detected_walls']
            detected_orange_object = detections['detected_orange']

            # Update orange detection history
            orange_detected_this_frame = bool(detected_orange_object)
            orange_detection_history.append(orange_detected_this_frame)
            if len(orange_detection_history) >= 3:
                if orange_detection_history[-1] and orange_detection_history[-2] and not orange_detection_history[-3]:
                    if not any(list(orange_detection_history)[:-2]):
                        turn_counter += 1
                        print(f"Orange detected after a gap. Turn counter is now: {turn_counter}")

            # --- Control Logic ---
            if detected_blocks:
                is_close_block = False
                for block in detected_blocks:
                    if block['type'] == 'close_block':
                        is_close_block = True
                        angle = -25 if block['color'] == 'red' else 20
                        servo.set_angle(angle)
                        motor.reverse(MOTOR_SPEED)
                        time.sleep(0.5)
                        break
                
                if not is_close_block:
                    motor.forward(MOTOR_SPEED)
                    block = detected_blocks[0]
                    block_color = block['color']
                    block_x, block_y = block['centroid']
                    debug.append((block_x, block_y))
                    
                    if block_color == 'red':
                        wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                        target = 300 if block_y > 130 and 240 < block_x < 400 else 170
                        debug.append(target)
                        if detections['detected_magenta'] and driving_direction == 'counter-clockwise':
                            target_x = detections['detected_magenta'][0]['target_x']
                            midpoint_x = (block_x + target_x) // 2
                            angle = ((midpoint_x - FRAME_MIDPOINT_X) * 0.20) + 8
                        else:
                            angle = ((block_x - (320 - target)) * 0.30) + 6
                        if wall_inner_right_size > 3000: angle = np.clip(angle, -45, -5)
                        else: angle = np.clip(angle, -45, 45)
                    
                    elif block_color == 'green':
                        wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                        target = 200 if block_y > 140 and 240 < block_x < 400 else 100
                        if detections['detected_magenta'] and driving_direction == 'clockwise':
                            target_x = detections['detected_magenta'][0]['target_x']
                            midpoint_x = (block_x + target_x) // 2
                            angle = ((midpoint_x - FRAME_MIDPOINT_X) * 0.20) + 8
                        else:
                            angle = ((block_x - (320 + target)) * 0.12) + 8
                        if wall_inner_left_size > 3000: angle = np.clip(angle, 15, 45)
                        else: angle = np.clip(angle, -45, 20)
            else:
                # Wall following logic
                left_pixel_size,right_pixel_size,wall_inner_left_size,wall_inner_right_size,left_distance,right_distance=0,0,0,0,0,0
                left_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_left')
                right_pixel_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_right')
                wall_inner_left_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_left')
                wall_inner_right_size = sum(obj['area'] for obj in detected_walls if obj['type'] == 'wall_inner_right')
                if abs((left_pixel_size + wall_inner_left_size) - (right_pixel_size + wall_inner_right_size)) < 2000:
                    #left_distance = sensor_readings['distance_left']
                    #right_distance = sensor_readings['distance_right']
                    print(left_distance, right_distance)
                if left_pixel_size<100:
                    right_pixel_size += 7000
                elif right_pixel_size<100:
                    left_pixel_size += 7000
                if left_distance is not None and left_distance > 900: right_pixel_size += 5000
                elif right_distance is not None and right_distance > 900: left_pixel_size += 5000
                
                debug.extend([left_pixel_size, right_pixel_size])
                angle = (((left_pixel_size + wall_inner_left_size) - (right_pixel_size + wall_inner_right_size)) * 0.002) + 5

            # --- Finalize and Actuate ---
            debug.append(round(angle))
            debug.append(turn_counter)
            
            annotated_frame = annotate_video_frame(frame, detections, debug_info=str(debug))
            
            out.write(annotated_frame)
            
            angle = np.clip(angle, prevangle - 10, prevangle + 10)
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

    finally:
        # --- Cleanup ---
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
