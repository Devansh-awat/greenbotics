from collections import deque
import time
from src.obstacle_challenge.config import LOWER_RED_1, UPPER_RED_1, LOWER_RED_2, UPPER_RED_2, LOWER_GREEN, UPPER_GREEN
from src.obstacle_challenge.main import get_angular_difference
from src.sensors import bno055, camera, vl53l1x
from src.motors import motor, servo
import numpy as np
import cv2
import cProfile
import threading
from gpiozero import Button
MOTOR_SPEED = 100
orange_detection_history = deque(maxlen=15)
turn_counter=0
profiler = cProfile.Profile()

fourcc = cv2.VideoWriter_fourcc(*'avc1')
out = cv2.VideoWriter('recording.mp4', fourcc, 20, (960, 520))

camera.initialize()
motor.initialize()
servo.initialize()
vl53l1x.initialize()
bno055.initialize()
button = Button(23)
angle=0
prevangle=0
FRAME_WIDTH = 960
FRAME_HEIGHT = 520
FRAME_MIDPOINT_X = FRAME_WIDTH // 2

LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 255, 120])
LOWER_ORANGE = np.array([10, 100, 20])
UPPER_ORANGE = np.array([25, 255, 255])
detection_params = {'min_area': 300, 'return_rule': 'biggest_in_job', 'return_mask': True}

left_roi_x = 70
left_roi_y = 30
left_roi_w = 100
left_roi_h = 100

right_roi_x = 790
right_roi_y = 30
right_roi_w = 100
right_roi_h = 100

inner_left_roi_x = 70
inner_left_roi_y = 140
inner_left_roi_w = 100
inner_left_roi_h = 100

inner_right_roi_x = 790
inner_right_roi_y = 140
inner_right_roi_w = 100
inner_right_roi_h = 100

orange_roi_x = 440 
orange_roi_y = 120
orange_roi_w = 80
orange_roi_h = 20

left_side_job = {'roi': (left_roi_x, left_roi_y, left_roi_w, left_roi_h), 'type': 'wall_left','colors': [{'name': 'black', 'lower': LOWER_BLACK, 'upper': UPPER_BLACK}], 'params': detection_params}
right_side_job = {'roi': (right_roi_x, right_roi_y, right_roi_w, right_roi_h), 'type': 'wall_right', 'colors': [{'name': 'black', 'lower': LOWER_BLACK, 'upper': UPPER_BLACK}], 'params': detection_params}
inner_left_side_job = {'roi': (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h), 'type': 'wall_inner_left', 'colors': [{'name': 'black', 'lower': LOWER_BLACK, 'upper': UPPER_BLACK}], 'params': detection_params}
inner_right_side_job = {'roi': (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h), 'type': 'wall_inner_right', 'colors': [{'name': 'black', 'lower': LOWER_BLACK, 'upper': UPPER_BLACK}], 'params': detection_params}
detection_jobs = [left_side_job, right_side_job, inner_left_side_job, inner_right_side_job]

full_frame_roi = (0, 20, 940, 300)
color_detection_job = [{
    'roi': full_frame_roi,
    'type': 'block',
    'colors': [
        {'name': 'red', 'lower': LOWER_RED_1, 'upper': UPPER_RED_1},
        {'name': 'red', 'lower': LOWER_RED_2, 'upper': UPPER_RED_2},
        {'name': 'green', 'lower': LOWER_GREEN, 'upper': UPPER_GREEN}
    ],
    'params': {
        'return_rule': 'biggest_in_job',
        'return_mask': True,
        'min_area': 1000
    },
    },
    {'roi': (300,0,1,1),
        'type': 'close_block',
    'colors': [
        {'name': 'red', 'lower': LOWER_RED_1, 'upper': UPPER_RED_1},
        {'name': 'red', 'lower': LOWER_RED_2, 'upper': UPPER_RED_2},
        {'name': 'green', 'lower': LOWER_GREEN, 'upper': UPPER_GREEN}
    ],
    'params': {
        'return_rule': 'biggest_in_job',
        'return_mask': True,
        }
}]

debug=[]
DASHBOARD_MASK_WIDTH = 160
DASHBOARD_MASK_HEIGHT = 120
DASHBOARD_MASK_SIZE = (DASHBOARD_MASK_WIDTH, DASHBOARD_MASK_HEIGHT)

roi_mask_walls = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
for job in detection_jobs:
    x, y, w, h = job['roi']
    cv2.rectangle(roi_mask_walls, (x, y), (x + w, y + h), 255, -1)
x, y, w, h = color_detection_job[0]['roi']
roi_mask_main_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
cv2.rectangle(roi_mask_main_blocks, (x, y), (x + w, y + h), 255, -1)
x, y, w, h = color_detection_job[1]['roi']
roi_mask_close_blocks = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
cv2.rectangle(roi_mask_close_blocks, (x, y), (x + w, y + h), 255, -1)
roi_mask_orange = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype="uint8")
cv2.rectangle(roi_mask_orange, (orange_roi_x, orange_roi_y), (orange_roi_x + orange_roi_w, orange_roi_h + orange_roi_y), 255, -1)
WALL_MIN_AREA = detection_params['min_area']
BLOCK_MIN_AREA = color_detection_job[0]['params']['min_area']
CLOSE_BLOCK_MIN_AREA = color_detection_job[1]['params'].get('min_area', 1)

class CameraThread(threading.Thread):
    def __init__(self, camera_instance):
        super().__init__()
        self.camera = camera_instance
        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = True
        self.daemon = True # Allows main program to exit even if this thread is running

    def run(self):
        """This method runs in the separate thread."""
        while self.running:
            # The blocking call is now isolated in this thread
            frame = self.camera.capture_frame()
            with self.lock:
                self.latest_frame = frame

    def get_frame(self):
        """This is called by the main thread to get the latest frame."""
        with self.lock:
            # Return a copy to prevent race conditions where the camera
            # thread overwrites the frame while the main thread is using it.
            if self.latest_frame is not None:
                return self.latest_frame.copy()
            return None

    def stop(self):
        """Signals the thread to stop."""
        self.running = False
def prepare_mask_for_dashboard(mask):
    """
    Creates a standardized view for a mask by resizing it to fit within a
    standard pane, maintaining aspect ratio and adding black bars (letterboxing).
    This avoids cropping and ensures the entire mask is visible.
    """
    # Create a black background of the standard size.
    standard_pane = np.zeros((DASHBOARD_MASK_HEIGHT, DASHBOARD_MASK_WIDTH, 3), dtype=np.uint8)

    # If the input mask is invalid or empty, return the black pane.
    if mask is None or mask.size == 0:
        return standard_pane

    # Ensure the mask is in BGR format (3-channel).
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if len(mask.shape) == 2 else mask

    # --- Aspect Ratio Preserving Resize Logic ---
    h, w = mask_bgr.shape[:2]
    
    # Calculate the scaling factor. We need the smaller of the two ratios
    # to ensure the entire image fits without being cropped.
    scale = min(DASHBOARD_MASK_WIDTH / w, DASHBOARD_MASK_HEIGHT / h)
    
    # If the mask is already smaller than the pane, don't scale it up.
    # This prevents small noise masks from becoming large and distracting.
    if scale > 1.0:
        scale = 1.0

    # Calculate the new dimensions
    new_w, new_h = int(w * scale), int(h * scale)
    
    # Resize the mask to the new calculated dimensions
    resized_mask = cv2.resize(mask_bgr, (new_w, new_h), interpolation=cv2.INTER_AREA)
    
    # Calculate offsets to center the resized mask on the black pane
    y_offset = (DASHBOARD_MASK_HEIGHT - new_h) // 2
    x_offset = (DASHBOARD_MASK_WIDTH - new_w) // 2
    
    # Paste the resized mask onto the center of the standard pane
    standard_pane[y_offset:y_offset + new_h, x_offset:x_offset + new_w] = resized_mask

    return standard_pane
def create_dashboard_view_grid(main_feed, masks):
    """
    Combines the main video feed and six masks into a single dashboard view
    using a fixed grid layout that avoids resizing.

    Args:
        main_feed: The main camera frame.
        masks (list): A list of up to 6 mask images to display.
    """
    # --- 1. Prepare all masks to be a standard size ---
    # We use a list comprehension and our new helper function.
    prepared_masks = [prepare_mask_for_dashboard(m) for m in masks]
    
    # Ensure we have exactly 6 items to display, adding black panes if needed.
    while len(prepared_masks) < 6:
        prepared_masks.append(np.zeros((DASHBOARD_MASK_HEIGHT, DASHBOARD_MASK_WIDTH, 3), dtype=np.uint8))

    # --- 2. Build the side columns ---
    # Stack the first three masks vertically for the left column.
    left_column = cv2.vconcat([prepared_masks[0], prepared_masks[1], prepared_masks[2]])
    
    # Stack the next three masks vertically for the right column.
    right_column = cv2.vconcat([prepared_masks[3], prepared_masks[4], prepared_masks[5]])

    # --- 3. Match column height to main feed height ---
    main_feed_height = main_feed.shape[0]
    columns = []
    for col in [left_column, right_column]:
        col_height = col.shape[0]
        # If the column is shorter than the feed, add black padding at the bottom.
        if col_height < main_feed_height:
            padding_height = main_feed_height - col_height
            padding = np.zeros((padding_height, DASHBOARD_MASK_WIDTH, 3), dtype=np.uint8)
            columns.append(cv2.vconcat([col, padding]))
        # If the column is taller, resize it down (this is a fallback).
        else:
            columns.append(cv2.resize(col, (DASHBOARD_MASK_WIDTH, main_feed_height)))
            
    # --- 4. Combine everything horizontally ---
    # Final dashboard: [Left Column] [Main Feed] [Right Column]
    return cv2.hconcat([columns[0], main_feed, columns[1]])

camera_thread = CameraThread(camera)
camera_thread.start()
dist_left = vl53l1x.get_distance(3)
dist_right = vl53l1x.get_distance(2)
print(dist_left,dist_right)
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
    if False:
        if driving_direction == "clockwise":
            servo.set_angle_unlimited(60)
            print(INITIAL_HEADING, bno055.get_heading())
            while not(get_angular_difference(INITIAL_HEADING, bno055.get_heading())>= 30):
                print(INITIAL_HEADING, bno055.get_heading())
                time.sleep(0.05)
        else:
            servo.set_angle_unlimited(-50)
            print(INITIAL_HEADING, bno055.get_heading())
            while not(get_angular_difference(INITIAL_HEADING, bno055.get_heading())>= 30):
                print(INITIAL_HEADING, bno055.get_heading())
                time.sleep(0.05)
    motor.forward(MOTOR_SPEED)
    while True:
        detected_orange_object = []
        debug=[]
        frame = camera_thread.get_frame()
        if frame is None:
            continue
        frame = frame[426:946, 164:1124]
        frame = cv2.GaussianBlur(frame,(7,7),0)
        #frame = cv2.bilateralFilter(frame, d=9, sigmaColor=75, sigmaSpace=75)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(frame)
        s = np.clip(s*1.5,0,255).astype(np.uint8)
        v = np.clip(v*1.5,0,255).astype(np.uint8)
        frame = cv2.merge((h,s,v))
        
        mask_black = cv2.inRange(frame, LOWER_BLACK, UPPER_BLACK)
        mask_red1 = cv2.inRange(frame, LOWER_RED_1, UPPER_RED_1)
        mask_red2 = cv2.inRange(frame, LOWER_RED_2, UPPER_RED_2)
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
        mask_green = cv2.inRange(frame, LOWER_GREEN, UPPER_GREEN)
        mask_orange = cv2.inRange(frame, LOWER_ORANGE, UPPER_ORANGE)

        mask_red_or_green = cv2.bitwise_or(mask_red, mask_green)
        pure_black_mask = cv2.bitwise_and(mask_black, cv2.bitwise_not(mask_red_or_green))

        final_mask_walls = cv2.bitwise_and(pure_black_mask, roi_mask_walls)
        final_mask_main_red = cv2.bitwise_and(mask_red, roi_mask_main_blocks)
        final_mask_main_green = cv2.bitwise_and(mask_green, roi_mask_main_blocks)
        final_mask_close_red = cv2.bitwise_and(mask_red, roi_mask_close_blocks)
        final_mask_close_green = cv2.bitwise_and(mask_green, roi_mask_close_blocks)
        final_mask_orange = cv2.bitwise_and(mask_orange, roi_mask_orange)

        detected_blocks = []
        detected_walls = []
        masks_blocks = {}
        masks_walls = {}

        contours, _ = cv2.findContours(final_mask_main_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            if area > BLOCK_MIN_AREA:
                M = cv2.moments(biggest_contour)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                detected_blocks.append({'type': 'block', 'color': 'red', 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour})
        masks_blocks['job_0_red'] = final_mask_main_red

        contours, _ = cv2.findContours(final_mask_main_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            if area > BLOCK_MIN_AREA:
                M = cv2.moments(biggest_contour)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                detected_blocks.append({'type': 'block', 'color': 'green', 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour})
        masks_blocks['job_0_green'] = final_mask_main_green

        contours, _ = cv2.findContours(final_mask_close_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            if area > CLOSE_BLOCK_MIN_AREA:
                M = cv2.moments(biggest_contour)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                detected_blocks.append({'type': 'close_block', 'color': 'red', 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour})
        contours, _ = cv2.findContours(final_mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            if area > 20:  # You can set a minimum area for orange detection
                M = cv2.moments(biggest_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    detected_orange_object.append({'type': 'orange_block', 'color': 'orange', 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour})
        contours, _ = cv2.findContours(final_mask_close_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            if area > CLOSE_BLOCK_MIN_AREA:
                M = cv2.moments(biggest_contour)
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                detected_blocks.append({'type': 'close_block', 'color': 'green', 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour})
        main_blocks = [b for b in detected_blocks if b['type'] == 'block']
        other_blocks = [b for b in detected_blocks if b['type'] != 'block']

        # If both a red and a green main block were detected, find the biggest one
        if len(main_blocks) > 1:
            # Find the block with the maximum area from the main_blocks list
            biggest_main_block = max(main_blocks, key=lambda b: b['area'])
            # The new list of main blocks will contain only this single biggest block
            main_blocks = [biggest_main_block]

        # Reconstruct the detected_blocks list. It will now contain EITHER:
        # - The single biggest red or green block.
        # - Any 'close_block' objects that were detected.
        detected_blocks = main_blocks + other_blocks
        wall_contours_by_roi = {
            'wall_left': [], 'wall_right': [],
            'wall_inner_left': [], 'wall_inner_right': []
        }

        contours, _ = cv2.findContours(final_mask_walls, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area > WALL_MIN_AREA:
                M = cv2.moments(c)
                try:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                except ZeroDivisionError:
                    continue

                # Determine which ROI this contour belongs to
                job_type = 'unknown'
                if left_side_job['roi'][0] <= cx < left_side_job['roi'][0] + left_side_job['roi'][2] and left_side_job['roi'][1] <= cy < left_side_job['roi'][1] + left_side_job['roi'][3]:
                    job_type = left_side_job['type']
                elif right_side_job['roi'][0] <= cx < right_side_job['roi'][0] + right_side_job['roi'][2] and right_side_job['roi'][1] <= cy < right_side_job['roi'][1] + right_side_job['roi'][3]:
                    job_type = right_side_job['type']
                elif inner_left_side_job['roi'][0] <= cx < inner_left_side_job['roi'][0] + inner_left_side_job['roi'][2] and inner_left_side_job['roi'][1] <= cy < inner_left_side_job['roi'][1] + inner_left_side_job['roi'][3]:
                    job_type = inner_left_side_job['type']
                elif inner_right_side_job['roi'][0] <= cx < inner_right_side_job['roi'][0] + inner_right_side_job['roi'][2] and inner_right_side_job['roi'][1] <= cy < inner_right_side_job['roi'][1] + inner_right_side_job['roi'][3]:
                    job_type = inner_right_side_job['type']
                
                # Apply the aspect ratio filter ONLY to the outer walls
                #if job_type in ['wall_left', 'wall_right']:
                #    _, _, w, h = cv2.boundingRect(c)
                #    if h == 0 or not (w < 0.3 * h):
                #        continue # Skip if it has no height or is too wide

                # If the contour is valid, add it to the list for its specific ROI
                if job_type != 'unknown':
                    wall_contours_by_roi[job_type].append(c)

        # 2. Now, find the single biggest contour from each ROI's list and process it
        for job_type, contour_list in wall_contours_by_roi.items():
            # If the list for this ROI is not empty...
            if contour_list:
                # ...find the single biggest contour in that list.
                biggest_contour = max(contour_list, key=cv2.contourArea)
                
                # Now we process ONLY this biggest contour
                area = cv2.contourArea(biggest_contour)
                M = cv2.moments(biggest_contour)
                try:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    # Add the final result to our detected_walls list
                    detected_walls.append({'type': job_type, 'color': 'black', 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour})
                except ZeroDivisionError:
                    continue

        lx, ly, lw, lh = left_side_job['roi']
        rx, ry, rw, rh = right_side_job['roi']
        ilx, ily, ilw, ilh = inner_left_side_job['roi']
        irx, iry, irw, irh = inner_right_side_job['roi']

        temp_mask = np.zeros_like(final_mask_walls) # Create black background once
        masks_walls['job_0_black'] = cv2.bitwise_and(final_mask_walls, cv2.rectangle(temp_mask.copy(), (lx, ly), (lx + lw, ly + lh), 255, -1))
        masks_walls['job_1_black'] = cv2.bitwise_and(final_mask_walls, cv2.rectangle(temp_mask.copy(), (rx, ry), (rx + rw, ry + rh), 255, -1))
        masks_walls['job_2_black'] = cv2.bitwise_and(final_mask_walls, cv2.rectangle(temp_mask.copy(), (ilx, ily), (ilx + ilw, ily + ilh), 255, -1))
        masks_walls['job_3_black'] = cv2.bitwise_and(final_mask_walls, cv2.rectangle(temp_mask.copy(), (irx, iry), (irx + irw, iry + irh), 255, -1))
        orange_detected_this_frame = bool(detected_orange_object)
        orange_detection_history.append(orange_detected_this_frame)
        if len(orange_detection_history) >= 3:
            # Check if the last two frames were detections, and the one before was NOT.
            if orange_detection_history[-1] and orange_detection_history[-2] and not orange_detection_history[-3]:
                
                # Before incrementing, check the older history for the 15-frame gap.
                # This checks if there were ANY detections in the 13 frames prior to the current 2-frame streak.
                # Note: list(orange_detection_history) creates a temporary copy.
                if not any(list(orange_detection_history)[:-2]):
                    turn_counter += 1
                    # You can add a print statement to verify it's working
                    print(f"Orange detected after a gap. Turn counter is now: {turn_counter}")
        annotated_frame = frame 
        if detected_blocks:
            for block in detected_blocks:
                if block['type'] == 'close_block':
                    if block['color'] == 'red':
                        angle=-25
                    else:
                        angle=20
                    servo.set_angle(angle)
                    motor.reverse(MOTOR_SPEED)
                    time.sleep(0.5)
                    break
            else:
                motor.forward(MOTOR_SPEED)
                block = detected_blocks[0]
                block_color = block['color']
                block_x = block['centroid'][0]
                block_y = block['centroid'][1]
                if 200<block_x<760:
                    if block_color == 'red':
                        wall_inner_right_size = 0
                        for obj in detected_walls:
                            if obj['type'] == 'wall_inner_right':
                                wall_inner_right_size = obj['area']
                        angle = ((block_x - (480-150)) * 0.05)+8
                        if wall_inner_right_size > 3000:
                            angle = np.clip(angle,-25,0)
                        else:
                            angle = np.clip(angle,-25,25)
                    elif block_color == 'green':
                        wall_inner_left_size = 0
                        for obj in detected_walls:
                            if obj['type'] == 'wall_inner_left':
                                wall_inner_left_size = obj['area']
                        angle = ((block_x - (480+150)) * 0.15) + 5
                        if wall_inner_left_size > 3000:
                            angle = np.clip(angle,0,25)
                        else:
                            angle = np.clip(angle,-25,25)
        else:
            left_pixel_size = 0
            right_pixel_size = 0
            for obj in detected_walls:
                if obj['type'] == 'wall_left':
                    left_pixel_size = obj['area']
                elif obj['type'] == 'wall_right':
                    right_pixel_size = obj['area']
            else:
                angle=((left_pixel_size-right_pixel_size)*0.003)+5
        annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_HSV2BGR)
        for wall in detected_walls:
            contour = wall['contour']
            area = wall['area']
            centroid = wall['centroid']
            cv2.drawContours(annotated_frame, [contour], -1, (255, 0, 0), 2)
            text = f"A: {area}"
            text_pos = (centroid[0] - 30, centroid[1])
            cv2.putText(annotated_frame, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.rectangle(annotated_frame, (orange_roi_x, orange_roi_y), (orange_roi_x + orange_roi_w, orange_roi_y + orange_roi_h), (0, 165, 255), 2)
        for block in detected_blocks:
            contour = block['contour']
            centroid = block['centroid']
            x_coord = centroid[0]
            draw_color = (0, 0, 255) if block['color'] == 'red' else (0, 255, 0)

            cv2.drawContours(annotated_frame, [contour], -1, draw_color, 2)
            # ... rest of the text drawing for blocks ...

        if detected_orange_object:
            # Loop through all detected orange objects (even if it's just one)
            for orange_obj in detected_orange_object:
                contour = orange_obj['contour']
                centroid = orange_obj['centroid']
                area = orange_obj['area']
                
                # Define the draw color for orange (BGR format)
                draw_color = (0, 165, 255) 
                
                # Draw the contour on the frame
                cv2.drawContours(annotated_frame, [contour], -1, draw_color, 2)
                
                # Add text to display the area of the orange object
                text = f"Orange: {area}"
                text_pos = (centroid[0] - 25, centroid[1])
                cv2.putText(annotated_frame, text, text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        
        debug.append(angle)
        info=str(debug)
        cv2.putText(annotated_frame,info,(300,50),cv2.FONT_HERSHEY_COMPLEX,1,(0,0,255))
        x, y, w, h = color_detection_job[1]['roi']
        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        out.write(annotated_frame)
        #angle = np.clip(angle, -30,40)
        #angle=np.clip(angle,prevangle-5, prevangle+5)
        servo.set_angle(angle)
        prevangle=angle
        if False:
            cv2.imshow("Robot Live Feed", annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        if button.is_pressed:
            motor.brake()
            break
finally:
    profiler.disable()
    motor.brake()
    out.release()
    print("Stopping profiler and saving stats...")
    profiler.dump_stats("loop_performance.pstats")
    camera.cleanup()
    servo.set_angle(0)
    servo.cleanup()
    motor.cleanup()
    cv2.destroyAllWindows()