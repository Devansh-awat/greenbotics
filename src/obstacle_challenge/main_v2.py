from collections import deque
import time
from src.sensors import camera
from src.motors import motor, servo
import numpy as np
import cv2
import cProfile
import threading
from gpiozero import Button

# Import from the new configuration file
from src.obstacle_challenge.config_v2 import *

# --- Initialization ---
MOTOR_SPEED = 100
prevangle=0
orange_detection_history = deque(maxlen=15)
turn_counter = 0
profiler = cProfile.Profile()
fourcc = cv2.VideoWriter_fourcc(*'avc1')
out = cv2.VideoWriter('recording_lab_roi.mp4', fourcc, 30, (FRAME_WIDTH, FRAME_HEIGHT))
button = Button(23)
# (CameraThread class remains the same as before)
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

def process_image(frame):
    """
    Performs image processing using bitwise AND on each ROI to find the 
    single biggest contour within that specific region.
    """
    lab_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    blurred_frame = cv2.GaussianBlur(lab_frame, (9, 9), 0)
    
    x_o, y_o, w_o, h_o = ROI_ORANGE
    orange_roi_crop = lab_frame[y_o:y_o+h_o, x_o:x_o+w_o]
    orange_roi_filtered = cv2.bilateralFilter(orange_roi_crop, d=9, sigmaColor=75, sigmaSpace=75)
    blurred_frame[y_o:y_o+h_o, x_o:x_o+w_o] = orange_roi_filtered

    # --- Global Color Masks ---
    mask_black = cv2.inRange(blurred_frame, LOWER_BLACK_LAB, UPPER_BLACK_LAB)
    mask_red = cv2.inRange(blurred_frame, LOWER_RED_LAB, UPPER_RED_LAB)
    mask_green = cv2.inRange(blurred_frame, LOWER_GREEN_LAB, UPPER_GREEN_LAB)
    mask_orange = cv2.inRange(blurred_frame, LOWER_ORANGE_LAB, UPPER_ORANGE_LAB)

    detected_walls = []
    detected_blocks = []
    detected_orange_object = None
    
    # --- Wall Detection: One ROI at a time ---
    for name, roi in WALL_ROIS.items():
        x, y, w, h = roi
        # Create a mask for the specific ROI
        roi_mask = np.zeros(frame.shape[:2], dtype="uint8")
        cv2.rectangle(roi_mask, (x, y), (x + w, y + h), 255, -1)
        
        # Isolate the black mask to only this ROI
        isolated_mask = cv2.bitwise_and(mask_black, roi_mask)
        
        contours, _ = cv2.findContours(isolated_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            biggest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest_contour)
            if area > WALL_MIN_AREA:
                M = cv2.moments(biggest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    detected_walls.append({'type': name, 'area': area, 'centroid': (cx, cy), 'contour': biggest_contour})

    # --- Block Detection in Main ROI ---
    x, y, w, h = ROI_BLOCKS_MAIN
    block_roi_mask = np.zeros(frame.shape[:2], dtype="uint8")
    cv2.rectangle(block_roi_mask, (x, y), (x + w, y + h), 255, -1)
    
    isolated_red_mask = cv2.bitwise_and(mask_red, block_roi_mask)
    isolated_green_mask = cv2.bitwise_and(mask_green, block_roi_mask)
    
    contours_red, _ = cv2.findContours(isolated_red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(isolated_green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    biggest_red_block = {'area': 0}
    if contours_red:
        contour = max(contours_red, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area > BLOCK_MIN_AREA:
            M = cv2.moments(contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            biggest_red_block = {'type': 'block', 'color': 'red', 'area': area, 'centroid': (cx, cy), 'contour': contour}

    biggest_green_block = {'area': 0}
    if contours_green:
        contour = max(contours_green, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area > BLOCK_MIN_AREA:
            M = cv2.moments(contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            biggest_green_block = {'type': 'block', 'color': 'green', 'area': area, 'centroid': (cx, cy), 'contour': contour}
            
    # Only keep the single largest block (red or green)
    if biggest_red_block['area'] > 0 or biggest_green_block['area'] > 0:
        if biggest_red_block['area'] > biggest_green_block['area']:
            detected_blocks.append(biggest_red_block)
        else:
            detected_blocks.append(biggest_green_block)
            
    # --- Orange Detection in Orange ROI ---
    x, y, w, h = ROI_ORANGE
    orange_roi_mask = np.zeros(frame.shape[:2], dtype="uint8")
    cv2.rectangle(orange_roi_mask, (x, y), (x + w, y + h), 255, -1)
    isolated_orange_mask = cv2.bitwise_and(mask_orange, orange_roi_mask)

    contours, _ = cv2.findContours(isolated_orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        biggest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(biggest_contour)
        if area > ORANGE_MIN_AREA:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                detected_orange_object = {'area': area, 'centroid': (cx, cy), 'contour': biggest_contour}
                
    return detected_walls, detected_blocks, detected_orange_object


def draw_annotations(frame, walls, blocks, orange_object, debug_info=""):
    """
    (This function remains the same as before)
    """
    annotated_frame = frame.copy()
    
    # Draw ROIs
    for roi in WALL_ROIS.values():
        cv2.rectangle(annotated_frame, roi[:2], (roi[0]+roi[2], roi[1]+roi[3]), (255, 255, 0), 2)
    cv2.rectangle(annotated_frame, ROI_BLOCKS_MAIN[:2], (ROI_BLOCKS_MAIN[0]+ROI_BLOCKS_MAIN[2], ROI_BLOCKS_MAIN[1]+ROI_BLOCKS_MAIN[3]), (0, 255, 255), 1)
    cv2.rectangle(annotated_frame, ROI_ORANGE[:2], (ROI_ORANGE[0]+ROI_ORANGE[2], ROI_ORANGE[1]+ROI_ORANGE[3]), (0, 165, 255), 2)

    # Draw detected walls
    for wall in walls:
        cv2.drawContours(annotated_frame, [wall['contour']], -1, (255, 0, 0), 2)
        text = f"{wall['type']}: {wall['area']}"
        cv2.putText(annotated_frame, text, wall['centroid'], cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Draw detected blocks
    for block in blocks:
        draw_color = (0, 0, 255) if block['color'] == 'red' else (0, 255, 0)
        cv2.drawContours(annotated_frame, [block['contour']], -1, draw_color, 2)
        text_pos = (block['centroid'][0] - 20, block['centroid'][1] - 15)
        cv2.putText(annotated_frame, f"X: {block['centroid'][0]}", text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    # Draw detected orange object
    if orange_object:
        cv2.drawContours(annotated_frame, [orange_object['contour']], -1, (0, 165, 255), 2)
        text_pos = (orange_object['centroid'][0] - 20, orange_object['centroid'][1] - 15)
        cv2.putText(annotated_frame, f"O: {orange_object['area']}", text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    
    cv2.putText(annotated_frame, debug_info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    
    return annotated_frame

# --- Main Loop ---
#camera_thread = CameraThread(camera)
#camera_thread.start()
camera.initialize()
motor.forward(MOTOR_SPEED)

try:
    profiler.enable()
    while True:
        #frame = camera_thread.get_frame()
        frame = camera.capture_frame()
        if frame is None:
            continue
            
        detected_walls, detected_blocks, detected_orange = process_image(frame)
        
        # (Orange detection history logic remains the same)
        orange_detected_this_frame = detected_orange is not None
        orange_detection_history.append(orange_detected_this_frame)
        if len(orange_detection_history) >= 3 and orange_detection_history[-1] and orange_detection_history[-2] and not orange_detection_history[-3]:
            if not any(list(orange_detection_history)[:-2]):
                turn_counter += 1
                print(f"Orange detected after a gap. Turn counter is now: {turn_counter}")

        # --- Decision Making Logic (Updated) ---
        if detected_blocks:
            block = detected_blocks[0]
            block_x = block['centroid'][0]
            if block['color'] == 'red':
                angle = ((block_x - 200) * 0.13) + 8
            else: # Green
                kp = block['area'] / 6000
                angle = ((block_x - 490) * kp) + 8
        else: # No blocks, use walls for navigation
            # Find the specific left and right walls from the detected list
            left_wall = next((w for w in detected_walls if w['type'] == 'wall_left'), None)
            right_wall = next((w for w in detected_walls if w['type'] == 'wall_right'), None)

            # Get their area, or 0 if they weren't found
            left_pixel_size = left_wall['area'] if left_wall else 0
            right_pixel_size = right_wall['area'] if right_wall else 0
            
            angle = ((left_pixel_size - right_pixel_size) * 0.0030) + 5
        
        # Angle Smoothing
        angle = np.clip(angle, prevangle - 5, prevangle + 5)
        servo.set_angle(angle)
        prevangle = angle

        debug_text = f"Angle: {angle:.2f}"
        annotated_frame = draw_annotations(frame, detected_walls, detected_blocks, detected_orange, debug_text)

        cv2.imshow("Robot Live Feed", annotated_frame)
        out.write(annotated_frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q') or button.is_pressed:
            break

finally:
    motor.brake()
    out.release()
    profiler.disable()
    profiler.dump_stats("loop_performance_lab_roi.pstats")
    #camera_thread.stop()
    camera.cleanup()
    servo.cleanup()
    motor.cleanup()
    cv2.destroyAllWindows()
    print("Execution finished and resources cleaned up.")
