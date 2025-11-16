import cv2
import numpy as np
import os
import time
from src.sensors import camera

# --- Configuration Settings ---
# HSV Color Thresholds
LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 255, 120])
LOWER_ORANGE = np.array([6, 70, 20])
UPPER_ORANGE = np.array([26, 255, 255])

# Area thresholds for contour detection
ORANGE_MIN_AREA = 500
WALL_MIN_AREA = 500 # A smaller value might be needed for smaller inner ROIs

# --- Regions of Interest (ROI) ---
# Using the new user-provided ROI definitions
# ROIs for Wall Detection
left_roi_x, left_roi_y, left_roi_w, left_roi_h = 0, 140, 135, 150
right_roi_x, right_roi_y, right_roi_w, right_roi_h = 505, 140, 135, 150
inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h = 140, 165, 100, 100
inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h = 400, 165 , 100, 100
close_x, close_y, close_w, close_h = 140, 120, 360, 10

# Jobs for wall detection, combining ROI and type
wall_jobs = [
    {'roi': (left_roi_x, left_roi_y, left_roi_w, left_roi_h), 'type': 'wall_left'},
    {'roi': (right_roi_x, right_roi_y, right_roi_w, right_roi_h), 'type': 'wall_right'},
    {'roi': (inner_left_roi_x, inner_left_roi_y, inner_left_roi_w, inner_left_roi_h), 'type': 'wall_inner_left'},
    {'roi': (inner_right_roi_x, inner_right_roi_y, inner_right_roi_w, inner_right_roi_h), 'type': 'wall_inner_right'}
]

# ROI for Orange Line Detection
orange_roi_x, orange_roi_y, orange_roi_w, orange_roi_h = 280, 200, 80, 40


def process_and_save_steps(frame):
    """
    Processes a single frame using the new ROI strategy, saving each step.
    """
    print("\nStarting image processing steps...")
    frame_height, frame_width, _ = frame.shape

    # 1. Save the original frame
    cv2.imwrite("01_original_frame.png", frame)
    print("Saved: 01_original_frame.png")

    # 2. Apply Gaussian Blur
    blurred_frame = cv2.GaussianBlur(frame, (1, 7), 0)
    cv2.imwrite("02_blurred_frame.png", blurred_frame)
    print("Saved: 02_blurred_frame.png")

    # 3. Convert to HSV color space
    hsv_frame = cv2.cvtColor(blurred_frame, cv2.COLOR_BGR2HSV)
    cv2.imwrite("03_hsv_frame.png", hsv_frame)
    print("Saved: 03_hsv_frame.png")

    # 4. Create base color masks
    mask_black = cv2.inRange(hsv_frame, LOWER_BLACK, UPPER_BLACK)
    cv2.imwrite("04_mask_black.png", mask_black)
    print("Saved: 04_mask_black.png")

    mask_orange = cv2.inRange(hsv_frame, LOWER_ORANGE, UPPER_ORANGE)
    cv2.imwrite("05_mask_orange.png", mask_orange)
    print("Saved: 05_mask_orange.png")
    
    # --- Data structure for storing detection results ---
    processed_data = {
        'detected_walls': [],
        'detected_orange': [],
        'detected_close_black': []
    }

    # --- 5. Process Each ROI Individually ---

    # --- Orange Line Detection ---
    roi_orange_mask = np.zeros_like(mask_orange)
    cv2.rectangle(roi_orange_mask, (orange_roi_x, orange_roi_y), (orange_roi_x + orange_roi_w, orange_roi_y + orange_roi_h), 255, -1)
    final_mask_orange = cv2.bitwise_and(mask_orange, mask_orange, mask=roi_orange_mask)
    cv2.imwrite("06_final_mask_orange.png", final_mask_orange)
    print("Saved: 06_final_mask_orange.png")
    
    contours, _ = cv2.findContours(final_mask_orange, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        biggest_contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(biggest_contour) > ORANGE_MIN_AREA:
            M = cv2.moments(biggest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                processed_data['detected_orange'].append({'centroid': (cx, cy), 'contour': biggest_contour})

    # --- Close Black Line Detection ---
    roi_close_black_mask = np.zeros_like(mask_black)
    cv2.rectangle(roi_close_black_mask, (close_x, close_y), (close_x + close_w, close_y + close_h), 255, -1)
    final_mask_close_black = cv2.bitwise_and(mask_black, mask_black, mask=roi_close_black_mask)
    cv2.imwrite("07_final_mask_close_black.png", final_mask_close_black)
    print("Saved: 07_final_mask_close_black.png")

    contours, _ = cv2.findContours(final_mask_close_black, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        # Using a smaller area threshold for this thin ROI might be necessary
        if cv2.contourArea(contour) > 50: 
            processed_data['detected_close_black'].append({'contour': contour})

    # --- Wall Detection (Iterating through each job) ---
    # Create a composite image to save the final combined wall mask
    combined_wall_mask = np.zeros_like(mask_black)

    for job in wall_jobs:
        x, y, w, h = job['roi']
        job_type = job['type']
        
        # Create a mask for this specific job's ROI
        roi_job_mask = np.zeros_like(mask_black)
        cv2.rectangle(roi_job_mask, (x, y), (x + w, y + h), 255, -1)
        
        # Apply the mask
        final_mask_job = cv2.bitwise_and(mask_black, mask_black, mask=roi_job_mask)
        combined_wall_mask = cv2.bitwise_or(combined_wall_mask, final_mask_job)
        
        # Find contours only within this specific ROI
        contours, _ = cv2.findContours(final_mask_job, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) > WALL_MIN_AREA:
                # Type is known directly from the job, no need to check centroid
                processed_data['detected_walls'].append({'type': job_type, 'contour': c})
    
    cv2.imwrite("08_final_mask_walls_combined.png", combined_wall_mask)
    print("Saved: 08_final_mask_walls_combined.png")
    
    print("...image processing complete.")
    return processed_data


def annotate_frame_and_save(frame, detections):
    """
    Draws new ROIs and detections on the frame and saves the final result.
    """
    print("\nStarting final annotation...")
    annotated_frame = frame.copy()
    light_blue = (255, 255, 0)
    red = (0, 0, 255)
    orange_color = (0, 165, 255)
    green = (0, 255, 0)

    # 1. Draw all ROIs
    # Draw wall ROIs
    for job in wall_jobs:
        x, y, w, h = job['roi']
        cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), light_blue, 1)
        # Optional: Put text to label the ROI
        cv2.putText(annotated_frame, job['type'], (x + 5, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, light_blue, 1)

    # Draw orange and close black ROIs
    cv2.rectangle(annotated_frame, (orange_roi_x, orange_roi_y), (orange_roi_x + orange_roi_w, orange_roi_y + orange_roi_h), light_blue, 2)
    cv2.rectangle(annotated_frame, (close_x, close_y), (close_x + close_w, close_y + close_h), light_blue, 2)

    # 2. Draw detected contours
    for wall in detections['detected_walls']:
        cv2.drawContours(annotated_frame, [wall['contour']], -1, red, 2)
    
    for line in detections['detected_close_black']:
        cv2.drawContours(annotated_frame, [line['contour']], -1, green, 2)

    for orange in detections['detected_orange']:
        cv2.drawContours(annotated_frame, [orange['contour']], -1, orange_color, 2)
        cv2.circle(annotated_frame, orange['centroid'], 5, orange_color, -1)

    # 3. Save the final annotated image
    cv2.imwrite("09_annotated_frame.png", annotated_frame)
    print("Saved: 09_annotated_frame.png")
    print("...annotation complete.")


if __name__ == "__main__":
    camera.initialize()
    
    print("Camera initialized. Allowing 2 seconds for sensor to adjust...")
    time.sleep(2)

    print("\nCapturing a single frame...")
    current_frame = camera.capture_frame()
    
    if current_frame is not None:
        frame_height, frame_width, _ = current_frame.shape
        print(f"Frame captured successfully. Actual dimensions: {frame_width}x{frame_height}")
        
        # Process the frame and save all intermediate steps
        detection_data = process_and_save_steps(current_frame)

        # Annotate the original frame with the final detections and save it
        annotate_frame_and_save(current_frame, detection_data)
        
        print(f"\nAll steps saved as PNG files in the directory: {os.getcwd()}")
    else:
        print("\nError: Failed to capture a valid frame from the camera.")

    # Release the camera resource
    camera.cleanup()
    print("\nScript finished and camera released.")
