import cv2
import numpy as np
import sys
import os
import time
from src.sensors import camera
from src.obstacle_challenge.main_v3 import HSV_RANGES, LAB_RANGES

# --- Configuration ---
# Change these variables to switch modes
TARGET_COLOR = 'RED'   # Options: RED, GREEN, BLUE, MAGENTA, ORANGE, BLACK
TARGET_SPACE = 'LAB'   # Options: HSV, LAB
# ---------------------

def nothing(x):
    pass

def get_default_ranges(color, space):
    ranges_dict = HSV_RANGES if space == 'HSV' else LAB_RANGES
    
    lower_key = f'LOWER_{color}'
    upper_key = f'UPPER_{color}'
    
    # Handle Red HSV special case (split range)
    if color == 'RED' and space == 'HSV':
        l1 = ranges_dict.get('LOWER_RED_1', np.array([0, 100, 100]))
        u1 = ranges_dict.get('UPPER_RED_1', np.array([10, 255, 255]))
        l2 = ranges_dict.get('LOWER_RED_2', np.array([160, 100, 100]))
        u2 = ranges_dict.get('UPPER_RED_2', np.array([180, 255, 255]))
        return (l1, u1, l2, u2)
    
    # Handle Red LAB (might use RED_1 keys if RED doesn't exist, or just RED if I added it? 
    # In main_v3 I added LOWER_RED_1 and LOWER_RED_2 for LAB too. 
    # Let's assume for LAB we just use RED_1 for now or check if RED exists.
    # The dictionary keys in main_v3 are LOWER_RED_1, etc.
    
    if color == 'RED':
         # For LAB, we might just want one range if it's continuous, but main_v3 has 2.
         # If the user wants to tune "RED", and we have RED_1 and RED_2, we should probably just tune RED_1 
         # unless we want 2 ranges for LAB too. 
         # The user said "For red and hsv more trackbars are needed", implying LAB Red is single range.
         # So for LAB Red, I'll default to LOWER_RED_1.
         lower_key = 'LOWER_RED_1'
         upper_key = 'UPPER_RED_1'

    l = ranges_dict.get(lower_key, np.array([0, 0, 0]))
    u = ranges_dict.get(upper_key, np.array([255, 255, 255]))
    return (l, u)

def main():
    print(f"Starting Color Tuning for {TARGET_COLOR} in {TARGET_SPACE} space...")
    
    if not camera.initialize():
        print("Failed to initialize camera.")
        return

    window_name = f"Tuning {TARGET_COLOR} ({TARGET_SPACE})"
    cv2.namedWindow(window_name)
    
    defaults = get_default_ranges(TARGET_COLOR, TARGET_SPACE)
    
    if TARGET_COLOR == 'RED' and TARGET_SPACE == 'HSV':
        l1, u1, l2, u2 = defaults
        # Set 1
        cv2.createTrackbar('L1_1', window_name, l1[0], 180, nothing)
        cv2.createTrackbar('L1_2', window_name, l1[1], 255, nothing)
        cv2.createTrackbar('L1_3', window_name, l1[2], 255, nothing)
        cv2.createTrackbar('U1_1', window_name, u1[0], 180, nothing)
        cv2.createTrackbar('U1_2', window_name, u1[1], 255, nothing)
        cv2.createTrackbar('U1_3', window_name, u1[2], 255, nothing)
        
        # Set 2
        cv2.createTrackbar('L2_1', window_name, l2[0], 180, nothing)
        cv2.createTrackbar('L2_2', window_name, l2[1], 255, nothing)
        cv2.createTrackbar('L2_3', window_name, l2[2], 255, nothing)
        cv2.createTrackbar('U2_1', window_name, u2[0], 180, nothing)
        cv2.createTrackbar('U2_2', window_name, u2[1], 255, nothing)
        cv2.createTrackbar('U2_3', window_name, u2[2], 255, nothing)
    else:
        l, u = defaults
        max_val_1 = 180 if TARGET_SPACE == 'HSV' else 255
        
        cv2.createTrackbar('L_1', window_name, l[0], max_val_1, nothing)
        cv2.createTrackbar('L_2', window_name, l[1], 255, nothing)
        cv2.createTrackbar('L_3', window_name, l[2], 255, nothing)
        cv2.createTrackbar('U_1', window_name, u[0], max_val_1, nothing)
        cv2.createTrackbar('U_2', window_name, u[1], 255, nothing)
        cv2.createTrackbar('U_3', window_name, u[2], 255, nothing)

    try:
        while True:
            frame = camera.capture_frame()
            if frame is None:
                time.sleep(0.1)
                continue
            
            # Blur
            frame = cv2.GaussianBlur(frame, (5, 5), 0)
            
            if TARGET_SPACE == 'HSV':
                converted = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            else:
                converted = cv2.cvtColor(frame, cv2.COLOR_BGR2Lab)
                
            mask = None
            
            if TARGET_COLOR == 'RED' and TARGET_SPACE == 'HSV':
                l1_1 = cv2.getTrackbarPos('L1_1', window_name)
                l1_2 = cv2.getTrackbarPos('L1_2', window_name)
                l1_3 = cv2.getTrackbarPos('L1_3', window_name)
                u1_1 = cv2.getTrackbarPos('U1_1', window_name)
                u1_2 = cv2.getTrackbarPos('U1_2', window_name)
                u1_3 = cv2.getTrackbarPos('U1_3', window_name)
                
                l2_1 = cv2.getTrackbarPos('L2_1', window_name)
                l2_2 = cv2.getTrackbarPos('L2_2', window_name)
                l2_3 = cv2.getTrackbarPos('L2_3', window_name)
                u2_1 = cv2.getTrackbarPos('U2_1', window_name)
                u2_2 = cv2.getTrackbarPos('U2_2', window_name)
                u2_3 = cv2.getTrackbarPos('U2_3', window_name)
                
                lower1 = np.array([l1_1, l1_2, l1_3])
                upper1 = np.array([u1_1, u1_2, u1_3])
                lower2 = np.array([l2_1, l2_2, l2_3])
                upper2 = np.array([u2_1, u2_2, u2_3])
                
                mask1 = cv2.inRange(converted, lower1, upper1)
                mask2 = cv2.inRange(converted, lower2, upper2)
                mask = cv2.bitwise_or(mask1, mask2)
                
                # Print current values occasionally or on change?
                # For now just let user see visual result
                
            else:
                l_1 = cv2.getTrackbarPos('L_1', window_name)
                l_2 = cv2.getTrackbarPos('L_2', window_name)
                l_3 = cv2.getTrackbarPos('L_3', window_name)
                u_1 = cv2.getTrackbarPos('U_1', window_name)
                u_2 = cv2.getTrackbarPos('U_2', window_name)
                u_3 = cv2.getTrackbarPos('U_3', window_name)
                
                lower = np.array([l_1, l_2, l_3])
                upper = np.array([u_1, u_2, u_3])
                
                mask = cv2.inRange(converted, lower, upper)

            # Show Original, Mask, Mask Applied
            res = cv2.bitwise_and(frame, frame, mask=mask)
            
            # Stack images for display
            # Convert mask to BGR so we can stack
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            
            # Resize for better visibility if needed, but 640x360 is small enough
            # Stack: Top: Original, Bottom: Mask | Result
            # Or 3 side by side? 640*3 = 1920, fits on most screens.
            
            stacked = np.hstack((frame, mask_bgr, res))
            
            cv2.imshow(window_name, stacked)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                print(f"--- Saved Values for {TARGET_COLOR} ({TARGET_SPACE}) ---")
                if TARGET_COLOR == 'RED' and TARGET_SPACE == 'HSV':
                    print(f"LOWER_RED_1 = np.array([{l1_1}, {l1_2}, {l1_3}])")
                    print(f"UPPER_RED_1 = np.array([{u1_1}, {u1_2}, {u1_3}])")
                    print(f"LOWER_RED_2 = np.array([{l2_1}, {l2_2}, {l2_3}])")
                    print(f"UPPER_RED_2 = np.array([{u2_1}, {u2_2}, {u2_3}])")
                else:
                    print(f"LOWER_{TARGET_COLOR} = np.array([{l_1}, {l_2}, {l_3}])")
                    print(f"UPPER_{TARGET_COLOR} = np.array([{u_1}, {u_2}, {u_3}])")

    except KeyboardInterrupt:
        pass
    finally:
        camera.cleanup()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

