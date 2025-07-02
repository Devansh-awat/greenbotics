# camera.py
from picamera2 import Picamera2
import cv2
import numpy as np
import time
import config

# Module-level hardware object
picam2 = None

def initialize():
    """Initializes the Picamera2."""
    global picam2
    try:
        picam2 = Picamera2()
        cam_config = picam2.create_preview_configuration(
            main={"format": 'XRGB8888', "size": (config.FRAME_WIDTH, config.FRAME_HEIGHT)},
            controls={"FrameRate": config.MAX_FPS}
        )
        picam2.configure(cam_config)
        picam2.start()
        time.sleep(1.0)
        print("INFO: Camera Initialized.")
        return True
    except Exception as e:
        print(f"FATAL: Camera failed to initialize: {e}")
        return False

def capture_frame():
    """Captures and returns a single frame from the camera."""
    if picam2:
        frame = picam2.capture_array("main")
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        return frame
    return None

def find_biggest_block(frame):
    """Processes a frame to find the largest red or green block."""
    h, w, _ = frame.shape
    roi_top_y = int(h * config.CROP_TOP_FRAC)
    roi_bottom_y = int(h * (1.0 - config.CROP_BOTTOM_FRAC))
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_r1 = cv2.inRange(hsv, config.LOWER_RED_1, config.UPPER_RED_1)
    mask_r2 = cv2.inRange(hsv, config.LOWER_RED_2, config.UPPER_RED_2)
    red_mask = cv2.bitwise_or(mask_r1, mask_r2)
    green_mask = cv2.inRange(hsv, config.LOWER_GREEN, config.UPPER_GREEN)
    
    contours_red, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours_green, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    valid_blocks = []
    for contours, color in [(contours_red, "red"), (contours_green, "green")]:
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < config.MIN_CONTOUR_AREA: continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            if not ch > cw: continue
            if y < roi_bottom_y and (y + ch) > roi_top_y:
                valid_blocks.append({'contour': cnt, 'color': color, 'area': area})

    overlay_frame = frame.copy()
    cv2.line(overlay_frame, (0, roi_top_y), (w, roi_top_y), config.BOX_COLOR_ROI, 1)
    cv2.line(overlay_frame, (0, roi_bottom_y), (w, roi_bottom_y), config.BOX_COLOR_ROI, 1)
    
    if not valid_blocks:
        return None, overlay_frame

    largest_block = max(valid_blocks, key=lambda b: b['area'])
    x, y, cw, ch = cv2.boundingRect(largest_block['contour'])
    box_color = config.BOX_COLOR_RED if largest_block['color'] == "red" else config.BOX_COLOR_GREEN
    
    cv2.rectangle(overlay_frame, (x, y), (x + cw, y + ch), box_color, 2)
    cv2.putText(overlay_frame, largest_block['color'].upper(), (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
    
    return largest_block, overlay_frame

def cleanup():
    """Stops the camera."""
    print("--- Cleaning up Camera ---")
    if picam2:
        picam2.stop()
    cv2.destroyAllWindows()

# Test routine
if __name__ == "__main__":
    print("--- Testing Camera and Vision Module ---")
    if not initialize():
        print("Camera test failed during initialization.")
    else:
        try:
            print("Displaying camera feed with block detection. Press 'q' to exit.")
            while True:
                frame = capture_frame()
                if frame is None:
                    print("Failed to capture frame.")
                    break
                
                block_data, overlay = find_biggest_block(frame)
                
                if block_data:
                    print(f"\rFound: {block_data['color']} block, Area: {block_data['area']:.0f}  ", end="")

                cv2.imshow("Camera Test", overlay)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()
