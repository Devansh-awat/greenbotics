from picamera2 import Picamera2
import cv2
import numpy as np
import time
from src.obstacle_challenge import config
import libcamera


picam2 = None


current_mouse_x = -1
current_mouse_y = -1


def mouse_event_handler(event, x, y, flags, param):
    """Callback function for mouse events to update mouse coordinates."""
    global current_mouse_x, current_mouse_y
    if event == cv2.EVENT_MOUSEMOVE:
        current_mouse_x = x
        current_mouse_y = y


def initialize():
    """Initializes the Picamera2."""
    global picam2
    try:
        picam2 = Picamera2()
        cam_config = picam2.create_preview_configuration(
            main={
                "format": "RGB888",
                "size": (2304, 1296),
            },
            lores={"size": (640, 360), "format": "RGB888"},

            controls={
                        #"AeEnable": False,
                        "ExposureTime": 9000,
                        #"AnalogueGain": 8.0,
                        "FrameRate": 56,
                    }
        )
        picam2.configure(cam_config)
        picam2.start()
        time.sleep(1.0)
        #picam2.set_controls(controls={'ExposureTime': 20000, 'AnalogueGain': 1})
        print("INFO: Camera Initialized.")
        return True
    except Exception as e:
        print(f"FATAL: Camera failed to initialize: {e}")
        return False


def capture_frame():
    """Captures and returns a single frame from the camera."""
    if picam2:
        frame = picam2.capture_array("lores")
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        #if frame.shape[2] == 4:
            #frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame
    return None

def find_objects_in_rois(frame, detection_jobs):
    """
    Processes a frame to find objects based on a list of detection jobs.

    Each job defines an ROI, colors to detect, and rules for returning contours.
    This version can optionally return the binary mask images for debugging.

    Args:
        frame (np.ndarray): The input camera frame.
        detection_jobs (list): A list of dictionaries, where each dict is a job.
            Each job's 'params' can now include:
            - 'return_mask': (optional) bool. If True, the function will return
                             the black and white mask for this job's colors.

    Returns:
        tuple: A tuple containing three items:
            - list: A list of dictionaries with info about each final detected object.
            - np.ndarray: The frame with ROIs and contours drawn on it.
            - dict: A dictionary where keys are strings like 'job_0_red' and values
                    are the corresponding binary mask images (np.ndarray).
    """
    overlay_frame = frame.copy()
    final_detected_objects = []
    returned_masks = {}  # New dictionary to store requested masks
    kernel = np.ones((5, 5), np.uint8)

    # Use enumerate to get the index of the job for unique mask keys
    for job_index, job in enumerate(detection_jobs):
        x, y, w, h = job['roi']
        hsv_roi = frame[y:y+h, x:x+w]

        # Check if this job requests its mask to be returned
        should_return_mask = job['params'].get('return_mask', False)

        job_detected_objects = []
        morph_kernel = job['params'].get('morph_kernel', kernel)

        colors_to_process = {}
        for color_info in job['colors']:
            name = color_info['name']
            if name not in colors_to_process:
                colors_to_process[name] = []
            colors_to_process[name].append(color_info)

        for color_name, color_defs in colors_to_process.items():
            combined_mask = np.zeros(hsv_roi.shape[:2], dtype=np.uint8)
            for color_def in color_defs:
                mask = cv2.inRange(hsv_roi, color_def['lower'], color_def['upper'])
                combined_mask = cv2.bitwise_or(combined_mask, mask)
            if color_name == 'black':
                red_1 = cv2.inRange(hsv_roi, config.LOWER_RED_1, config.UPPER_RED_1)
                red_2 = cv2.inRange(hsv_roi, config.LOWER_RED_2, config.UPPER_RED_2)
                red = cv2.bitwise_or(red_1, red_2)
                green = cv2.inRange(hsv_roi, config.LOWER_GREEN, config.UPPER_GREEN)
                combined_mask = cv2.subtract(combined_mask, red)
                combined_mask = cv2.subtract(combined_mask, green)
            cleaned_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, morph_kernel)

            if should_return_mask:
                mask_key = f"job_{job_index}_{color_name}"
                returned_masks[mask_key] = cleaned_mask

            contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > job['params'].get('min_area', 100):
                    M = cv2.moments(cnt)
                    cx = int(M["m10"] / M["m00"]) if M["m00"] != 0 else 0
                    cy = int(M["m01"] / M["m00"]) if M["m00"] != 0 else 0

                    job_detected_objects.append({
                        "contour": cnt, "color": color_name, "area": area,
                        "centroid": (cx + x, cy + y),
                        "bounding_rect": cv2.boundingRect(cnt),
                        "roi_origin": (x, y),
                        "draw_color_hsv": np.uint8([[np.mean([color_defs[0]['lower'], color_defs[0]['upper']], axis=0)]]),
                        "type": job.get('type', 'unknown')
                    })

        if not job_detected_objects:
            continue

        rule = job['params']['return_rule']
        if rule == 'biggest_in_job':
            biggest = max(job_detected_objects, key=lambda o: o['area'])
            final_detected_objects.append(biggest)
        elif rule == 'biggest_per_color':
            objects_by_color = {}
            for obj in job_detected_objects:
                color = obj['color']
                if color not in objects_by_color or obj['area'] > objects_by_color[color]['area']:
                    objects_by_color[color] = obj
            final_detected_objects.extend(list(objects_by_color.values()))
        elif rule == 'all':
            final_detected_objects.extend(job_detected_objects)

    for obj in final_detected_objects:
        roi_x, roi_y = obj['roi_origin']
        offset_contour = obj['contour'].copy()
        offset_contour[:, :, 0] += roi_x
        offset_contour[:, :, 1] += roi_y
        
        draw_color_bgr = cv2.cvtColor(obj['draw_color_hsv'], cv2.COLOR_HSV2BGR)[0][0]
        box_color = tuple(map(int, draw_color_bgr))

        cv2.drawContours(overlay_frame, [offset_contour], -1, box_color, 3)
        rx, ry, _, _ = obj['bounding_rect']
        cv2.putText(overlay_frame, str(obj['centroid'][0]), (rx + roi_x, ry + roi_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, box_color, 2)
    
    return final_detected_objects, overlay_frame, returned_masks

def find_biggest_block(frame):
    """
    Processes a cropped region of the frame to find the largest red or green block.
    Returns the cropped frame as the new overlay.
    """

    full_h, _, _ = frame.shape
    roi_top_y = int(full_h * config.CROP_TOP_FRAC)
    roi_bottom_y = int(full_h * (1.0 - config.CROP_BOTTOM_FRAC))

    cropped_frame = frame[roi_top_y:roi_bottom_y, :]

    h, w, _ = cropped_frame.shape
    total_screen_area = w * h
    max_allowed_area = total_screen_area * config.MAX_BLOCK_AREA_FRACTION

    hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)

    mask_r1 = cv2.inRange(hsv, config.LOWER_RED_1, config.UPPER_RED_1)
    mask_r2 = cv2.inRange(hsv, config.LOWER_RED_2, config.UPPER_RED_2)
    red_mask = cv2.bitwise_or(mask_r1, mask_r2)
    green_mask = cv2.inRange(hsv, config.LOWER_GREEN, config.UPPER_GREEN)

    kernel = np.ones((5, 5), np.uint8)
    red_mask_cleaned = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
    green_mask_cleaned = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)

    contours_red, _ = cv2.findContours(
        red_mask_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    contours_green, _ = cv2.findContours(
        green_mask_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    valid_blocks = []
    for contours, color in [(contours_red, "red"), (contours_green, "green")]:
        for cnt in contours:
            area = cv2.contourArea(cnt)

            if not (config.MIN_CONTOUR_AREA < area < max_allowed_area):
                continue

            x, y, cw, ch = cv2.boundingRect(cnt)
            if not ch > cw:
                continue

            M = cv2.moments(cnt)
            center_x = int(M["m10"] / M["m00"]) if M["m00"] != 0 else 0
            center_y = int(M["m01"] / M["m00"]) if M["m00"] != 0 else 0

            valid_blocks.append(
                {
                    "contour": cnt,
                    "color": color,
                    "area": area,
                    "centroid": (center_x, center_y),
                }
            )

    overlay_frame = cropped_frame

    if not valid_blocks:
        return None, overlay_frame, hsv

    largest_block = max(valid_blocks, key=lambda b: b["area"])
    x, y, cw, ch = cv2.boundingRect(largest_block["contour"])
    box_color = (
        config.BOX_COLOR_RED
        if largest_block["color"] == "red"
        else config.BOX_COLOR_GREEN
    )

    cv2.rectangle(overlay_frame, (x, y), (x + cw, y + ch), box_color, 2)
    cv2.putText(
        overlay_frame,
        largest_block["color"].upper(),
        (x, y - 10),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        box_color,
        2,
    )

    return largest_block, overlay_frame, hsv


def cleanup():
    """Stops the camera."""
    print("--- Cleaning up Camera ---")
    if picam2:
        picam2.stop()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print("--- Testing Camera and Vision Module with Cropping ---")
    if not initialize():
        print("Camera test failed during initialization.")
    else:
        cv2.namedWindow("Camera Test")
        cv2.setMouseCallback("Camera Test", mouse_event_handler)

        try:
            print(
                "Displaying CROPPED camera feed with block detection. Press 'q' to exit."
            )
            print("Move mouse over the frame to see pixel RGB and HSV values.")
            while True:
                full_frame = capture_frame()
                if full_frame is None:
                    print("Failed to capture frame.")
                    break

                block_data, overlay, hsv_frame = find_biggest_block(full_frame)

                if (
                    hsv_frame is not None
                    and 0 <= current_mouse_x < overlay.shape[1]
                    and 0 <= current_mouse_y < overlay.shape[0]
                ):
                    b, g, r = overlay[current_mouse_y, current_mouse_x]
                    h, s, v = hsv_frame[current_mouse_y, current_mouse_x]
                    text_rgb = f"RGB: ({r:3d}, {g:3d}, {b:3d})"
                    text_hsv = f"HSV: ({h:3d}, {s:3d}, {v:3d})"
                    cv2.putText(
                        overlay,
                        text_rgb,
                        (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )
                    cv2.putText(
                        overlay,
                        text_hsv,
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (255, 255, 255),
                        2,
                    )

                if block_data:
                    print(
                        f"\rFound: {block_data['color']} block, Area: {block_data['area']:.0f}, Centroid: {block_data['centroid']}  ",
                        end="",
                    )
                else:
                    print(
                        "\rNo block found.                                      ",
                        end="",
                    )

                cv2.imshow("Camera Test", overlay)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()
