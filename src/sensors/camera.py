from picamera2 import Picamera2
import cv2
import numpy as np
import time
from src.obstacle_challenge import config


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
                "format": "XRGB8888",
                "size": (config.FRAME_WIDTH, config.FRAME_HEIGHT),
            },
            controls={"FrameRate": config.MAX_FPS},
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
        if frame.shape[2] == 4:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        return frame
    return None


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
