import cv2
import numpy as np
import os
from src.sensors import camera
from src.obstacle_challenge.config import LOWER_RED_1, LOWER_RED_2, UPPER_RED_1,UPPER_RED_2,LOWER_GREEN,UPPER_GREEN

LOWER_BLACK = np.array([0, 0, 0])
UPPER_BLACK = np.array([180, 255, 120])
LOWER_ORANGE = np.array([6, 70, 20])
UPPER_ORANGE = np.array([26, 255, 255])
LOWER_MAGENTA = np.array([158, 73, 64])
UPPER_MAGENTA = np.array([173, 255, 223])
LOWER_BLUE = np.array([94, 45, 58])
UPPER_BLUE = np.array([140, 226, 185])

FRAME_WIDTH, FRAME_HEIGHT = 640, 360
WALL_MIN_AREA, BLOCK_MIN_AREA, MAGENTA_MIN_AREA, CLOSE_BLOCK_MIN_AREA = 300, 500, 500, 15

rois = {
    'left': (0, 140, 135, 150), 'right': (505, 140, 135, 150),
    'inner_left': (140, 165, 100, 100), 'inner_right': (400, 165, 100, 100),
    'line': (280, 200, 80, 40), 'close_black_roi': (140, 120, 360, 10),
    'full_frame': (0, 100, 640, 165), 'close_block': (250, 230, 140, 10)
}

def create_roi_mask(shape, roi_rects):
    mask = np.zeros(shape, dtype="uint8")
    for x, y, w, h in roi_rects:
        cv2.rectangle(mask, (x, y), (x + w, y + h), 255, -1)
    return mask

roi_mask_walls = create_roi_mask((FRAME_HEIGHT, FRAME_WIDTH), [rois['left'], rois['right'], rois['inner_left'], rois['inner_right']])
roi_mask_main_blocks = create_roi_mask((FRAME_HEIGHT, FRAME_WIDTH), [rois['full_frame']])
roi_mask_close_blocks = create_roi_mask((FRAME_HEIGHT, FRAME_WIDTH), [rois['close_block']])
roi_mask_line = create_roi_mask((FRAME_HEIGHT, FRAME_WIDTH), [rois['line']])
roi_mask_magenta = create_roi_mask((FRAME_HEIGHT, FRAME_WIDTH), [rois['full_frame']])
roi_mask_close_black = create_roi_mask((FRAME_HEIGHT, FRAME_WIDTH), [rois['close_black_roi']])

def process_frame(frame):
    cv2.imwrite('01_original.png', frame)
    hsv = cv2.cvtColor(cv2.GaussianBlur(frame, (1, 7), 0), cv2.COLOR_BGR2HSV)
    cv2.imwrite('02_hsv.png', hsv)

    masks = {
        'black': cv2.inRange(hsv, LOWER_BLACK, UPPER_BLACK),
        'red': cv2.bitwise_or(cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1), cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)),
        'green': cv2.inRange(hsv, LOWER_GREEN, UPPER_GREEN),
        'orange': cv2.inRange(hsv, LOWER_ORANGE, UPPER_ORANGE),
        'magenta': cv2.inRange(hsv, LOWER_MAGENTA, UPPER_MAGENTA),
        'blue': cv2.inRange(hsv, LOWER_BLUE, UPPER_BLUE)
    }
    for name, mask in masks.items(): cv2.imwrite(f'03_mask_{name}.png', mask)
    
    pure_black = cv2.bitwise_and(masks['black'], cv2.bitwise_not(cv2.bitwise_or(masks['red'], masks['green'])))
    final_masks = {
        'walls': cv2.bitwise_and(pure_black, roi_mask_walls), 'main_red': cv2.bitwise_and(masks['red'], roi_mask_main_blocks),
        'main_green': cv2.bitwise_and(masks['green'], roi_mask_main_blocks), 'close_red': cv2.bitwise_and(masks['red'], roi_mask_close_blocks),
        'close_green': cv2.bitwise_and(masks['green'], roi_mask_close_blocks), 'orange': cv2.bitwise_and(masks['orange'], roi_mask_line),
        'blue': cv2.bitwise_and(masks['blue'], roi_mask_line), 'magenta': cv2.bitwise_and(masks['magenta'], roi_mask_magenta),
        'close_black': cv2.bitwise_and(cv2.bitwise_or(pure_black, masks['magenta']), roi_mask_close_black), 'close_magenta': cv2.bitwise_and(masks['magenta'], roi_mask_close_blocks)
    }
    for name, mask in final_masks.items(): cv2.imwrite(f'04_final_mask_{name}.png', mask)

    annotated_frame = frame.copy()
    contour_params = [
        (final_masks['magenta'], (255, 0, 255), MAGENTA_MIN_AREA), (final_masks['main_red'], (0, 0, 255), BLOCK_MIN_AREA),
        (final_masks['main_green'], (0, 255, 0), BLOCK_MIN_AREA), (final_masks['close_red'], (0, 0, 255), CLOSE_BLOCK_MIN_AREA),
        (final_masks['close_green'], (0, 255, 0), CLOSE_BLOCK_MIN_AREA), (final_masks['close_magenta'], (255, 0, 255), CLOSE_BLOCK_MIN_AREA),
        (final_masks['orange'], (0, 165, 255), 20), (final_masks['blue'], (255, 0, 0), 20),
        (final_masks['close_black'], (50, 50, 50), WALL_MIN_AREA), (final_masks['walls'], (0, 0, 0), WALL_MIN_AREA)
    ]
    for mask, color, min_area in contour_params:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) > min_area: cv2.drawContours(annotated_frame, [c], -1, color, 2)
                
    cv2.imwrite('05_final_annotated.png', annotated_frame)
    print("Processing complete. Images saved in the current directory.")

if __name__ == "__main__":
    try:
        camera.initialize()
        frame = camera.capture_frame()
        if frame is not None:
            process_frame(frame)
        else:
            print("Error: Failed to capture frame from camera module.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        camera.cleanup()
