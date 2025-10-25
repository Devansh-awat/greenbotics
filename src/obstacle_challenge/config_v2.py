# config_v2.py
import numpy as np

# --- Frame Configuration ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 360
FRAME_MIDPOINT_X = FRAME_WIDTH // 2

# --- Regions of Interest (ROIs) as Tuples (x, y, w, h) ---
ROI_LEFT = (0, 60, 100, 290)
ROI_RIGHT = (540, 60, 100, 260)
ROI_INNER_LEFT = (140, 150, 60, 270)
ROI_INNER_RIGHT = (440, 165, 60, 270)
ROI_ORANGE = (280, 80, 80, 20)
ROI_BLOCKS_MAIN = (0, 70, 640, 130)

# Dictionary to group wall ROIs for easier iteration
WALL_ROIS = {
    'wall_left': ROI_LEFT,
    'wall_right': ROI_RIGHT,
    'wall_inner_left': ROI_INNER_LEFT,
    'wall_inner_right': ROI_INNER_RIGHT,
}


# --- Color Ranges in LAB Color Space ---
LOWER_BLACK_LAB = np.array([0, 0, 0])
UPPER_BLACK_LAB = np.array([90, 255, 255])

LOWER_ORANGE_LAB = np.array([57, 103, 128])
UPPER_ORANGE_LAB = np.array([186, 186, 172])

LOWER_RED_LAB = np.array([0, 150, 150])
UPPER_RED_LAB = np.array([255, 255, 255])

LOWER_GREEN_LAB = np.array([0, 0, 130])
UPPER_GREEN_LAB = np.array([255, 120, 255])

# --- Detection Parameters ---
WALL_MIN_AREA = 300
BLOCK_MIN_AREA = 500
ORANGE_MIN_AREA = 20