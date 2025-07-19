# src/obstacle_challenge/config.py
# This file contains all the configuration and tuning parameters for the robot.
# Edit these values to change the robot's behavior.

import numpy as np

# --- Driving & Steering Configuration ---
DRIVE_SPEED = 70.0

# --- Vision & FOV Configuration ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
MAX_FPS = 30
CAMERA_H_FOV = 120.0
CROP_TOP_FRAC = 1 / 4
CROP_BOTTOM_FRAC = 1 / 4
MIN_CONTOUR_AREA = 2500
MAX_BLOCK_AREA_FRACTION = 0.05  # NEW: Ignore blocks larger than 25% of the screen
MIN_BLOCK_ROI_OVERLAP = 0.50  # NEW: Block must have 50% of its height inside the ROI

# --- Behavior Constants ---
MIN_BLOCK_AREA_FOR_ACTION = 7500
AVOID_ANGLE_DEG = 45.0
PEEK_AVOID_ANGLE_DEG = 15.0
CLEARANCE_FRAMES = 45
SCAN_DURATION_FRAMES = 15
SEARCH_DRIVE_FRAMES = 35
FORCE_CLEARANCE_FRAMES = (
    70  # NEW: Frames to drive forward before forcing a dead-end turn
)

# --- Dead-End Navigation Logic ---
DEAD_END_TURN_DIRECTION = "RIGHT"  # "RIGHT" or "LEFT"
DEAD_END_TURN_ANGLE = 90.0

# --- HSV Color Ranges (Tuned) ---
LOWER_RED_1 = np.array([0, 150, 80])
UPPER_RED_1 = np.array([10, 255, 255])
LOWER_RED_2 = np.array([170, 150, 80])
UPPER_RED_2 = np.array([180, 255, 255])
LOWER_GREEN = np.array([35, 60, 40])
UPPER_GREEN = np.array([90, 255, 255])

# --- Gyroscope Configuration ---
GYRO_ENABLED = True
GYRO_KP = 3.0
HEADING_LOCK_TOLERANCE = 5.0

# --- Time-of-Flight (ToF) Sensor Configuration ---
TOF_ENABLED = True
TOF_CHANNELS_TO_USE = range(0, 1, 2)  # Channels 0, 1, 2 on the mux
TOF_FORWARD_SENSOR_CHANNEL = 1  # The channel for the sensor facing forward
TOF_OBSTACLE_THRESHOLD_MM = 50  # Distance in mm to trigger dead-end logic

# --- Hardware Pin & PWM Configuration ---
AIN1_PIN = 26
AIN2_PIN = 13
STBY_PIN = 6
MOTOR_PWM_PIN = 19
MOTOR_PWM_FREQ = 10000
MOTOR_PWM_CHIP = 0
MOTOR_PWM_CHANNEL = 3
SERVO_GPIO = 18
SERVO_PWM_FREQ = 50
SERVO_PWM_CHIP = 0
SERVO_PWM_CHANNEL = 2
CALIBRATED_MIN_PW_S = 0.001
CALIBRATED_MAX_PW_S = 0.002
CALIBRATED_ANGLE_MIN = 45.0
CALIBRATED_ANGLE_MAX = 135.0
INPUT_ANGLE_MIN_SERVO = -45.0
INPUT_ANGLE_MAX_SERVO = 45.0
SAFETY_MIN_PW_S = 0.0005
SAFETY_MAX_PW_S = 0.0025
SERVO_PWM_PERIOD_S = 1.0 / SERVO_PWM_FREQ
BOX_COLOR_RED = (0, 0, 255)
BOX_COLOR_GREEN = (0, 255, 0)
BOX_COLOR_ROI = (255, 0, 255)
