# src/open_challenge/config.py
# Configuration for the Open Challenge (lap-based wall following)

# --- Safety Configuration ---
TILT_THRESHOLD_DEGREES = 15.0

# --- Behavior Constants ---
DRIVE_SPEED = 100
TARGET_DISTANCE = 200
KP = 0.25
TOTAL_TURNS = 12
FINAL_FOLLOW_DURATION_S = 2.0
# NEW: Threshold to detect if the robot starts too close to a wall.
START_CLOSE_WALL_THRESHOLD = 70 # in mm

# --- Hardware & Sensor Channels ---
LEFT_SENSOR_CHANNEL = 0
RIGHT_SENSOR_CHANNEL = 2

# --- Gyroscope Configuration ---
GYRO_ENABLED = True
GYRO_KP = 2.5
HEADING_LOCK_TOLERANCE = 5.0
GYRO_MAX_STEER_LEFT = 25.0
GYRO_MAX_STEER_RIGHT = 45.0
