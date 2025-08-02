# src/open_challenge/config.py
# Configuration for the Open Challenge (lap-based wall following)

# --- Behavior Constants ---
DRIVE_SPEED = 100             # Constant forward speed (0-100)
TARGET_DISTANCE = 200        # Desired distance from wall in mm
KP = 0.3                    # Proportional gain for steering
TOTAL_TURNS = 12             # Total corner turns to complete before final run
FINAL_FOLLOW_DURATION_S = 2.0 # Duration of the final wall-follow in seconds

# --- Hardware & Sensor Channels ---
LEFT_SENSOR_CHANNEL = 0
RIGHT_SENSOR_CHANNEL = 2

# --- Gyroscope Configuration ---
GYRO_ENABLED = True
GYRO_KP = 2.5                # Proportional gain for gyro-based steering
HEADING_LOCK_TOLERANCE = 5.0 # Degrees of tolerance for completing a turn
# NEW: The maximum angle the servo can be set to by the gyro logic.
GYRO_LEFT_MAX_STEER_ANGLE = 25.0
GYRO_RIGHT_MAX_STEER_ANGLE = 45.0
