import time
import sys
import traceback
from gpiozero import Button

from src.motors import motor, servo
from src.sensors import bno055, vl53l1x
from src.obstacle_challenge.utils import SafetyMonitor

# ==============================================================================
# --- MANEUVER CONFIGURATION ---
# ==============================================================================
MANEUVER_SEQUENCE = [
    # Stage 1: Go straight back for 10 frames.
    {'type': 'drive', 'target_heading': 0.0, 'servo_angle': 0, 'unlimited_servo': False, 'drive_direction': 'reverse', 'speed': 60, 'duration_frames': 20},
    
    # Stage 2: Turn to 45 degrees while driving FORWARD.
    {'type': 'turn', 'target_heading': 45.0,  'servo_angle': 45, 'unlimited_servo': False, 'drive_direction': 'forward', 'speed': 80, 'duration_frames': 0},
    
    # Stage 3: Go back for 60 frames while maintaining the 45-degree heading.
    {'type': 'drive', 'target_heading': 45.0,   'servo_angle': 0,  'unlimited_servo': False, 'drive_direction': 'reverse', 'speed': 80, 'duration_frames': 60},
    
    # Stage 4: Turn from 45 down to 15 degrees while driving BACKWARD.
    {'type': 'turn', 'target_heading': 15.0,   'servo_angle': 60, 'unlimited_servo': True,  'drive_direction': 'reverse', 'speed': 80, 'duration_frames': 0},
    
    # Stage 5: Turn from 15 back to 0 degrees while driving FORWARD to straighten out.
    {'type': 'turn', 'target_heading': 0.0,   'servo_angle': -40,  'unlimited_servo': False, 'drive_direction': 'forward', 'speed': 90, 'duration_frames': 0},
]
# ==============================================================================

# --- Hardcoded System Constants ---
DRIVE_SPEED = 80
HEADING_LOCK_TOLERANCE = 5.0
TILT_THRESHOLD_DEGREES = 15.0
BUTTON_PIN = 23

# Constants for the preliminary sequence
WALL_APPROACH_THRESHOLD_MM = 55
POSITIONING_DRIVE_FRAMES = 140
SCAN_THRESHOLD_MM = 50
FRONT_SENSOR_CHANNEL = 1
LEFT_SENSOR_CHANNEL = 0
kp=3


def get_angular_difference(angle1, angle2):
    """Calculates the shortest angle between two headings."""
    if angle1 is None or angle2 is None:
        return 360
    diff = angle1 - angle2
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return abs(diff)

def steer_with_gyro(current_heading_goal, current_yaw):
    if current_heading_goal is not None and current_yaw is not None:
        error = current_heading_goal - current_yaw
        while error <= -180: error += 360
        while error > 180: error -= 360
        steer = kp * error # Using a local KP for simplicity
        servo.set_angle(steer)
        return steer
    servo.set_angle(0.0)
    return 0.0


safety_monitor = None
start_button = None


def initialize_all():
    """Initializes all hardware components."""
    global safety_monitor, start_button
    print("--- Initializing All Systems for Maneuver Sequencer ---")
    if not all([motor.initialize(), servo.initialize(), bno055.initialize(), vl53l1x.initialize()]):
        return False
    safety_monitor = SafetyMonitor(bno055.sensor, TILT_THRESHOLD_DEGREES)
    start_button = Button(BUTTON_PIN)
    time.sleep(0.5)
    print("--- System Initialization Complete ---")
    return True


def cleanup_all():
    """Cleans up all hardware resources safely."""
    print("\n--- Cleaning up All Systems ---")
    if safety_monitor:
        safety_monitor.stop()
    motor.cleanup()
    servo.cleanup()
    bno055.cleanup()
    vl53l1x.cleanup()
    print("--- Cleanup Complete ---")


if __name__ == "__main__":
    if not initialize_all():
        cleanup_all()
        sys.exit(1)

    INITIAL_HEADING = 0.0
    print(f"INFO: Ready. Press button to set '0' heading and start.")

    current_state = "WAITING_FOR_START"
    stage_index = 0
    frames_in_state = 0
    start_time = 0
    target_heading = 0

    try:
        while True:
            if safety_monitor and safety_monitor.is_triggered():
                break

            elapsed_time = time.monotonic() - start_time if start_time > 0 else 0
            current_yaw = bno055.get_heading()

            if current_state == "WAITING_FOR_START":
                print("\rWaiting for start button press...", end="")
                if start_button.is_pressed:
                    new_heading = bno055.get_heading()
                    if new_heading is not None:
                        INITIAL_HEADING = new_heading
                        target_heading = INITIAL_HEADING
                        print(f"\n--- BUTTON PRESSED ---")
                        print(f"New '0' heading is: {INITIAL_HEADING:.2f}°")
                        start_time = time.monotonic()
                        current_state = "APPROACHING_WALL_1"
                        print(f"[STATE CHANGE] ==> {current_state}")
                    else:
                        print("\nError: Could not read gyro to set new heading.")
                    time.sleep(0.5)

            elif current_state == "APPROACHING_WALL_1":
                # MODIFIED: Use gyro to drive straight
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(DRIVE_SPEED)
                forward_dist = vl53l1x.get_distance(FRONT_SENSOR_CHANNEL)
                dist_str = f"{forward_dist:.1f}" if forward_dist is not None else "N/A"
                print(f"\rDriving forward, waiting for wall... Dist: {dist_str} mm", end="")
                if forward_dist is not None and forward_dist < WALL_APPROACH_THRESHOLD_MM:
                    frames_in_state = 0
                    current_state = "PERFORMING_TURN"
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            elif current_state == "PERFORMING_TURN":
                frames_in_state += 1
                if frames_in_state == 1:
                    target_heading = (INITIAL_HEADING + 90 + 360) % 360
                    print(f"ACTION: Performing 90-degree CW reverse turn. Target: {target_heading:.1f}°")
                servo.set_angle(-45)
                motor.reverse(DRIVE_SPEED)
                if get_angular_difference(current_yaw, target_heading) < HEADING_LOCK_TOLERANCE:
                    frames_in_state = 0
                    current_state = "DRIVE_FORWARD_POSITIONING"
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            elif current_state == "DRIVE_FORWARD_POSITIONING":
                frames_in_state += 1
                # MODIFIED: Use gyro to drive straight
                steer_with_gyro(target_heading-1, current_yaw)
                motor.forward(DRIVE_SPEED)
                print(f"\rDriving forward to scan position... Frames: {frames_in_state}/{POSITIONING_DRIVE_FRAMES}", end="")
                if frames_in_state > POSITIONING_DRIVE_FRAMES:
                    frames_in_state = 0
                    current_state = "WAITING_FOR_SCAN"
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            elif current_state == "WAITING_FOR_SCAN":
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(50)
                dist_left = vl53l1x.get_distance(LEFT_SENSOR_CHANNEL)
                dist_str = f"{dist_left:.1f}" if dist_left is not None else "N/A"
                print(f"\rScanning... Left Dist: {dist_str} mm", end="")
                if dist_left is not None and dist_left < SCAN_THRESHOLD_MM:
                    print(f"\nScan triggered at {dist_left:.1f}mm. Starting maneuver sequence.")
                    motor.brake()
                    time.sleep(0.5)
                    # THIS IS THE FIX: Set the new "0" reference for the maneuver.
                    maneuver_base_heading = target_heading
                    current_state = "RUNNING_MANEUVER"

            elif current_state == "RUNNING_MANEUVER":
                if stage_index >= len(MANEUVER_SEQUENCE):
                    current_state = "MISSION_COMPLETE"
                    continue

                stage = MANEUVER_SEQUENCE[stage_index]
                stage_type = stage['type']
                
                if stage_type == 'drive' and stage['target_heading'] == 0.0:
                    current_stage_target_heading = maneuver_base_heading
                else:
                    current_stage_target_heading = (maneuver_base_heading + stage['target_heading'] + 360) % 360

                if stage_type == 'turn':
                    frames_in_state += 1
                    if frames_in_state == 1:
                        print(f"\n[STAGE {stage_index + 1}] Starting TURN to {current_stage_target_heading:.1f}°")
                    
                    if stage['unlimited_servo']:
                        servo.set_angle_unlimited(stage['servo_angle'])
                    else:
                        servo.set_angle(stage['servo_angle'])

                    if stage['drive_direction'] == 'forward':
                        motor.forward(stage['speed'])
                    else:
                        motor.reverse(stage['speed'])

                    if get_angular_difference(current_yaw, current_stage_target_heading) < HEADING_LOCK_TOLERANCE:
                        print(f"--- Stage {stage_index + 1} complete. ---")
                        stage_index += 1
                        frames_in_state = 0
                
                elif stage_type == 'drive':
                    frames_in_state += 1
                    if frames_in_state == 1:
                        print(f"\n[STAGE {stage_index + 1}] Starting DRIVE for {stage['duration_frames']} frames.")

                    steer_with_gyro(current_stage_target_heading, current_yaw)
                    print('/n',current_stage_target_heading,current_yaw)

                    if stage['drive_direction'] == 'forward':
                        motor.forward(stage['speed'])
                    else:
                        kp=-3
                        motor.reverse(stage['speed'])
                    
                    print(f"\rDriving... Frames: {frames_in_state}/{stage['duration_frames']}", end="")

                    if frames_in_state > stage['duration_frames']:
                        print(f"\n--- Stage {stage_index + 1} complete. ---")
                        stage_index += 1
                        frames_in_state = 0

            elif current_state == "MISSION_COMPLETE":
                motor.brake()
                servo.set_angle(0)
                print(f"\nManeuver sequence complete. Final Time: {elapsed_time:.2f} seconds.")
                break

            time.sleep(0.02)

    except KeyboardInterrupt:
        print("\nINFO: Keyboard interrupt detected. Stopping.")
    except Exception as e:
        print(f"\nFATAL: An error occurred in main loop: {e}")
        traceback.print_exc()
    finally:
        cleanup_all()
