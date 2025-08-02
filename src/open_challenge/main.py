# src/open_challenge/main.py
# New state-machine version for the Open Challenge.
# Implements gyro-stabilized start and longer, safer debugging pauses.

import time
import sys

import numpy as np

# Import our hardware and logic modules
from src.motors import motor, servo
from src.sensors import vl53l1x, bno055
from src.open_challenge import config
from src.obstacle_challenge.utils import SafetyMonitor


# --- Helper Functions ---
def get_angular_difference(angle1, angle2):
    if angle1 is None or angle2 is None: return 360
    diff = angle1 - angle2
    while diff <= -180: diff += 360
    while diff > 180: diff -= 360
    return abs(diff)

def steer_with_gyro(current_heading_goal, current_yaw, max_angle_left, max_angle_right):
    """
    Steers the robot using the gyro.
    Applies separate maximum steering angles for left and right turns.
    """
    if config.GYRO_ENABLED and current_heading_goal is not None and current_yaw is not None:
        error = current_heading_goal - current_yaw
        while error <= -180: error += 360
        while error > 180: error -= 360
        
        steer = config.GYRO_KP * error
        
        # NEW: Asymmetrical clamping
        # A negative steer value is a left turn, a positive value is a right turn.
        # This ensures the steer value is between -max_angle_left and +max_angle_right.
        steer = max(-max_angle_left, min(steer, max_angle_right))
        
        servo.set_angle(steer)
        return steer
        
    servo.set_angle(0.0)
    return 0.0

# --- Initialization ---
try:
    print("--- Initializing Systems for Open Challenge ---")
    servo.initialize()
    motor.initialize()
    vl53l1x.initialize()
    bno055.initialize()
    safety_monitor = SafetyMonitor(bno055.sensor, 15)
    print("--- Initialization Complete ---")

    print("Waiting 1 second for sensors to stabilize...")
    time.sleep(1.0)

except Exception as e:
    print(f"FATAL: An error occurred during initialization: {e}")
    motor.cleanup(); servo.cleanup(); vl53l1x.cleanup(); bno055.cleanup()
    exit()

# --- Main Execution Logic ---
# State Machine Variables
current_state = "STARTING_RUN"
turn_counter = 0
locked_turn_direction = None
target_heading = bno055.get_heading()
final_run_end_time = 0
frames_in_state = 0
max_sensor_threshold=700

print(f"\n[STATE CHANGE] ==> {current_state}")
motor.forward(config.DRIVE_SPEED)

try:
    while True:
        if safety_monitor and safety_monitor.is_triggered():
            print("\nINFO: Safety Monitor triggered stop.")
            break
        current_yaw = bno055.get_heading()
        dist_left = vl53l1x.get_distance(config.LEFT_SENSOR_CHANNEL)
        dist_right = vl53l1x.get_distance(config.RIGHT_SENSOR_CHANNEL)
        if dist_left is not None:
            if dist_left > max_sensor_threshold:dist_left=None
        if dist_right is not None:
            if dist_right > max_sensor_threshold:dist_right=None

        if current_state == "STARTING_RUN":
            steer_with_gyro(target_heading, current_yaw, config.GYRO_LEFT_MAX_STEER_ANGLE,config.GYRO_RIGHT_MAX_STEER_ANGLE)
            print(f"\rDriving straight with gyro, waiting for a wall to be lost...", end="")
            
            wall_lost = False
            if dist_left is None:
                locked_turn_direction = 'left'
                print(f"\nINFO: LEFT wall lost first. Locking to LEFT turns.")
                wall_lost = True

            elif dist_right is None:
                locked_turn_direction = 'right'
                print(f"\nINFO: RIGHT wall lost first. Locking to RIGHT turns.")
                wall_lost = True

            if wall_lost:
                
                current_state = "PERFORMING_TURN"
                max_sensor_threshold=300
                print(f"[STATE CHANGE] ==> {current_state}")
                frames_in_state = 0

        elif current_state == "WALL_FOLLOWING":
            wall_lost = False
            if locked_turn_direction == "left":
                if dist_left is not None:
                    # CORRECTED LOGIC FOR LEFT WALL
                    error = dist_left - config.TARGET_DISTANCE
                    steer_angle = -error * config.KP
                    servo.set_angle(steer_angle)
                    print(f"\rFollowing LEFT wall. Dist: {dist_left:6.1f}mm | Steer: {steer_angle:5.1f}", end="")
                else:
                    wall_lost = True

            elif locked_turn_direction == "right":
                if dist_right is not None:
                    # CORRECTED LOGIC FOR RIGHT WALL
                    error = dist_right - config.TARGET_DISTANCE
                    steer_angle = error * config.KP # Note: a positive error requires a positive (right) turn
                    servo.set_angle(steer_angle)
                    print(f"\rFollowing RIGHT wall. Dist: {dist_right:6.1f}mm | Steer: {steer_angle:5.1f}", end="")
                else:
                    wall_lost = True

            if wall_lost:
                print(f"\nINFO: {locked_turn_direction.upper()} wall lost. Preparing to turn.")
                frames_in_state = 0
                current_state = "PERFORMING_TURN"

        elif current_state == "PERFORMING_TURN":
            frames_in_state += 1
            if frames_in_state == 1:
                if locked_turn_direction == "left":
                    target_heading = (target_heading - 90 + 360) % 360
                else: # right
                    target_heading = (target_heading + 90 + 360) % 360
                print(f"[STATE CHANGE] ==> {current_state} | New Target: {target_heading:.1f}°")

            steer_with_gyro(target_heading, current_yaw, config.GYRO_LEFT_MAX_STEER_ANGLE,config.GYRO_RIGHT_MAX_STEER_ANGLE)
            
            if get_angular_difference(current_yaw, target_heading) < config.HEADING_LOCK_TOLERANCE:
                turn_counter += 1
                print(f"\nINFO: Turn {turn_counter}/{config.TOTAL_TURNS} complete.")

                if turn_counter >= config.TOTAL_TURNS:
                    current_state = "FINAL_WALL_FOLLOW"
                    print(f"[STATE CHANGE] ==> {current_state}")
                else:
                    current_state = "SEARCHING_FOR_WALL"
                    print(f"[STATE CHANGE] ==> {current_state}")
                frames_in_state = 0

        elif current_state == "SEARCHING_FOR_WALL":
            steer_with_gyro(target_heading, current_yaw, config.GYRO_LEFT_MAX_STEER_ANGLE,config.GYRO_RIGHT_MAX_STEER_ANGLE)
            print("\rSearching for next wall...", end="")

            sensor_to_check = dist_left if locked_turn_direction == "left" else dist_right
            if sensor_to_check is not None:
                print("\nINFO: Next wall found.")

                current_state = "WALL_FOLLOWING"
                print(f"[STATE CHANGE] ==> {current_state}")

        elif current_state == "FINAL_WALL_FOLLOW":
            if frames_in_state == 0:
                final_run_end_time = time.time() + config.FINAL_FOLLOW_DURATION_S
                print(f"Beginning final {config.FINAL_FOLLOW_DURATION_S}s wall-follow.")
            frames_in_state += 1

            if time.time() >= final_run_end_time:
                current_state = "MISSION_COMPLETE"
                print("\nINFO: Final run timer complete.")
                continue

            if locked_turn_direction == "left":
                error = dist_left - config.TARGET_DISTANCE if dist_left is not None else 0
                servo.set_angle(-error * config.KP)
            else:
                error = dist_right - config.TARGET_DISTANCE if dist_right is not None else 0
                servo.set_angle(error * config.KP)
            
            remaining_time = final_run_end_time - time.time()
            print(f"\rFinal run... Time remaining: {remaining_time:.1f}s", end="")

        elif current_state == "MISSION_COMPLETE":
            print("\nMission complete. Stopping robot.")
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nSTOP: Program interrupted by user.")

finally:
    print("Stopping robot and cleaning up resources.")
    motor.cleanup()
    servo.cleanup()
    vl53l1x.cleanup()
    bno055.cleanup()
