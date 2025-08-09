# src/obstacle_challenge/main.py
import time
import sys
import traceback
import cv2
import numpy as np

# Import our hardware and logic modules
from src.obstacle_challenge import config
from src.motors import motor, servo
from src.sensors import camera, bno055, vl53l1x
from src.obstacle_challenge.utils import FPSCounter, SafetyMonitor

# --- Helper Functions ---
def get_angular_difference(angle1, angle2):
    if angle1 is None or angle2 is None: return 360
    diff = angle1 - angle2
    while diff <= -180: diff += 360
    while diff > 180: diff -= 360
    return abs(diff)

def steer_with_gyro(current_heading_goal, current_yaw):
    if config.GYRO_ENABLED and current_heading_goal is not None and current_yaw is not None:
        error = current_heading_goal - current_yaw
        while error <= -180: error += 360
        while error > 180: error -= 360
        steer = config.GYRO_KP * error
        servo.set_angle(steer)
        return steer
    servo.set_angle(0.0)
    return 0.0

# --- Initialization and Cleanup ---
safety_monitor = None
def initialize_all():
    global safety_monitor
    print("--- Initializing All Systems ---")
    if not all([motor.initialize(), servo.initialize(), camera.initialize(), bno055.initialize(), vl53l1x.initialize()]):
        return False
    
    if config.GYRO_ENABLED:
        safety_monitor = SafetyMonitor(bno055.sensor, config.TILT_THRESHOLD_DEGREES)
        
    time.sleep(0.5)
    print("--- System Initialization Complete ---")
    return True

def cleanup_all():
    print("\n--- Cleaning up All Systems ---")
    if safety_monitor: safety_monitor.stop()
    motor.cleanup(); servo.cleanup(); camera.cleanup(); bno055.cleanup(); vl53l1x.cleanup()
    print("--- Cleanup Complete ---")

# ==============================================================================
# --- MAIN EXECUTION LOGIC ---
# ==============================================================================
if __name__ == "__main__":
    if not initialize_all():
        cleanup_all(); sys.exit(1)

    INITIAL_HEADING = bno055.get_heading()
    if INITIAL_HEADING is None:
        print("WARNING: Gyro failed to get initial read, retrying..."); time.sleep(1)
        INITIAL_HEADING = bno055.get_heading()
        if INITIAL_HEADING is None: INITIAL_HEADING = 0.0
    print(f"INFO: Initial heading locked at: {INITIAL_HEADING:.2f}°")
    
    current_state = "DRIVING_STRAIGHT"
    target_heading = INITIAL_HEADING
    maneuver_base_heading = INITIAL_HEADING
    maneuver_color = None
    turn_counter = 0
    block_avoidance_complete = False
    frames_in_state = 0
    fps_counter = FPSCounter().start()
    # NEW: Mission timer start
    start_time = time.monotonic()

    print(f"\n[STATE CHANGE] ==> {current_state}")
    try:
        cv2.imshow("Robot View", np.zeros((config.FRAME_HEIGHT, config.FRAME_WIDTH, 3), dtype=np.uint8))
        cv2.waitKey(100)

        while True:
            if safety_monitor and safety_monitor.is_triggered():
                print("\nINFO: Safety Monitor triggered stop.")
                break

            # NEW: Calculate elapsed time on each loop
            elapsed_time = time.monotonic() - start_time

            frame = camera.capture_frame()
            if frame is None: break
            
            if not block_avoidance_complete:
                block_data, overlay_frame, _ = camera.find_biggest_block(frame)
            else:
                crop_y = int(frame.shape[0] * config.CROP_TOP_FRAC)
                overlay_frame = frame[crop_y:, :]

            current_yaw = bno055.get_heading()
            
            if current_state == "DRIVING_STRAIGHT":
                steer = steer_with_gyro(target_heading, current_yaw); motor.forward(config.DRIVE_SPEED)
                forward_dist = vl53l1x.get_distance(config.TOF_FORWARD_SENSOR_CHANNEL)
                
                dist_str = f"{forward_dist:.1f} mm" if forward_dist is not None else "N/A"
                # MODIFIED: Added timer to printout
                print(f"\rT:{elapsed_time:5.1f}s | ACTION: DRIVING | Wall:{dist_str} | Turns:{turn_counter}/{config.TOTAL_TURNS} ", end="")
                
                if block_data and block_data['area'] > config.MIN_BLOCK_AREA_FOR_ACTION:
                    maneuver_color = block_data['color']
                    maneuver_base_heading = target_heading
                    if maneuver_color == 'red': target_heading = (maneuver_base_heading + config.MANEUVER_ANGLE_DEG) % 360
                    else: target_heading = (maneuver_base_heading - config.MANEUVER_ANGLE_DEG + 360) % 360
                    frames_in_state = 0; current_state = "MANEUVER_TURN_1"
                    print(f"\n[STATE CHANGE] ==> {current_state} for {maneuver_color.upper()}.")
                
                elif forward_dist is not None and forward_dist < config.TOF_CORNERING_THRESHOLD_MM:
                    # We only turn if we haven't completed all laps yet
                    if turn_counter < config.TOTAL_TURNS:
                        frames_in_state = 0
                        current_state = "PERFORMING_CORNER_TURN"
                        print(f"\n[STATE CHANGE] ==> {current_state} (Turn {turn_counter + 1})")
                    else:
                        # If 12 turns are done and we see a wall, something is wrong. Stop.
                        print("\nINFO: 12 turns complete but wall detected. Stopping.")
                        current_state = "MISSION_COMPLETE"

            elif current_state == "MANEUVER_TURN_1":
                frames_in_state += 1; steer = steer_with_gyro(target_heading, current_yaw); motor.forward(config.DRIVE_SPEED)
                print(f"\rT:{elapsed_time:5.1f}s | ACTION: MANEUVER 1/8 (TURN)", end="")
                heading_is_correct = get_angular_difference(current_yaw, target_heading) < config.HEADING_LOCK_TOLERANCE
                if frames_in_state > config.MIN_FRAMES_IN_TURN and heading_is_correct:
                    frames_in_state = 0; current_state = "MANEUVER_DRIVE_1"
            
            elif current_state == "MANEUVER_DRIVE_1":
                frames_in_state += 1; steer = steer_with_gyro(target_heading, current_yaw); motor.forward(config.DRIVE_SPEED)
                print(f"\rT:{elapsed_time:5.1f}s | ACTION: MANEUVER 2/8 (DRIVE)", end="")
                if frames_in_state > config.MANEUVER_DRIVE_FRAMES_SIDEWAYS:
                    target_heading = maneuver_base_heading
                    frames_in_state = 0; current_state = "MANEUVER_TURN_2"

            elif current_state == "MANEUVER_TURN_2":
                frames_in_state += 1; steer = steer_with_gyro(target_heading, current_yaw); motor.forward(config.DRIVE_SPEED)
                print(f"\rT:{elapsed_time:5.1f}s | ACTION: MANEUVER 3/8 (TURN)", end="")
                heading_is_correct = get_angular_difference(current_yaw, target_heading) < config.HEADING_LOCK_TOLERANCE
                if frames_in_state > config.MIN_FRAMES_IN_TURN and heading_is_correct:
                    frames_in_state = 0; current_state = "MANEUVER_DRIVE_2"

            elif current_state == "MANEUVER_DRIVE_2":
                frames_in_state += 1; steer = steer_with_gyro(target_heading, current_yaw); motor.forward(config.DRIVE_SPEED)
                print(f"\rT:{elapsed_time:5.1f}s | ACTION: MANEUVER 4/8 (DRIVE)", end="")
                if frames_in_state > config.MANEUVER_DRIVE_FRAMES_STRAIGHT:
                    if maneuver_color == 'red': target_heading = (maneuver_base_heading - config.MANEUVER_ANGLE_DEG + 360) % 360
                    else: target_heading = (maneuver_base_heading + config.MANEUVER_ANGLE_DEG) % 360
                    frames_in_state = 0; current_state = "MANEUVER_TURN_3"

            elif current_state == "MANEUVER_TURN_3":
                frames_in_state += 1; steer = steer_with_gyro(target_heading, current_yaw); motor.forward(config.DRIVE_SPEED)
                print(f"\rT:{elapsed_time:5.1f}s | ACTION: MANEUVER 5/8 (TURN)", end="")
                heading_is_correct = get_angular_difference(current_yaw, target_heading) < config.HEADING_LOCK_TOLERANCE
                if frames_in_state > config.MIN_FRAMES_IN_TURN and heading_is_correct:
                    frames_in_state = 0; current_state = "MANEUVER_DRIVE_3"

            elif current_state == "MANEUVER_DRIVE_3":
                frames_in_state += 1; steer = steer_with_gyro(target_heading, current_yaw); motor.forward(config.DRIVE_SPEED)
                print(f"\rT:{elapsed_time:5.1f}s | ACTION: MANEUVER 6/8 (DRIVE)", end="")
                if frames_in_state > config.MANEUVER_DRIVE_FRAMES_SIDEWAYS:
                    target_heading = maneuver_base_heading
                    frames_in_state = 0; current_state = "MANEUVER_TURN_4"

            elif current_state == "MANEUVER_TURN_4":
                frames_in_state += 1; steer = steer_with_gyro(target_heading, current_yaw); motor.forward(config.DRIVE_SPEED)
                print(f"\rT:{elapsed_time:5.1f}s | ACTION: MANEUVER 7/8 (TURN)", end="")
                heading_is_correct = get_angular_difference(current_yaw, target_heading) < config.HEADING_LOCK_TOLERANCE
                if frames_in_state > config.MIN_FRAMES_IN_TURN and heading_is_correct:
                    # MODIFIED: New end-game logic
                    if turn_counter >= config.TOTAL_TURNS:
                        print("\nINFO: Final obstacle passed. Mission complete.")
                        current_state = "MISSION_COMPLETE"
                    else:
                        frames_in_state = 0
                        current_state = "DRIVING_STRAIGHT"
                        print(f"\n[STATE CHANGE] ==> Maneuver complete, returning to {current_state}")

            elif current_state == "PERFORMING_CORNER_TURN":
                frames_in_state += 1
                if frames_in_state == 1:
                    target_heading = (target_heading - 90 + 360) % 360
                    print(f"\nACTION: CORNER TURN | New Target Heading: {target_heading:.1f}°")
                servo.set_angle(45); motor.reverse(100) # Using your tuned angle
                diff = get_angular_difference(current_yaw, target_heading)
                print(f"\rT:{elapsed_time:5.1f}s | Current Yaw: {current_yaw:6.1f}° | Remaining: {diff:6.1f}°", end="")
                if abs(diff) < config.HEADING_LOCK_TOLERANCE:
                    turn_counter += 1
                    frames_in_state = 0
                    current_state = "DRIVING_STRAIGHT"
                    print(f"\n[STATE CHANGE] ==> {current_state} | Turns completed: {turn_counter}/{config.TOTAL_TURNS}")
            
            # REMOVED: The FINAL_APPROACH and PERFORMING_REVERSE_TURN states are no longer needed.

            elif current_state == "MISSION_COMPLETE":
                motor.brake(); servo.set_angle(0)
                print(f"\nMission complete. Final Time: {elapsed_time:.2f} seconds.")
                break

            fps_counter.update(); fps_counter.display_on_frame(overlay_frame)
            cv2.imshow("Robot View", overlay_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nINFO: 'q' pressed, exiting."); break

    except KeyboardInterrupt: print("\nINFO: Keyboard interrupt detected. Stopping.")
    except Exception as e: print(f"\nFATAL: An error occurred in main loop: {e}"); traceback.print_exc()
    finally: cleanup_all()
