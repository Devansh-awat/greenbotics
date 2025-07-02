# main.py
import time
import sys
import traceback

# Import our hardware and logic modules
import config
import motor
import servo
import camera
import bno055

# --- Helper Functions ---
def get_angular_difference(angle1, angle2):
    """Calculates the shortest difference between two angles in degrees."""
    if angle1 is None or angle2 is None: return 360 # Return large value if data is bad
    diff = angle1 - angle2
    while diff <= -180: diff += 360
    while diff > 180: diff -= 360
    return abs(diff)

def steer_with_gyro(current_heading_goal, current_yaw):
    """Steers the robot to a target heading using the gyro."""
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
def initialize_all():
    """Initializes all hardware modules."""
    print("--- Initializing All Systems ---")
    if not motor.initialize(): return False
    if not servo.initialize(): return False
    if not camera.initialize(): return False
    if not bno055.initialize(): return False
    print("--- System Initialization Complete ---")
    return True

def cleanup_all():
    """Cleans up all hardware modules."""
    print("\n--- Cleaning up All Systems ---")
    motor.cleanup()
    servo.cleanup()
    camera.cleanup()
    bno055.cleanup()
    print("--- Cleanup Complete ---")

# ==============================================================================
# --- MAIN EXECUTION LOGIC ---
# ==============================================================================
if __name__ == "__main__":
    if not initialize_all():
        cleanup_all()
        sys.exit(1)

    INITIAL_HEADING = bno055.get_initial_heading() if config.GYRO_ENABLED else 0.0
    
    # State Machine Variables
    current_state = "DRIVING_STRAIGHT"
    last_avoidance_direction = None
    dynamic_target_heading = INITIAL_HEADING
    frames_in_state = 0
    frames_since_block_lost = 0
    first_search_completed = False
    is_peek_avoidance = False

    print(f"\n[STATE CHANGE] ==> {current_state}")
    try:
        while True:
            # --- Per-Frame Setup ---
            frame = camera.capture_frame()
            if frame is None:
                print("ERROR: Failed to get frame, stopping.")
                break
            
            block_data, overlay_frame = camera.find_biggest_block(frame)
            current_yaw = bno055.get_heading()

            # --- Universal Override Logic ---
            if block_data and block_data['area'] > config.MIN_BLOCK_AREA_FOR_ACTION:
                if current_state not in ["INITIATING_TURN", "TURNING_WITH_GYRO", "CLEARING_OBSTACLE"]:
                    print(f"\n[OVERRIDE] Block is too close! Area: {block_data['area']:.0f}")
                    if current_state in ["SEARCHING_DRIVE", "APPROACHING_DURING_PEEK"]:
                        is_peek_avoidance = True
                    current_state = "INITIATING_TURN"
            
            # ===================================================
            # --- State Machine ---
            # ===================================================
            if current_state == "DRIVING_STRAIGHT":
                steer = steer_with_gyro(INITIAL_HEADING, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                print(f"\rACTION: DRIVING STRAIGHT | Steer: {steer:5.1f}°", end="")
                if block_data:
                    current_state = "APPROACHING_OBSTACLE"
                    print(f"\n[STATE CHANGE] ==> {current_state}")
            
            elif current_state == "APPROACHING_OBSTACLE":
                steer = steer_with_gyro(INITIAL_HEADING, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                area = block_data['area'] if block_data else 0
                print(f"\rACTION: APPROACHING ON MAIN PATH | Area: {area:.0f}/{config.MIN_BLOCK_AREA_FOR_ACTION}", end="")
                if not block_data:
                    current_state = "DRIVING_STRAIGHT"
                    print(f"\n[STATE CHANGE] ==> {current_state} (Lost Target)")

            elif current_state == "INITIATING_TURN":
                color = block_data['color']
                if is_peek_avoidance:
                    current_heading_at_decision = current_yaw if current_yaw is not None else dynamic_target_heading
                    if color == 'red':
                        dynamic_target_heading = (current_heading_at_decision + config.AVOID_ANGLE_DEG + 360) % 360
                    else: # Green
                        dynamic_target_heading = (current_heading_at_decision - config.AVOID_ANGLE_DEG + 360) % 360
                    print(f"\n[PEEK AVOIDANCE] Avoiding {color.upper()} relative to peek. New Goal: {dynamic_target_heading:.1f}°")
                    is_peek_avoidance = False
                else:
                    right_lane = (INITIAL_HEADING + config.AVOID_ANGLE_DEG + 360) % 360
                    left_lane = (INITIAL_HEADING - config.AVOID_ANGLE_DEG + 360) % 360
                    if color == 'red':
                        last_avoidance_direction = 'RIGHT'
                        dynamic_target_heading = right_lane
                    else: # Green
                        last_avoidance_direction = 'LEFT'
                        dynamic_target_heading = left_lane
                    print(f"\n[LANE AVOIDANCE] Avoiding {color.upper()}. New Goal: {dynamic_target_heading:.1f}°")
                current_state = "TURNING_WITH_GYRO"
                print(f"[STATE CHANGE] ==> {current_state}")

            elif current_state == "TURNING_WITH_GYRO":
                steer = steer_with_gyro(dynamic_target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                print(f"\rACTION: TURNING | Steer: {steer:5.1f}° | Goal: {dynamic_target_heading:.1f}°", end="")
                if get_angular_difference(current_yaw, dynamic_target_heading) < config.HEADING_LOCK_TOLERANCE:
                    current_state = "CLEARING_OBSTACLE"
                    frames_since_block_lost = 0
                    print(f"\n[STATE CHANGE] ==> {current_state} (Turn Complete)")
            
            elif current_state == "CLEARING_OBSTACLE":
                steer = steer_with_gyro(dynamic_target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if not block_data:
                    frames_since_block_lost += 1
                else: # Reset counter if we see the block again
                    frames_since_block_lost = 0
                print(f"\rACTION: CLEARING | Countdown: {frames_since_block_lost}/{config.CLEARANCE_FRAMES}", end="")
                if frames_since_block_lost > config.CLEARANCE_FRAMES:
                    current_state = "RETURNING_TO_CENTER"
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            elif current_state == "RETURNING_TO_CENTER":
                steer = steer_with_gyro(INITIAL_HEADING, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                print(f"\rACTION: RETURNING TO CENTER | Steer: {steer:5.1f}°", end="")
                if get_angular_difference(current_yaw, INITIAL_HEADING) < config.HEADING_LOCK_TOLERANCE:
                    current_state = "SCANNING_AT_CENTER"
                    frames_in_state = 0
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            elif current_state == "SCANNING_AT_CENTER":
                steer = steer_with_gyro(INITIAL_HEADING, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                frames_in_state += 1
                print(f"\rACTION: SCANNING... {frames_in_state}/{config.SCAN_DURATION_FRAMES}", end="")
                if frames_in_state > config.SCAN_DURATION_FRAMES:
                    if not first_search_completed:
                        current_state = "SEARCHING_PEEK_TURN"
                        print(f"\n[STATE CHANGE] ==> {current_state} (Initiating search)")
                    else:
                        current_state = "DRIVING_STRAIGHT"
                        print(f"\n[STATE CHANGE] ==> {current_state} (Search complete, resuming)")

            elif current_state == "SEARCHING_PEEK_TURN":
                peek_right = (INITIAL_HEADING + config.AVOID_ANGLE_DEG + 360) % 360
                peek_left = (INITIAL_HEADING - config.AVOID_ANGLE_DEG + 360) % 360
                if last_avoidance_direction == 'RIGHT':
                    dynamic_target_heading = peek_left
                else:
                    dynamic_target_heading = peek_right
                steer = steer_with_gyro(dynamic_target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                print(f"\rACTION: PEEK TURN | Steer: {steer:5.1f}° | Goal: {dynamic_target_heading:.1f}°", end="")
                if get_angular_difference(current_yaw, dynamic_target_heading) < config.HEADING_LOCK_TOLERANCE:
                    current_state = "SEARCHING_DRIVE"
                    frames_in_state = 0
                    print(f"\n[STATE CHANGE] ==> {current_state}")
            
            elif current_state == "SEARCHING_DRIVE":
                steer = steer_with_gyro(dynamic_target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                frames_in_state += 1
                print(f"\rACTION: SEARCHING DRIVE... {frames_in_state}/{config.SEARCH_DRIVE_FRAMES}", end="")
                if block_data: # If we see something, interrupt the search
                    current_state = "APPROACHING_DURING_PEEK"
                    print(f"\n[STATE CHANGE] ==> {current_state} (Block spotted during search)")
                elif frames_in_state > config.SEARCH_DRIVE_FRAMES:
                    first_search_completed = True # Search was successful and uninterrupted
                    current_state = "RETURNING_TO_CENTER"
                    print(f"\n[STATE CHANGE] ==> {current_state} (Search Finished)")

            elif current_state == "APPROACHING_DURING_PEEK":
                steer = steer_with_gyro(dynamic_target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                area = block_data['area'] if block_data else 0
                print(f"\rACTION: APPROACHING ON PEEK PATH | Area: {area:.0f}/{config.MIN_BLOCK_AREA_FOR_ACTION}", end="")
                if not block_data:
                    frames_in_state = 0 # <<<<<<< THE FIX
                    current_state = "SEARCHING_DRIVE"
                    print(f"\n[STATE CHANGE] ==> {current_state} (Lost Target during peek, restarting search drive)")
            
            # --- Display & Exit ---
            cv2.imshow("Robot View", overlay_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("\nINFO: 'q' pressed, exiting.")
                break

    except KeyboardInterrupt:
        print("\nINFO: Keyboard interrupt detected. Stopping.")
    except Exception as e:
        print(f"\nFATAL: An error occurred in the main loop: {e}")
        traceback.print_exc()
    finally:
        cleanup_all()
