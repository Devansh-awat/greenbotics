# src/open_challenge/main.py
# State-machine version for the Open Challenge.

import time
from src.motors import motor, servo
from src.sensors import vl53l1x, bno055
from src.open_challenge import config
from src.obstacle_challenge.utils import SafetyMonitor


def get_angular_difference(angle1, angle2):
    if angle1 is None or angle2 is None:
        return 360
    diff = angle1 - angle2
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return abs(diff)


def steer_with_gyro(current_heading_goal, current_yaw, max_angle_left,
                    max_angle_right):
    if not (config.GYRO_ENABLED and current_heading_goal is not None and
            current_yaw is not None):
        servo.set_angle(0.0)
        return 0.0
    error = current_heading_goal - current_yaw
    while error <= -180:
        error += 360
    while error > 180:
        error -= 360
    steer = config.GYRO_KP * error
    steer = max(-max_angle_left, min(steer, max_angle_right))
    servo.set_angle(steer)
    return steer


# --- Initialization ---
safety_monitor = None
try:
    print("--- Initializing Systems for Open Challenge ---")
    servo.initialize()
    motor.initialize()
    vl53l1x.initialize()
    bno055.initialize()

    if config.GYRO_ENABLED:
        safety_monitor = SafetyMonitor(bno055.sensor, config.TILT_THRESHOLD_DEGREES)

    print("--- Initialization Complete ---")

except Exception as e:
    print(f"FATAL: An error occurred during initialization: {e}")
    if safety_monitor:
        safety_monitor.stop()
    motor.cleanup(); servo.cleanup(); vl53l1x.cleanup(); bno055.cleanup()
    exit()

# --- Main Execution Logic ---
current_state = "STARTING_RUN"
turn_counter = 0
locked_turn_direction = None
target_heading = bno055.get_heading()
final_run_end_time = 0
frames_in_state = 0
# NEW: Mission timer start
start_time = time.monotonic()

print(f"\n[STATE CHANGE] ==> {current_state}")
motor.forward(config.DRIVE_SPEED)

try:
    while True:
        if safety_monitor and safety_monitor.is_triggered():
            print("\nINFO: Safety Monitor triggered stop.")
            break
        
        # NEW: Calculate elapsed time on each loop
        elapsed_time = time.monotonic() - start_time

        current_yaw = bno055.get_heading()
        dist_left = vl53l1x.get_distance(config.LEFT_SENSOR_CHANNEL)
        dist_right = vl53l1x.get_distance(config.RIGHT_SENSOR_CHANNEL)

        if current_state == "STARTING_RUN":
            steer_with_gyro(target_heading, current_yaw,
                            config.GYRO_MAX_STEER_LEFT,
                            config.GYRO_MAX_STEER_RIGHT)
            
            if dist_left is not None and dist_left < config.START_CLOSE_WALL_THRESHOLD:
                locked_turn_direction = 'left'
                print(f"\nINFO: Start too close to LEFT wall. Immediately following.")
                current_state = "WALL_FOLLOWING"
            elif dist_right is not None and dist_right < config.START_CLOSE_WALL_THRESHOLD:
                locked_turn_direction = 'right'
                print(f"\nINFO: Start too close to RIGHT wall. Immediately following.")
                current_state = "WALL_FOLLOWING"
            elif dist_left is None:
                locked_turn_direction = 'left'
                print(f"\nINFO: LEFT wall lost first. Locking to LEFT turns.")
                current_state = "PERFORMING_TURN"
                frames_in_state = 0
            elif dist_right is None:
                locked_turn_direction = 'right'
                print(f"\nINFO: RIGHT wall lost first. Locking to RIGHT turns.")
                current_state = "PERFORMING_TURN"
                frames_in_state = 0
            else:
                # MODIFIED: Added timer to printout
                print(f"\rT:{elapsed_time:5.1f}s | Driving straight, waiting for event...", end="")

        elif current_state == "WALL_FOLLOWING":
            wall_lost = False
            if locked_turn_direction == "left":
                if dist_left is not None:
                    error = dist_left - config.TARGET_DISTANCE
                    steer_angle = -error * config.KP
                    servo.set_angle(steer_angle)
                    # MODIFIED: Added timer to printout
                    print(f"\rT:{elapsed_time:5.1f}s | Following LEFT | Dist: {dist_left:6.1f}mm", end="")
                else:
                    wall_lost = True
            elif locked_turn_direction == "right":
                if dist_right is not None:
                    error = dist_right - config.TARGET_DISTANCE
                    steer_angle = error * config.KP
                    servo.set_angle(steer_angle)
                    # MODIFIED: Added timer to printout
                    print(f"\rT:{elapsed_time:5.1f}s | Following RIGHT | Dist: {dist_right:6.1f}mm", end="")
                else:
                    wall_lost = True

            if wall_lost:
                print(f"\nINFO: {locked_turn_direction.upper()} wall lost.")
                frames_in_state = 0
                current_state = "PERFORMING_TURN"

        elif current_state == "PERFORMING_TURN":
            frames_in_state += 1
            if frames_in_state == 1:
                if locked_turn_direction == "left":
                    target_heading = (target_heading - 90 + 360) % 360
                else:
                    target_heading = (target_heading + 90 + 360) % 360
                print(f"[STATE CHANGE] ==> {current_state} | New Target: {target_heading:.1f}°")

            steer_with_gyro(target_heading, current_yaw,
                            config.GYRO_MAX_STEER_LEFT,
                            config.GYRO_MAX_STEER_RIGHT)
            
            print(f"\rT:{elapsed_time:5.1f}s | Turning to {target_heading:.1f}°...", end="")

            if get_angular_difference(current_yaw, target_heading) < config.HEADING_LOCK_TOLERANCE:
                turn_counter += 1
                print(f"\nINFO: Turn {turn_counter}/{config.TOTAL_TURNS} complete.")

                if turn_counter >= config.TOTAL_TURNS:
                    current_state = "FINAL_WALL_FOLLOW"
                else:
                    current_state = "SEARCHING_FOR_WALL"
                print(f"[STATE CHANGE] ==> {current_state}")
                frames_in_state = 0

        elif current_state == "SEARCHING_FOR_WALL":
            servo.set_angle(0)
            print(f"\rT:{elapsed_time:5.1f}s | Searching for next wall...", end="")

            sensor_to_check = dist_left if locked_turn_direction == "left" else dist_right
            if sensor_to_check is not None:
                print("\nINFO: Next wall found.")
                current_state = "WALL_FOLLOWING"
                print(f"[STATE CHANGE] ==> {current_state}")

        elif current_state == "FINAL_WALL_FOLLOW":
            if frames_in_state == 0:
                final_run_end_time = time.time() + config.FINAL_FOLLOW_DURATION_S
                print(f"Beginning final {config.FINAL_FOLLOW_DURATION_S}s run.")
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
            print(f"\rT:{elapsed_time:5.1f}s | Final run... Time remaining: {remaining_time:.1f}s", end="")

        elif current_state == "MISSION_COMPLETE":
            print(f"\nMission complete. Final Time: {elapsed_time:.2f} seconds.")
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nSTOP: Program interrupted by user.")

finally:
    print("Stopping robot and cleaning up resources.")
    if safety_monitor:
        safety_monitor.stop()
    motor.cleanup()
    servo.cleanup()
    vl53l1x.cleanup()
    bno055.cleanup()
