# src/obstacle_challenge/main.py
import time
import sys
import traceback
import cv2
import numpy as np

# Import our hardware and logic modules
from src.obstacle_challenge import config
from src.motors import motor, servo
from src.sensors import camera, bno055, vl53l0x
from src.obstacle_challenge.utils import FPSCounter, AsyncSensorReader


# --- Helper Functions ---
def get_angular_difference(angle1, angle2):
    """Calculates the shortest difference between two angles in degrees."""
    if angle1 is None or angle2 is None:
        return 360  # Return large value if data is bad
    diff = angle1 - angle2
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return abs(diff)


def steer_with_gyro(current_heading_goal, current_yaw):
    """Steers the robot to a target heading using the gyro."""
    if (
        config.GYRO_ENABLED
        and current_heading_goal is not None
        and current_yaw is not None
    ):
        error = current_heading_goal - current_yaw
        while error <= -180:
            error += 360
        while error > 180:
            error -= 360
        steer = config.GYRO_KP * error
        servo.set_angle(steer)
        return steer
    servo.set_angle(0.0)
    return 0.0


# --- Initialization and Cleanup ---
tof_reader, gyro_reader = None, None


def initialize_all():
    """Initializes all hardware modules."""
    global tof_reader, gyro_reader
    print("--- Initializing All Systems ---")
    if not all(
        [
            motor.initialize(),
            servo.initialize(),
            camera.initialize(),
            bno055.initialize(),
            vl53l0x.initialize(),
        ]
    ):
        return False
    if config.TOF_ENABLED:
        tof_reader = AsyncSensorReader(
            vl53l0x.get_distance, config.TOF_FORWARD_SENSOR_CHANNEL
        )
    if config.GYRO_ENABLED:
        gyro_reader = AsyncSensorReader(bno055.get_heading)
    time.sleep(0.5)
    print("--- System Initialization Complete ---")
    return True


def cleanup_all():
    """Cleans up all hardware modules."""
    print("\n--- Cleaning up All Systems ---")
    if tof_reader:
        tof_reader.stop()
    if gyro_reader:
        gyro_reader.stop()
    motor.cleanup()
    servo.cleanup()
    camera.cleanup()
    bno055.cleanup()
    vl53l0x.cleanup()
    print("--- Cleanup Complete ---")


# ==============================================================================
# --- MAIN EXECUTION LOGIC ---
# ==============================================================================
if __name__ == "__main__":
    if not initialize_all():
        cleanup_all()
        sys.exit(1)

    INITIAL_HEADING = gyro_reader.read() if gyro_reader else 0.0
    if INITIAL_HEADING is None:
        print("WARNING: Gyro thread failed to get initial read, retrying...")
        time.sleep(1)
        INITIAL_HEADING = gyro_reader.read() if gyro_reader else 0.0
        if INITIAL_HEADING is None:
            INITIAL_HEADING = 0.0
    print(f"INFO: Initial heading locked at: {INITIAL_HEADING:.2f}°")

    # State Machine Variables
    current_state, last_avoidance_direction = "DRIVING_STRAIGHT", None
    dynamic_target_heading, future_initial_heading = INITIAL_HEADING, INITIAL_HEADING
    frames_in_state, frames_since_block_lost = 0, 0
    is_peek_avoidance, is_dead_end_turn = False, False
    has_avoided_a_block = False
    completed_avoidance_cycles = 0

    fps_counter = FPSCounter().start()
    alerted_for_low_fps = False

    print(f"\n[STATE CHANGE] ==> {current_state}")
    try:
        cv2.imshow(
            "Robot View",
            np.zeros((config.FRAME_HEIGHT, config.FRAME_WIDTH, 3), dtype=np.uint8),
        )
        cv2.waitKey(100)

        while True:
            frame = camera.capture_frame()
            if frame is None:
                break
            block_data, overlay_frame = camera.find_biggest_block(frame)
            current_yaw = gyro_reader.read() if gyro_reader else None
            forward_dist = tof_reader.read() if tof_reader else None

            # --- High-Priority Overrides ---
            if (
                has_avoided_a_block
                and not block_data
                and (
                    forward_dist is not None
                    and forward_dist < config.TOF_OBSTACLE_THRESHOLD_MM
                )
            ):
                if current_state in ["DRIVING_STRAIGHT", "APPROACHING_OBSTACLE"]:
                    print(f"\n[DEAD END OVERRIDE] Wall detected at {forward_dist} mm!")
                    is_dead_end_turn, current_state = True, "INITIATING_TURN"
            elif block_data and block_data["area"] > config.MIN_BLOCK_AREA_FOR_ACTION:
                if current_state not in [
                    "INITIATING_TURN",
                    "TURNING_WITH_GYRO",
                    "CLEARING_OBSTACLE",
                ]:
                    print(
                        f"\n[BLOCK OVERRIDE] Block is too close! Area: {block_data['area']:.0f}"
                    )
                    is_peek_avoidance = current_state in [
                        "SEARCHING_DRIVE",
                        "APPROACHING_DURING_PEEK",
                    ]
                    current_state = "INITIATING_TURN"

            # --- State Machine ---
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
                area = block_data["area"] if block_data else 0
                print(
                    f"\rACTION: APPROACHING ON MAIN PATH | Area: {area:.0f}/{config.MIN_BLOCK_AREA_FOR_ACTION}",
                    end="",
                )
                if not block_data:
                    current_state = "DRIVING_STRAIGHT"
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            elif current_state == "INITIATING_TURN":
                current_heading_at_decision = (
                    current_yaw if current_yaw is not None else INITIAL_HEADING
                )
                if is_dead_end_turn:
                    turn_angle = (
                        config.DEAD_END_TURN_ANGLE
                        if config.DEAD_END_TURN_DIRECTION == "RIGHT"
                        else -config.DEAD_END_TURN_ANGLE
                    )
                    dynamic_target_heading = (
                        current_heading_at_decision + turn_angle + 360
                    ) % 360
                    future_initial_heading = (INITIAL_HEADING + turn_angle + 360) % 360
                    print(
                        f"\n[DEAD END TURN] Turning {config.DEAD_END_TURN_DIRECTION}. New Goal: {dynamic_target_heading:.1f}°"
                    )
                elif is_peek_avoidance:
                    color = block_data["color"]
                    turn_angle = (
                        config.PEEK_AVOID_ANGLE_DEG
                        if color == "red"
                        else -config.PEEK_AVOID_ANGLE_DEG
                    )
                    dynamic_target_heading = (
                        current_heading_at_decision + turn_angle + 360
                    ) % 360
                    print(
                        f"\n[PEEK AVOIDANCE] Avoiding {color.upper()} with {abs(turn_angle)} deg turn. New Goal: {dynamic_target_heading:.1f}°"
                    )
                else:
                    color = block_data["color"]
                    if color == "red":
                        last_avoidance_direction = "RIGHT"
                        dynamic_target_heading = (
                            INITIAL_HEADING + config.AVOID_ANGLE_DEG + 360
                        ) % 360
                    else:
                        last_avoidance_direction = "LEFT"
                        dynamic_target_heading = (
                            INITIAL_HEADING - config.AVOID_ANGLE_DEG + 360
                        ) % 360
                    print(
                        f"\n[LANE AVOIDANCE] Avoiding {color.upper()}. New Goal: {dynamic_target_heading:.1f}°"
                    )
                current_state = "TURNING_WITH_GYRO"
                print(f"[STATE CHANGE] ==> {current_state}")

            elif current_state == "TURNING_WITH_GYRO":
                steer = steer_with_gyro(dynamic_target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                print(
                    f"\rACTION: TURNING | Steer: {steer:5.1f}° | Goal: {dynamic_target_heading:.1f}°",
                    end="",
                )
                if (
                    get_angular_difference(current_yaw, dynamic_target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    if is_dead_end_turn:
                        current_state = "HEADING_RESET"
                    else:
                        current_state = "CLEARING_OBSTACLE"
                        frames_since_block_lost = 0
                    print(f"\n[STATE CHANGE] ==> {current_state}")
                    is_peek_avoidance, is_dead_end_turn = False, False

            elif current_state == "HEADING_RESET":
                INITIAL_HEADING, dynamic_target_heading = (
                    future_initial_heading,
                    future_initial_heading,
                )
                last_avoidance_direction, has_avoided_a_block = None, False
                completed_avoidance_cycles = 0  # RESET cycle counter for the new path
                print(
                    f"\n[HEADING RESET] New 'straight' is {INITIAL_HEADING:.1f}°. Resetting mission."
                )
                current_state = "DRIVING_STRAIGHT"
                print(f"[STATE CHANGE] ==> {current_state}")

            elif current_state == "CLEARING_OBSTACLE":
                steer = steer_with_gyro(dynamic_target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if not block_data:
                    frames_since_block_lost += 1
                else:
                    frames_since_block_lost = 0
                print(
                    f"\rACTION: CLEARING | Countdown: {frames_since_block_lost}/{config.CLEARANCE_FRAMES}",
                    end="",
                )
                if frames_since_block_lost > config.CLEARANCE_FRAMES:
                    has_avoided_a_block = True
                    current_state = "RETURNING_TO_CENTER"
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            elif current_state == "RETURNING_TO_CENTER":
                steer = steer_with_gyro(INITIAL_HEADING, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                print(f"\rACTION: RETURNING TO CENTER | Steer: {steer:5.1f}°", end="")
                if (
                    get_angular_difference(current_yaw, INITIAL_HEADING)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    # <<< UPDATED: Increment counter upon returning to center >>>
                    completed_avoidance_cycles += 1
                    print(
                        f"\n[INFO] Avoidance cycles completed: {completed_avoidance_cycles}"
                    )
                    current_state = "SCANNING_AT_CENTER"
                    frames_in_state = 0
                    print(f"[STATE CHANGE] ==> {current_state}")

            elif current_state == "SCANNING_AT_CENTER":
                steer = steer_with_gyro(INITIAL_HEADING, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                frames_in_state += 1
                print(
                    f"\rACTION: SCANNING... {frames_in_state}/{config.SCAN_DURATION_FRAMES}",
                    end="",
                )
                if frames_in_state > config.SCAN_DURATION_FRAMES:
                    # <<< UPDATED: "No Peek on Second Block" Logic using the counter >>>
                    if completed_avoidance_cycles <= 1:
                        current_state = "SEARCHING_PEEK_TURN"
                        print(
                            f"\n[STATE CHANGE] ==> {current_state} (First block cycle, peeking is allowed)"
                        )
                    else:
                        current_state = "FORCE_DEAD_END_CLEARANCE"
                        frames_in_state = 0
                        print(
                            f"\n[STATE CHANGE] ==> {current_state} (Subsequent block, forcing dead end)"
                        )

            elif current_state == "FORCE_DEAD_END_CLEARANCE":
                steer = steer_with_gyro(INITIAL_HEADING, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                frames_in_state += 1
                print(
                    f"\rACTION: FORCING CLEARANCE... {frames_in_state}/{config.FORCE_CLEARANCE_FRAMES}",
                    end="",
                )
                if frames_in_state > config.FORCE_CLEARANCE_FRAMES:
                    print("\nForcing dead end turn...")
                    is_dead_end_turn = True
                    current_state = "INITIATING_TURN"

            elif current_state == "SEARCHING_PEEK_TURN":
                peek_right = (INITIAL_HEADING + config.AVOID_ANGLE_DEG + 360) % 360
                peek_left = (INITIAL_HEADING - config.AVOID_ANGLE_DEG + 360) % 360
                dynamic_target_heading = (
                    peek_left if last_avoidance_direction == "RIGHT" else peek_right
                )
                steer = steer_with_gyro(dynamic_target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                print(
                    f"\rACTION: PEEK TURN | Steer: {steer:5.1f}° | Goal: {dynamic_target_heading:.1f}°",
                    end="",
                )
                if (
                    get_angular_difference(current_yaw, dynamic_target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    current_state = "SEARCHING_DRIVE"
                    frames_in_state = 0
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            elif current_state == "SEARCHING_DRIVE":
                steer = steer_with_gyro(dynamic_target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                frames_in_state += 1
                print(
                    f"\rACTION: SEARCHING DRIVE... {frames_in_state}/{config.SEARCH_DRIVE_FRAMES}",
                    end="",
                )
                if block_data:
                    current_state = "APPROACHING_DURING_PEEK"
                    print(f"\n[STATE CHANGE] ==> {current_state}")
                elif frames_in_state > config.SEARCH_DRIVE_FRAMES:
                    current_state = "RETURNING_TO_CENTER"
                    print(
                        f"\n[STATE CHANGE] ==> {current_state} (Search Finished, did not find anything)"
                    )

            elif current_state == "APPROACHING_DURING_PEEK":
                steer = steer_with_gyro(dynamic_target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                area = block_data["area"] if block_data else 0
                print(
                    f"\rACTION: APPROACHING ON PEEK PATH | Area: {area:.0f}/{config.MIN_BLOCK_AREA_FOR_ACTION}",
                    end="",
                )
                if not block_data:
                    frames_in_state = 0
                    current_state = "SEARCHING_DRIVE"
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            fps_counter.update()
            fps_counter.display_on_frame(overlay_frame, alert_threshold=28.0)
            cv2.imshow("Robot View", overlay_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("\nINFO: 'q' pressed, exiting.")
                break

    except KeyboardInterrupt:
        print("\nINFO: Keyboard interrupt detected. Stopping.")
    except Exception as e:
        print(f"\nFATAL: An error occurred in the main loop: {e}")
        traceback.print_exc()
    finally:
        cleanup_all()
