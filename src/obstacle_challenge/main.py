import time
import sys
import traceback
import cv2
import numpy as np


from src.obstacle_challenge import config
from src.motors import motor, servo
from src.sensors import camera, bno055, vl53l1x
from src.obstacle_challenge.utils import FPSCounter, SafetyMonitor


def get_angular_difference(angle1, angle2):
    if angle1 is None or angle2 is None:
        return 360
    diff = angle1 - angle2
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return abs(diff)


def steer_with_gyro(current_heading_goal, current_yaw):
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


safety_monitor = None


def initialize_all():
    global safety_monitor
    print("--- Initializing All Systems ---")
    if not all(
        [
            motor.initialize(),
            servo.initialize(),
            camera.initialize(),
            bno055.initialize(),
            vl53l1x.initialize(),
        ]
    ):
        return False
    if config.GYRO_ENABLED:
        safety_monitor = SafetyMonitor(bno055.sensor, config.TILT_THRESHOLD_DEGREES)
    time.sleep(0.5)
    print("--- System Initialization Complete ---")
    return True


def cleanup_all():
    print("\n--- Cleaning up All Systems ---")
    if safety_monitor:
        safety_monitor.stop()
    motor.cleanup()
    servo.cleanup()
    camera.cleanup()
    bno055.cleanup()
    vl53l1x.cleanup()
    print("--- Cleanup Complete ---")


if __name__ == "__main__":
    if not initialize_all():
        cleanup_all()
        sys.exit(1)

    INITIAL_HEADING = bno055.get_heading()
    if INITIAL_HEADING is None:
        INITIAL_HEADING = 0.0
    print(f"INFO: Initial heading (our '0') locked at: {INITIAL_HEADING:.2f}°")

    print("INFO: Detecting driving direction...")
    time.sleep(1.0)
    dist_left = vl53l1x.get_distance(0)
    dist_right = vl53l1x.get_distance(2)
    driving_direction = "clockwise"
    if dist_left is not None and dist_right is not None:
        if dist_left < dist_right:
            driving_direction = "clockwise"
        else:
            driving_direction = "counter-clockwise"
    elif dist_left is not None:
        driving_direction = "clockwise"
    elif dist_right is not None:
        driving_direction = "counter-clockwise"
    else:
        print("WARNING: No walls detected. Defaulting to CLOCKWISE.")
    print(f"INFO: Driving direction set to {driving_direction.upper()}")

    current_state = (
        "INITIAL_RIGHT_TURN_CW"
        if driving_direction == "clockwise"
        else "INITIAL_LEFT_TURN"
    )

    target_heading = INITIAL_HEADING
    maneuver_base_heading = 0
    detected_block_color = None
    maneuver_color = None
    scan_initiated = False
    turn_counter = 0
    blocks_passed_this_lap = 0
    frames_in_state = 0
    start_time = time.monotonic()

    print(f"\n[STATE CHANGE] ==> {current_state}")
    try:
        WINDOW_NAME = "Robot View"
        cv2.namedWindow(WINDOW_NAME)

        while True:
            if safety_monitor and safety_monitor.is_triggered():
                break
            elapsed_time = time.monotonic() - start_time
            frame = camera.capture_frame()
            if frame is None:
                break
            block_data, overlay_frame, _ = camera.find_biggest_block(frame)
            current_yaw = bno055.get_heading()

            if current_state == "INITIAL_LEFT_TURN":
                frames_in_state += 1
                if frames_in_state == 1:
                    target_heading = (
                        INITIAL_HEADING - config.CCW_TURN_ANGLE + 360
                    ) % 360
                    motor.forward(config.DRIVE_SPEED)
                servo.set_angle_unlimited(-60)
                if (
                    get_angular_difference(INITIAL_HEADING, current_yaw)
                    >= config.CCW_SCAN_ANGLE
                    and not scan_initiated
                ):
                    scan_initiated = True
                    motor.brake()
                    time.sleep(0.5)
                    scan_frame = camera.capture_frame()
                    if scan_frame is not None:
                        scan_block_data, _, _ = camera.find_biggest_block(scan_frame)
                        if scan_block_data:
                            detected_block_color = scan_block_data["color"]
                    motor.forward(config.DRIVE_SPEED)
                if (
                    scan_initiated
                    and get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    if detected_block_color == "green":
                        current_state = "DRIVE_FORWARD_GREEN"
                    elif detected_block_color == "red":
                        current_state = "REVERSE_BEFORE_TURN"
                    else:
                        current_state = "MISSION_COMPLETE"

            elif current_state == "DRIVE_FORWARD_GREEN":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if frames_in_state > config.CCW_NORMAL_DRIVE_FRAMES:
                    frames_in_state = 0
                    current_state = "FINAL_RIGHT_TURN"

            elif current_state == "REVERSE_BEFORE_TURN":
                frames_in_state += 1
                servo.set_angle(0)
                motor.reverse(config.DRIVE_SPEED)
                if frames_in_state > config.CCW_REVERSE_FRAMES:
                    frames_in_state = 0
                    current_state = "FINAL_RIGHT_TURN"

            elif current_state == "FINAL_RIGHT_TURN":
                frames_in_state += 1
                if frames_in_state == 1:
                    target_heading = INITIAL_HEADING
                servo.set_angle_unlimited(60)
                motor.forward(config.DRIVE_SPEED)
                if (
                    get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    blocks_passed_this_lap = 1
                    frames_in_state = 0
                    current_state = "DRIVING_STRAIGHT"

            elif current_state == "INITIAL_RIGHT_TURN_CW":
                frames_in_state += 1
                if frames_in_state == 1:
                    target_heading = (
                        INITIAL_HEADING + config.CCW_TURN_ANGLE + 360
                    ) % 360
                    motor.forward(config.DRIVE_SPEED)
                servo.set_angle_unlimited(60)
                if (
                    get_angular_difference(INITIAL_HEADING, current_yaw)
                    >= config.CCW_SCAN_ANGLE
                    and not scan_initiated
                ):
                    scan_initiated = True
                    motor.brake()
                    time.sleep(0.5)
                    scan_frame = camera.capture_frame()
                    if scan_frame is not None:
                        scan_block_data, _, _ = camera.find_biggest_block(scan_frame)
                        if scan_block_data:
                            detected_block_color = scan_block_data["color"]
                    motor.forward(config.DRIVE_SPEED)
                if (
                    scan_initiated
                    and get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    if detected_block_color == "green":
                        current_state = "REVERSE_FOR_GREEN_CW"
                    elif detected_block_color == "red":
                        current_state = "DRIVE_FORWARD_RED_CW"
                    else:
                        current_state = "MISSION_COMPLETE"

            elif current_state == "DRIVE_FORWARD_RED_CW":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if frames_in_state > config.CW_DRIVE_FORWARD_RED_FRAMES:
                    frames_in_state = 0
                    current_state = "FINAL_LEFT_TURN_CW"

            elif current_state == "REVERSE_FOR_GREEN_CW":
                frames_in_state += 1
                servo.set_angle(0)
                motor.reverse(config.DRIVE_SPEED)
                if frames_in_state > config.CW_REVERSE_GREEN_FRAMES:
                    frames_in_state = 0
                    current_state = "FINAL_LEFT_TURN_CW"

            elif current_state == "FINAL_LEFT_TURN_CW":
                frames_in_state += 1
                if frames_in_state == 1:
                    target_heading = INITIAL_HEADING
                servo.set_angle_unlimited(-60)
                motor.forward(config.DRIVE_SPEED)
                if (
                    get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    blocks_passed_this_lap = 1
                    frames_in_state = 0
                    current_state = "DRIVING_STRAIGHT"

            elif current_state == "DRIVING_STRAIGHT":
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                forward_dist = vl53l1x.get_distance(config.TOF_FORWARD_SENSOR_CHANNEL)
                look_for_block = blocks_passed_this_lap < 2
                look_for_wall = blocks_passed_this_lap > 0
                if (
                    look_for_block
                    and block_data
                    and block_data["area"] > config.MIN_BLOCK_AREA_FOR_ACTION
                ):
                    maneuver_color = block_data["color"]
                    maneuver_base_heading = target_heading
                    if maneuver_color == "red":
                        target_heading = (
                            maneuver_base_heading + config.MANEUVER_ANGLE_DEG
                        ) % 360
                    else:
                        target_heading = (
                            maneuver_base_heading - config.MANEUVER_ANGLE_DEG + 360
                        ) % 360
                    frames_in_state = 0
                    current_state = "MANEUVER_TURN_1"
                elif (
                    look_for_wall
                    and forward_dist is not None
                    and forward_dist < config.TOF_CORNERING_THRESHOLD_MM
                ):
                    if turn_counter < config.TOTAL_TURNS:
                        frames_in_state = 0
                        current_state = "PERFORMING_CORNER_TURN"
                    else:
                        current_state = "MISSION_COMPLETE"

            elif current_state == "MANEUVER_TURN_1":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if (
                    frames_in_state > config.MIN_FRAMES_IN_TURN
                    and get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    frames_in_state = 0
                    current_state = "MANEUVER_DRIVE_1"

            elif current_state == "MANEUVER_DRIVE_1":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)

                drive_duration = config.MANEUVER_DRIVE_FRAMES_SIDEWAYS
                if turn_counter == 4 or turn_counter == 8:
                    drive_duration = config.SPECIAL_TURN_SIDEWAYS_FRAMES
                if frames_in_state > drive_duration:
                    target_heading = maneuver_base_heading
                    frames_in_state = 0
                    current_state = "MANEUVER_TURN_2"

            elif current_state == "MANEUVER_TURN_2":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if (
                    frames_in_state > config.MIN_FRAMES_IN_TURN
                    and get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    frames_in_state = 0
                    current_state = "MANEUVER_DRIVE_2"

            elif current_state == "MANEUVER_DRIVE_2":
                frames_in_state += 1
                steer_with_gyro(maneuver_base_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if frames_in_state > config.MANEUVER_DRIVE_FRAMES_STRAIGHT:
                    if maneuver_color == "red":
                        target_heading = (
                            maneuver_base_heading - config.MANEUVER_ANGLE_DEG + 360
                        ) % 360
                    else:
                        target_heading = (
                            maneuver_base_heading + config.MANEUVER_ANGLE_DEG + 360
                        ) % 360
                    frames_in_state = 0
                    current_state = "MANEUVER_TURN_3"

            elif current_state == "MANEUVER_TURN_3":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if (
                    get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    frames_in_state = 0
                    current_state = "MANEUVER_DRIVE_3"

            elif current_state == "MANEUVER_DRIVE_3":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)

                drive_duration = config.MANEUVER_DRIVE_FRAMES_SIDEWAYS
                if turn_counter == 4 or turn_counter == 8:
                    drive_duration = config.SPECIAL_TURN_SIDEWAYS_FRAMES
                if frames_in_state > drive_duration:
                    target_heading = maneuver_base_heading
                    frames_in_state = 0
                    current_state = "MANEUVER_TURN_4"

            elif current_state == "MANEUVER_TURN_4":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if (
                    get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    blocks_passed_this_lap += 1
                    if (
                        turn_counter >= config.TOTAL_TURNS
                        and blocks_passed_this_lap > 0
                    ):
                        current_state = "MISSION_COMPLETE"
                    else:
                        current_state = "DRIVING_STRAIGHT"

            elif current_state == "PERFORMING_CORNER_TURN":
                frames_in_state += 1
                if frames_in_state == 1:
                    if driving_direction == "counter-clockwise":
                        target_heading = (target_heading - 90 + 360) % 360
                    else:
                        target_heading = (target_heading + 90 + 360) % 360
                servo_angle = -45 if driving_direction == "clockwise" else 45
                servo.set_angle(servo_angle)
                motor.reverse(config.DRIVE_SPEED)
                if (
                    abs(get_angular_difference(current_yaw, target_heading))
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    turn_counter += 1
                    blocks_passed_this_lap = 0
                    frames_in_state = 0
                    current_state = "DRIVING_STRAIGHT"

            elif current_state == "MISSION_COMPLETE":
                motor.brake()
                servo.set_angle(0)
                break

            cv2.imshow(WINDOW_NAME, overlay_frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    except KeyboardInterrupt:
        print("\nINFO: Keyboard interrupt detected.")
    except Exception as e:
        print(f"\nFATAL: An error occurred: {e}")
        traceback.print_exc()
    finally:
        cleanup_all()
