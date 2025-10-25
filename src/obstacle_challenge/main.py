import time
import sys
import traceback
import cv2
import numpy as np
from gpiozero import Button, LED
import os
from datetime import datetime

from src.obstacle_challenge import config, neopixel
from src.motors import motor, servo
from src.sensors import camera, bno055, vl53l1x
from src.obstacle_challenge.utils import FPSCounter, SafetyMonitor


# ==============================================================================
# --- PARKING MANEUVER CONFIGURATION FOR CLOCKWISE DIRECTION---
# ==============================================================================
MANEUVER_SEQUENCE = [
    {'type': 'drive', 'target_heading': 0.0, 'servo_angle': 0, 'unlimited_servo': False, 'drive_direction': 'reverse', 'speed': 60, 'duration_frames': 45},
    {'type': 'turn', 'target_heading': 45.0,  'servo_angle': 45, 'unlimited_servo': False, 'drive_direction': 'forward', 'speed': 80, 'duration_frames': 0},
    {'type': 'drive', 'target_heading': 45.0,   'servo_angle': 0,  'unlimited_servo': False, 'drive_direction': 'reverse', 'speed': 80, 'duration_frames': 50},
    {'type': 'turn', 'target_heading': 15.0,   'servo_angle': 60, 'unlimited_servo': True,  'drive_direction': 'reverse', 'speed': 80, 'duration_frames': 0},
    {'type': 'turn', 'target_heading': 5.0,   'servo_angle': -40,  'unlimited_servo': False, 'drive_direction': 'forward', 'speed': 90, 'duration_frames': 0},
]
# ==============================================================================

# --- System & Parking Constants ---
TILT_THRESHOLD_DEGREES = 15.0
BUTTON_PIN = 23
WALL_APPROACH_THRESHOLD_MM = 100
POSITIONING_DRIVE_FRAMES = 100
SCAN_THRESHOLD_MM = 140
FRONT_SENSOR_CHANNEL = 1
LEFT_SENSOR_CHANNEL = 3
PARKING_DRIVE_SPEED = 80
PARKING_HEADING_LOCK_TOLERANCE = 5.0

kp = 0.0 

HEADLESS = not sys.stdout.isatty() or os.environ.get('DISPLAY') is None

def get_angular_difference(angle1, angle2):
    if angle1 is None or angle2 is None:
        return 360
    diff = angle1 - angle2
    while diff <= -180:
        diff += 360
    while diff > 180:
        diff -= 360
    return abs(diff)


def steer_with_gyro(current_heading_goal, current_yaw,clip_right=-45,clip_left=45):
    global kp
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
        steer = kp * error
        steer = np.clip(steer,clip_right,clip_left)
        servo.set_angle(steer)
        return steer
    servo.set_angle(0.0)
    return 0.0


safety_monitor = None
video_writer = None
video_path = None

def init_video_writer(example_frame, base_dir="/home/devansh/videos", preferred="mp4"):
    """
    Create a cv2.VideoWriter sized to the frame you pass in.
    Tries MP4 first, then falls back to AVI (XVID) if MP4 fails.
    """
    os.makedirs(base_dir, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    h, w = example_frame.shape[:2]
    fps = 30.0

    if preferred == "mp4":
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")   # widely supported on Pi
        mp4_path = os.path.join(base_dir, f"wro_run_{ts}.mp4")
        vw = cv2.VideoWriter(mp4_path, fourcc, fps, (w, h))
        if vw.isOpened():
            print(f"[REC] Writing MP4 to: {mp4_path}")
            return vw, mp4_path

    # Fallback to AVI if MP4 couldn't open (codec missing, etc.)
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    avi_path = os.path.join(base_dir, f"wro_run_{ts}.avi")
    vw = cv2.VideoWriter(avi_path, fourcc, fps, (w, h))
    print(f"[REC] Writing AVI to: {avi_path}")
    return vw, avi_path

def initialize_all():
    global safety_monitor,button,led
    print("--- Initializing All Systems ---")
    button = Button(BUTTON_PIN)
    led = LED(12)
    neopixel.init()
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
    neopixel.cleanup()
    global video_writer, video_path
    if video_writer is not None:
        video_writer.release()
        print(f"[REC] Saved recording to: {video_path}")
    print("--- Cleanup Complete ---")


if __name__ == "__main__":
    if not initialize_all():
        cleanup_all()
        sys.exit(1)

    kp = config.GYRO_KP
    
    INITIAL_HEADING = 0.0
    target_heading = 0.0
    maneuver_base_heading = 0.0
    detected_block_color = None
    maneuver_color = None
    scan_initiated = False
    turn_counter = 0
    blocks_passed_this_lap = 0
    frames_in_state = 0
    start_time = 0
    current_state=None
    driving_direction=None
    
    stage_index = 0
    maneuver_base_heading = 0

    try:
        WINDOW_NAME = "Robot View"
        if not HEADLESS:
            cv2.namedWindow(WINDOW_NAME)
        led.on()
        print("\nINFO: Ready. Press button to set '0' heading and start.")

        while True:
            if safety_monitor and safety_monitor.is_triggered():
                break
                
            elapsed_time = time.monotonic() - start_time if start_time > 0 else 0
            frame = camera.capture_frame()
            if frame is None:
                break
            block_data, overlay_frame, _ = camera.find_biggest_block(frame)
            # --- Recording setup & write ---
            if overlay_frame is None:
                overlay_frame = frame  # ensure we always have something to write/show

            if video_writer is None:
                video_writer, video_path = init_video_writer(overlay_frame)

            # Write current frame to file
            video_writer.write(overlay_frame)
            # --- end recording block ---
            current_yaw = bno055.get_heading()
            print(current_state)
            if current_state is None and button.is_pressed:
                print("\n--- Button Pressed! Starting Obstacle Challenge ---")
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
                if driving_direction=='counter-clockwise':
                    MANEUVER_SEQUENCE=[
                    {'type': 'drive', 'target_heading': 0.0, 'servo_angle': 0, 'unlimited_servo': False, 'drive_direction': 'reverse', 'speed': 60, 'duration_frames': 5},
                    {'type': 'turn', 'target_heading': -50.0,  'servo_angle': -45, 'unlimited_servo': False, 'drive_direction': 'forward', 'speed': 80, 'duration_frames': 0},
                    {'type': 'drive', 'target_heading': -50.0,   'servo_angle': 0,  'unlimited_servo': False, 'drive_direction': 'reverse', 'speed': 80, 'duration_frames': 45},
                    {'type': 'turn', 'target_heading': -15.0,   'servo_angle': -60, 'unlimited_servo': True,  'drive_direction': 'reverse', 'speed': 80, 'duration_frames': 0},
                    {'type': 'turn', 'target_heading': -10.0,   'servo_angle': 60,  'unlimited_servo': True, 'drive_direction': 'forward', 'speed': 90, 'duration_frames': 0},
                    {'type': 'drive', 'target_heading': 0.0, 'servo_angle': 0, 'unlimited_servo': False, 'drive_direction': 'reverse', 'speed': 60, 'duration_frames': 10},
                    ]
                    POSITIONING_DRIVE_FRAMES=200
                new_heading = bno055.get_heading()
                if new_heading is not None:
                    INITIAL_HEADING = new_heading
                else:
                    INITIAL_HEADING = 0.0
                target_heading = INITIAL_HEADING
                print(f"INFO: Initial heading (our '0') locked at: {INITIAL_HEADING:.2f}°")
                
                if config.GYRO_ENABLED:
                    print("--- Starting Safety Monitor ---")
                    safety_monitor = SafetyMonitor(bno055.sensor, TILT_THRESHOLD_DEGREES)
                #time.sleep(10)
                start_time = time.monotonic()
                current_state = (
                    "INITIAL_RIGHT_TURN_CW"
                    if driving_direction == "clockwise"
                    else "INITIAL_LEFT_TURN"
                )
                neopixel.solid(250,250,255)
                # starting from middle for testing
                #current_state='DRIVING_STRAIGHT'
                #turn_counter=4
                #driving_direction='counter-clockwise'
                
                led.off()

            elif current_state == "INITIAL_LEFT_TURN":
                frames_in_state += 1
                if frames_in_state == 1:
                    maneuver_base_heading=INITIAL_HEADING
                    # Set the target heading for a counter-clockwise turn
                    target_heading = (
                        INITIAL_HEADING - config.CCW_TURN_ANGLE + 360
                    ) % 360
                    motor.forward(config.DRIVE_SPEED)
                
                # Apply a left turn
                servo.set_angle_unlimited(-60)

                # Check if it's time to start the scan (same logic as the right turn)
                if (
                    get_angular_difference(INITIAL_HEADING, current_yaw)
                    >= config.CCW_SCAN_ANGLE
                    and not scan_initiated
                ):
                    scan_initiated = True
                    print("--- Starting 5-second continuous scan ---")
                    motor.brake()
                    
                    # Start a 5-second timer
                    scan_start_time = time.monotonic()
                    scan_duration = 5.0

                    # Loop for the duration, actively looking for a block
                    while time.monotonic() - scan_start_time < scan_duration:
                        # Capture a frame in every loop
                        scan_frame = camera.capture_frame()
                        if scan_frame is None:
                            continue # Skip this loop iteration if frame is bad

                        # Analyze the frame for a block
                        scan_block_data, overlay_frame_scan, _ = camera.find_biggest_block(scan_frame)
                        
                        # Provide live visual feedback during the scan
                        if not HEADLESS and overlay_frame_scan is not None:
                            cv2.imshow(WINDOW_NAME, overlay_frame_scan)
                            cv2.waitKey(1)

                        # If a block is found, save its color and exit the scan loop immediately
                        if scan_block_data:
                            detected_block_color = scan_block_data["color"]
                            print(f"Block found! Color: {detected_block_color}. Ending scan early.")
                            break # Exit the 'while' loop
                    
                    # If the loop finished without finding a block, detected_block_color remains None
                    if detected_block_color is None:
                        print("Scan finished. No block was detected.")

                    # Resume driving after the scan is complete (or was broken out of)
                    motor.forward(config.DRIVE_SPEED)
                    
                # Check if the full turn is complete after the scan has been attempted
                if (
                    scan_initiated
                    and get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    frames_in_state = 0
                    # Transition to the correct states for the counter-clockwise path
                    if detected_block_color == "green":
                        current_state = "DRIVE_FORWARD_GREEN"
                    elif detected_block_color == "red":
                        current_state = "REVERSE_BEFORE_TURN"
                    else:
                        current_state = "DRIVE_FORWARD_NONE"

            elif current_state == "DRIVE_FORWARD_GREEN":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if frames_in_state > config.CCW_NORMAL_DRIVE_FRAMES:
                    frames_in_state = 0
                    target_heading = (
                            maneuver_base_heading + config.MANEUVER_ANGLE_DEG+7 + 360
                        ) % 360
                    current_state = "MANEUVER_TURN_2"

            elif current_state == "DRIVE_FORWARD_NONE":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if frames_in_state > config.CCW_SHORT_DRIVE_FRAMES:
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
                    blocks_passed_this_lap = 2
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

                # Check if it's time to start the scan
                if (
                    get_angular_difference(INITIAL_HEADING, current_yaw)
                    >= config.CCW_SCAN_ANGLE
                    and not scan_initiated
                ):
                    scan_initiated = True
                    print("--- Starting 5-second continuous scan ---")
                    motor.brake()
                    
                    # Start a 5-second timer
                    scan_start_time = time.monotonic()
                    scan_duration = 5.0

                    # Loop for the duration, actively looking for a block
                    while time.monotonic() - scan_start_time < scan_duration:
                        # Capture a frame in every loop
                        scan_frame = camera.capture_frame()
                        if scan_frame is None:
                            continue # Skip this loop iteration if frame is bad

                        # Analyze the frame for a block
                        scan_block_data, overlay_frame, _ = camera.find_biggest_block(scan_frame)
                        
                        # Provide live visual feedback during the scan
                        if not HEADLESS:
                            cv2.imshow(WINDOW_NAME, overlay_frame)
                            cv2.waitKey(1)

                        # If a block is found, save its color and exit the scan loop immediately
                        if scan_block_data:
                            detected_block_color = scan_block_data["color"]
                            print(f"Block found! Color: {detected_block_color}. Ending scan early.")
                            break # Exit the 'while' loop
                    
                    # If the loop finished without finding a block, detected_block_color remains None
                    if detected_block_color is None:
                        print("Scan finished. No block was detected.")

                    # Resume driving after the scan is complete (or was broken out of)
                    motor.forward(config.DRIVE_SPEED)
                if (
                    scan_initiated
                    and get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    frames_in_state = 0
                    if detected_block_color == "green":
                        current_state = "REVERSE_FOR_GREEN_CW"
                    elif detected_block_color == "red":
                        current_state = "DRIVE_FORWARD_RED_CW"
                    else:
                        current_state = "DRIVE_FORWARD_NONE_CW"

            elif current_state == "DRIVE_FORWARD_RED_CW":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if frames_in_state > config.CW_DRIVE_FORWARD_RED_FRAMES:
                    frames_in_state = 0
                    target_heading = (
                            maneuver_base_heading - config.MANEUVER_ANGLE_DEG-7 + 360
                        ) % 360
                    current_state = "MANEUVER_TURN_2"

            elif current_state == "DRIVE_FORWARD_NONE_CW":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                if frames_in_state > config.CW_DRIVE_FORWARD_NONE_FRAMES:
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
                frames_in_state+=1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                forward_dist = vl53l1x.get_distance(config.TOF_FORWARD_SENSOR_CHANNEL)
                look_for_block = blocks_passed_this_lap < 2
                look_for_wall = blocks_passed_this_lap > 0
                min_block_area=config.MIN_BLOCK_AREA_FOR_ACTION
                if (turn_counter==4 or turn_counter==8) and maneuver_color=='red' and driving_direction=='counter-clockwise':
                    min_block_area+=1500 
                if (
                    look_for_block
                    and block_data
                    and block_data["area"] > min_block_area
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
                    if turn_counter==4 or turn_counter==8:
                        if maneuver_color=='red' and driving_direction=='counter-clockwise':
                            target_heading = (
                            maneuver_base_heading + config.MANEUVER_ANGLE_DEG-10
                        ) % 360
                        elif maneuver_color=='green' and driving_direction=='clockwise':
                            target_heading = (
                            maneuver_base_heading - config.MANEUVER_ANGLE_DEG+15 + 360
                        ) % 360
                    current_state = "MANEUVER_TURN_1"
                # Determine if this is the final leg of the obstacle course.
                is_final_leg = (turn_counter == config.TOTAL_TURNS - 1)

                # Set the appropriate distance threshold for the wall.
                if is_final_leg:
                    # On the final approach, use the tight parking threshold.
                    wall_detection_threshold = WALL_APPROACH_THRESHOLD_MM
                    x=0
                else:
                    # On normal laps, use the standard cornering threshold.
                    wall_detection_threshold = config.TOF_CORNERING_THRESHOLD_MM
                    if driving_direction == "counter-clockwise":
                        wall_detection_threshold -= 100

                # Check if the robot has reached the wall using the chosen threshold.
                if (look_for_wall and forward_dist is not None and forward_dist < wall_detection_threshold)and frames_in_state>25:
                    
                    # If the wall is detected, decide what to do next based on the lap.
                    if is_final_leg:
                        # --- TRANSITION DIRECTLY TO PARKING ---
                        print("\n--- Final approach complete. Starting parking sequence. ---")
                        # We are already at the wall, so we have completed the
                        # "APPROACHING_WALL_1" state's job.
                        # Now, we start the first turn of the parking maneuver.
                        kp = 3.0 # Switch to the parking gyro gain
                        frames_in_state = 0
                        current_state = "PARKING_TURN_1" # Go to the parking turn state
                    else:
                        print(forward_dist,wall_detection_threshold)
                        # --- PERFORM A NORMAL CORNER TURN ---
                        frames_in_state = 0
                        current_state = "PERFORMING_CORNER_TURN"
                    
            elif current_state == "MANEUVER_TURN_1":
                frames_in_state += 1
                x=35
                if (turn_counter==4 or turn_counter==8) and maneuver_color=='red' and driving_direction=='counter-clockwise':
                    x=45
                y=-29
                if (turn_counter==4 or turn_counter==8) and maneuver_color=='green' and driving_direction=='clockwise': 
                    y=-45
                steer_with_gyro(target_heading, current_yaw,y,x)
                motor.forward(config.DRIVE_SPEED)
                if (
                    frames_in_state > config.MIN_FRAMES_IN_TURN
                    and get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    frames_in_state = 0
                    if maneuver_color == "red":
                        current_state = "MANEUVER_DRIVE_1"
                    else:
                        target_heading = (
                            maneuver_base_heading + config.MANEUVER_ANGLE_DEG+7 + 360
                        ) % 360
                        if (turn_counter==4 or turn_counter==8) and maneuver_color=='green' and driving_direction=='clockwise':
                            target_heading = (
                            maneuver_base_heading + config.MANEUVER_ANGLE_DEG-10 + 360
                        ) % 360
                        current_state = "MANEUVER_TURN_2"

            elif current_state == "MANEUVER_DRIVE_1":
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(config.DRIVE_SPEED)
                
                if current_state == "MANEUVER_DRIVE_1":
                    drive_duration = config.MANEUVER_DRIVE_FRAMES_SIDEWAYS
                    if (turn_counter==4 or turn_counter==8) and maneuver_color=='red' and driving_direction=='counter-clockwise':
                        drive_duration=config.SPECIAL_TURN_SIDEWAYS_FRAMES
                    if frames_in_state > drive_duration:
                        if maneuver_color == "red":
                            target_heading = (
                                maneuver_base_heading - config.MANEUVER_ANGLE_DEG-10 + 360
                            ) % 360
                        if (turn_counter==4 or turn_counter==8) and maneuver_color=='red' and driving_direction=='counter-clockwise':
                            target_heading = (
                            maneuver_base_heading - config.MANEUVER_ANGLE_DEG+10
                        ) % 360
                            
                        frames_in_state = 0
                        current_state = "MANEUVER_TURN_2"

            elif current_state == "MANEUVER_TURN_2":
                x=37
                y=-32
                if (turn_counter==4 or turn_counter==8) and maneuver_color=='red' and driving_direction=='counter-clockwise':
                    x=45
                if maneuver_color=='red':
                    y=-28
                frames_in_state += 1
                steer_with_gyro(target_heading, current_yaw,y,x)
                motor.forward(config.DRIVE_SPEED)
                if (
                    frames_in_state > config.MIN_FRAMES_IN_TURN
                    and get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    target_heading=maneuver_base_heading
                    frames_in_state = 0
                    current_state = "MANEUVER_TURN_4"

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

                sensor_to_check = None
                if maneuver_color == 'red':
                    sensor_to_check = 0
                elif maneuver_color == 'green':
                    sensor_to_check = 2

                if sensor_to_check is not None:
                    dist = vl53l1x.get_distance(sensor_to_check)
                    if dist is not None and 0 < dist < 500:
                        print(f"  -> Drive 3 exit by sensor {sensor_to_check} at {dist}mm.")
                        target_heading = maneuver_base_heading
                        frames_in_state = 0
                        current_state = "MANEUVER_TURN_4"
                if current_state == "MANEUVER_DRIVE_3":
                    drive_duration = 0
                    if turn_counter == 4 or turn_counter == 8:
                        drive_duration = config.SPECIAL_TURN_SIDEWAYS_FRAMES
                    else:
                        drive_duration = config.MANEUVER_DRIVE_FRAMES_SIDEWAYS

                    if frames_in_state > drive_duration:
                        print("  -> Drive 3 exit by timer.")
                        print(dist)
                        target_heading = maneuver_base_heading
                        frames_in_state = 0
                        current_state = "MANEUVER_TURN_4"

            elif current_state == "MANEUVER_TURN_4":
                frames_in_state += 1
                x=35
                if (turn_counter==4 or turn_counter==8)and maneuver_color=='red' and driving_direction=='counter-clockwise':
                    x=45
                steer_with_gyro(target_heading, current_yaw,-20,x)
                motor.forward(config.DRIVE_SPEED)
                if (
                    get_angular_difference(current_yaw, target_heading)
                    < config.HEADING_LOCK_TOLERANCE
                ):
                    blocks_passed_this_lap += 1
                    current_state = "DRIVING_STRAIGHT"

            elif current_state == "PERFORMING_CORNER_TURN":
                frames_in_state += 1
                if frames_in_state == 1:
                    if driving_direction == "counter-clockwise":
                        target_heading = (target_heading - 90+0.15 + 360) % 360
                    else:
                        target_heading = (target_heading + 90-0.35 + 360) % 360
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
                    
                    if turn_counter < config.TOTAL_TURNS:
                        current_state = "DRIVING_STRAIGHT"
                    else:
                        print("\n--- OBSTACLE COURSE COMPLETE ---")
                        print("--- STARTING PARKING MANEUVER ---")
                        kp = 3.0
                        stage_index = 0
                        current_state = "PARKING_TURN_1"


            # --- PARKING SEQUENCE STATES ---
            
            elif current_state == "APPROACHING_WALL_1":
                target_heading=INITIAL_HEADING
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(PARKING_DRIVE_SPEED)
                forward_dist = vl53l1x.get_distance(FRONT_SENSOR_CHANNEL)
                dist_str = f"{forward_dist:.1f}" if forward_dist is not None else "N/A"
                print(f"\rDriving forward, waiting for wall... Dist: {dist_str} mm", end="")
                if forward_dist is not None and forward_dist < WALL_APPROACH_THRESHOLD_MM:
                    frames_in_state = 0
                    current_state = "PARKING_TURN_1"
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            elif current_state == "PARKING_TURN_1":
                frames_in_state += 1
                if frames_in_state == 1:
                    if driving_direction=='clockwise':
                        target_heading = (target_heading + 90 + 360) % 360
                        print(f"ACTION: Performing 90-degree CW reverse turn. Target: {target_heading:.1f}°")
                    else:
                        target_heading = (target_heading - 90 + 360) % 360
                        print(f"ACTION: Performing 90-degree CCW reverse turn. Target: {target_heading:.1f}°")
                if get_angular_difference(current_yaw, target_heading) < PARKING_HEADING_LOCK_TOLERANCE:
                    servo.set_angle(5)
                elif driving_direction=='clockwise':
                    servo.set_angle(-45)
                else:
                    servo.set_angle_unlimited(60)
                motor.reverse(PARKING_DRIVE_SPEED)
                if get_angular_difference(current_yaw, target_heading) < PARKING_HEADING_LOCK_TOLERANCE:
                    frames_in_state = 0
                    current_state = "DRIVE_FORWARD_POSITIONING"
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            elif current_state == "DRIVE_FORWARD_POSITIONING":
                frames_in_state += 1
                x=2
                if driving_direction=='clockwise':
                    x=-3
                steer_with_gyro(target_heading+x, current_yaw)
                motor.forward(PARKING_DRIVE_SPEED)
                print(f"\rDriving forward to scan position... Frames: {frames_in_state}/{POSITIONING_DRIVE_FRAMES}", end="")
                if frames_in_state > POSITIONING_DRIVE_FRAMES:
                    frames_in_state = 0
                    current_state = "WAITING_FOR_SCAN"
                    print(f"\n[STATE CHANGE] ==> {current_state}")

            elif current_state == "WAITING_FOR_SCAN":
                frames_in_state+=1
                steer_with_gyro(target_heading, current_yaw)
                motor.forward(50)
                if driving_direction=='clockwise':
                    dist_left = vl53l1x.get_distance(LEFT_SENSOR_CHANNEL)
                else:
                    dist_left = vl53l1x.get_distance(2)
                dist_str = f"{dist_left:.1f}" if dist_left is not None else "N/A"
                print(f"\rScanning... Left Dist: {dist_str} mm", end="")
                if dist_left is not None and dist_left < SCAN_THRESHOLD_MM:
                    print(f"\nScan triggered at {dist_left:.1f}mm. Starting maneuver sequence.")
                    motor.brake()
                    time.sleep(0.5)
                    maneuver_base_heading = target_heading
                    current_state = "RUNNING_MANEUVER"
                if frames_in_state>200:
                    current_state='MISSION_COMPLETE'

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

                    if get_angular_difference(current_yaw, current_stage_target_heading) < PARKING_HEADING_LOCK_TOLERANCE:
                        print(f"--- Stage {stage_index + 1} complete. ---")
                        stage_index += 1
                        frames_in_state = 0
                
                elif stage_type == 'drive':
                    frames_in_state += 1
                    if frames_in_state == 1:
                        print(f"\n[STAGE {stage_index + 1}] Starting DRIVE for {stage['duration_frames']} frames.")

                    steer_with_gyro(current_stage_target_heading, current_yaw)
                    
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
                print(f"\nFull Challenge Complete! Final Time: {elapsed_time:.2f} seconds.")
                break
            if not HEADLESS:
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
