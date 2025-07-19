from src.motors import motor, servo

# We assume the sensor library is imported like this.
# Please adjust if your import is different.
from src.sensors import vl53l0x
import time

# --- Constants ---
TARGET_DISTANCE = 250
KP = 0.3
TURN_COOLDOWN = 9.0
MAX_TURNS = 12
FINAL_WALL_FOLLOW_DURATION = 2.0

# --- Initialization ---
try:
    servo.initialize()
    motor.initialize()
    vl53l0x.initialize()
except Exception as e:
    print(f"An error occurred during initialization: {e}")
    exit()

# --- State Variables ---
servo.set_angle(0)
turn_count = 0
locked_turn_direction = None
last_turn_time = 0
# New state to track if we are in the middle of the final turn
final_turn_is_active = False

print("Starting program. Defaulting to right-wall following.")
motor.forward(100)

try:
    # Main loop runs indefinitely until we explicitly break out
    while True:
        # Always read sensors
        dis_left = vl53l0x.get_distance(0)
        dis_right = vl53l0x.get_distance(2)

        # --- NEW: LOGIC TO COMPLETE THE 12th TURN ---
        if final_turn_is_active:
            print("Finishing final turn...")
            # Check which direction we're turning and wait for sensor to get a reading
            if locked_turn_direction == "left" and dis_left is not None:
                print("12th LEFT turn complete.")
                final_turn_is_active = False  # Turn is done
            elif locked_turn_direction == "right" and dis_right is not None:
                print("12th RIGHT turn complete.")
                final_turn_is_active = False  # Turn is done

            # If the turn is now finished, start the final wall-follow
            if not final_turn_is_active:
                print(
                    f"Beginning final {FINAL_WALL_FOLLOW_DURATION}-second wall-follow."
                )
                end_time = time.time() + FINAL_WALL_FOLLOW_DURATION
                while time.time() < end_time:
                    # Execute one last wall-following loop
                    if locked_turn_direction == "left":
                        current_dis = vl53l0x.get_distance(0)
                        if current_dis is not None:
                            angle = (TARGET_DISTANCE - current_dis) * KP
                            servo.set_angle(angle)
                    elif locked_turn_direction == "right":
                        current_dis = vl53l0x.get_distance(2)
                        if current_dis is not None:
                            angle = (current_dis - TARGET_DISTANCE) * KP
                            servo.set_angle(angle)
                    time.sleep(0.1)

                # After the final wall-follow is done, break the main loop
                print("Final sequence complete.")
                break

            # If we are still in the middle of the turn, just sleep and loop again
            time.sleep(0.1)
            continue  # Skip the rest of the logic and re-check turn completion

        # ---- Standard Operating Logic (before 12th turn) ----

        # Case 1: Locked onto the LEFT wall
        if locked_turn_direction == "left":
            if dis_left is None:
                servo.set_angle(-25)
                if (time.time() - last_turn_time) > TURN_COOLDOWN:
                    turn_count += 1
                    last_turn_time = time.time()
                    print(f"Left turn counted! Total turns: {turn_count}/{MAX_TURNS}")
                    if turn_count == MAX_TURNS:
                        final_turn_is_active = True
            else:  # Follow left wall
                angle = (TARGET_DISTANCE - dis_left) * KP
                servo.set_angle(angle)

        # Case 2: Locked onto the RIGHT wall
        elif locked_turn_direction == "right":
            if dis_right is None:
                servo.set_angle(45)
                print("turning right")
                if (time.time() - last_turn_time) > TURN_COOLDOWN:
                    turn_count += 1
                    last_turn_time = time.time()
                    print(f"Right turn counted! Total turns: {turn_count}/{MAX_TURNS}")
                    if turn_count == MAX_TURNS:
                        final_turn_is_active = True
            else:  # Follow right wall
                angle = (dis_right - TARGET_DISTANCE) * KP
                print(angle)
                servo.set_angle(angle)

        # Case 3: Initial state (no direction locked)
        else:
            if dis_left is None:
                print("First turn detected: LEFT. Locking to left sensor.")
                locked_turn_direction = "left"
                servo.set_angle(-18)
                turn_count += 1
                last_turn_time = time.time()
                print(f"Left turn counted! Total turns: {turn_count}/{MAX_TURNS}")
                if turn_count == MAX_TURNS:
                    final_turn_is_active = True

            elif dis_right is None:
                print("First turn detected: RIGHT. Locking to right sensor.")
                locked_turn_direction = "right"
                servo.set_angle(40)
                turn_count += 1
                last_turn_time = time.time()
                print(f"Right turn counted! Total turns: {turn_count}/{MAX_TURNS}")
                if turn_count == MAX_TURNS:
                    final_turn_is_active = True
            else:
                servo.set_angle(10)

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping due to user interrupt (KeyboardInterrupt).")

finally:
    # --- Cleanup ---
    print("Cleaning up motor and servo resources.")
    motor.cleanup()
    servo.cleanup()
