import board
import adafruit_bno055
import time
import numpy as np
import json
from src.obstacle_challenge import config

# Define the file where calibration data will be stored
CALIBRATION_FILE = "src/sensors/bno055_calibration.json"

sensor = None


def load_calibration():
    """Loads calibration offsets from a file and applies them to the sensor."""
    global sensor
    try:
        with open(CALIBRATION_FILE, "r") as cal_file:
            data = json.load(cal_file)
            print("INFO: Found calibration file. Loading offsets...")
            
            # The BNO055 must be in CONFIG_MODE to accept new offsets.
            original_mode = sensor.mode
            sensor.mode = adafruit_bno055.CONFIG_MODE
            time.sleep(0.02) # Wait for mode switch
            
            sensor.offsets_accelerometer = tuple(data['accel'])
            sensor.offsets_gyroscope = tuple(data['gyro'])
            sensor.offsets_magnetometer = tuple(data['mag'])
            
            # Restore the original mode
            sensor.mode = original_mode
            time.sleep(0.02) # Wait for mode switch
            
            print("INFO: Offsets loaded successfully.")
            return True
            
    except FileNotFoundError:
        print("WARNING: No calibration file found. Sensor will be uncalibrated.")
        print("         Run this script directly to perform calibration.")
        return False
    except Exception as e:
        print(f"ERROR: Could not load calibration data: {e}")
        return False


def save_calibration():
    """Reads the sensor's offsets and saves them to a file."""
    if not sensor:
        print("ERROR: Cannot save calibration. Sensor not initialized.")
        return

    # Get the calibration offsets
    accel_offsets = sensor.offsets_accelerometer
    gyro_offsets = sensor.offsets_gyroscope
    mag_offsets = sensor.offsets_magnetometer

    # Store them in a dictionary
    cal_data = {
        'accel': accel_offsets,
        'gyro': gyro_offsets,
        'mag': mag_offsets
    }

    try:
        with open(CALIBRATION_FILE, "w") as cal_file:
            json.dump(cal_data, cal_file, indent=4)
        print(f"\nINFO: Calibration data saved to {CALIBRATION_FILE}")
    except Exception as e:
        print(f"ERROR: Could not save calibration data: {e}")


def perform_interactive_calibration():
    """
    Guides the user through the interactive calibration process, ensuring all
    three sensors (gyro, accel, mag) are fully calibrated.
    """
    if not sensor:
        print("WARNING: Cannot calibrate. Sensor not initialized.")
        return

    print("\n--- Starting Interactive Gyro Calibration ---")
    print("Please perform the following actions until each component is fully calibrated (3).")
    print("  - Gyroscope:      Keep the sensor perfectly still on a flat surface.")
    print("  - Accelerometer:  Slowly move the sensor into 6 different stable positions.")
    print("                    (e.g., flat, upside down, on each of its 4 edges).")
    print("                    Hold each position for a few seconds.")
    print("  - Magnetometer:   Make slow, large figure-eight motions in the air.")
    print("\nWaiting for full calibration...")

    while True:
        try:
            sys, gyro, accel, mag = sensor.calibration_status
            
            # Print the status
            print(f"\rCALIBRATION: Sys={sys} Gyro={gyro} Accel={accel} Mag={mag}", end="")

            # --- MODIFIED: Stricter condition ---
            # Wait for all three components to be fully calibrated.
            if gyro >= 3 and accel >= 3 and mag >= 3:
                # The System status (sys) will automatically become 3 once the
                # components are calibrated, so we don't need to check it explicitly.
                print("\n\nINFO: All components are fully calibrated!")
                break
            
            time.sleep(0.1)
        except Exception as e:
            print(f"\nError reading calibration status: {e}")
            time.sleep(1)

# The rest of your code (initialize, save_calibration, etc.) can remain the same.


def initialize():
    """
    Initializes the BNO055 sensor and attempts to load pre-saved calibration data.
    """
    global sensor
    if not config.GYRO_ENABLED:
        print("INFO: Gyro is disabled in config.")
        return True

    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        time.sleep(1) # Allow sensor to boot up
        #load_calibration()
        print(f"INFO: Gyro (BNO055) Initialized. Temp: {sensor.temperature}°C")
        print(f"INFO: Current calibration status: {sensor.calibration_status}")
        return True
    except Exception as e:
        print(f"WARNING: Gyro disabled. Could not initialize: {e}")
        sensor = None
        return True


def get_heading():
    """Returns the current yaw (heading) in degrees, or None if unavailable."""
    if sensor:
        try:
            heading, _, _ = sensor.euler
            if heading is not None:
                return heading
        except Exception:
            return None
    return None


def get_initial_heading(num_readings=20):
    """Gets an averaged, stable initial heading."""
    if not sensor:
        return 0.0

    print("INFO: Acquiring initial heading for gyro zero point...")
    readings = []
    for _ in range(num_readings):
        yaw = get_heading()
        if yaw is not None:
            readings.append(yaw)
        time.sleep(0.05)

    if readings:
        initial_heading = np.mean(readings)
        print(f"INFO: Gyro zero point set to: {initial_heading:.2f} degrees.")
        return initial_heading
    else:
        print("WARNING: Could not get initial gyro heading.")
        return 0.0


def cleanup():
    """No specific cleanup needed for this library."""
    print("--- Cleaning up Gyro (BNO055) ---")
    pass


if __name__ == "__main__":
    print("--- BNO055 Calibration Utility ---")
    if not initialize() or sensor is None:
        print("Calibration failed. Could not initialize sensor.")
    else:
        # Guide user through calibration
        perform_interactive_calibration()

        # Save the results
        save_calibration()

        print("\n--- Testing with new calibration ---")
        try:
            print("Reading gyro heading. Press Ctrl+C to exit.")
            while True:
                cal_status = sensor.calibration_status
                heading = get_heading()
                if heading is not None:
                    print(f"\rHeading: {heading:7.2f}° | Cal Status (S,G,A,M): {cal_status}", end="")
                else:
                    print("\rCould not read heading.", end="")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()
