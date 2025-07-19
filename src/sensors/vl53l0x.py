# src/sensors/vl53l0x.py
import time
import board
import busio
import adafruit_tca9548a
import adafruit_vl53l0x
from src.obstacle_challenge import config

# Module-level objects
i2c = None
mux = None
sensors = {}


def initialize():
    """Initializes the I2C bus, multiplexer, and all specified ToF sensors."""
    global i2c, mux, sensors

    try:
        i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        print(f"FATAL: Could not initialize I2C bus. Error: {e}")
        return False

    try:
        mux = adafruit_tca9548a.TCA9548A(i2c)
    except ValueError:
        print("FATAL: TCA9548A Mux not found on I2C bus. Run 'sudo i2cdetect -y 1'")
        return False

    print(f"INFO: Probing for ToF sensors on channels: {config.TOF_CHANNELS_TO_USE}...")
    for i in config.TOF_CHANNELS_TO_USE:
        try:
            sensor_obj = adafruit_vl53l0x.VL53L0X(mux[i])
            sensor_obj.measurement_timing_budget = 33000
            sensors[i] = sensor_obj
            print(f"  - SUCCESS: Sensor found and initialized on channel {i}.")
        except (ValueError, OSError):
            print(
                f"  - ERROR: No sensor found on channel {i}. Check wiring to the mux."
            )
            sensors[i] = None

    print("INFO: ToF Sensor initialization complete.")
    return True


def get_distance(channel):
    """
    Gets a single distance reading in mm from a sensor.
    Includes filtering for the 'Out of Range' value (8191).
    """
    if channel in sensors and sensors[channel] is not None:
        try:
            raw_range = sensors[channel].range
            if raw_range == 8191 or raw_range == 8190:
                return None
            return raw_range
        except OSError as e:
            print(f"\nWARN: I/O Error reading sensor on channel {channel}: {e}")
            return None
    return None


def cleanup():
    """No specific cleanup needed, but good practice to have."""
    print("--- Cleaning up ToF Sensors ---")
    pass


# Test routine to check hardware and raw sensor values
if __name__ == "__main__":
    print("--- Testing ToF Sensor Module (with 8191 filtering) ---")

    if not initialize():
        print("ToF test failed during initialization.")
    else:
        try:
            print("\nReading all configured sensors. Press Ctrl+C to stop.")
            while True:
                output_line = ""
                for i in config.TOF_CHANNELS_TO_USE:
                    dist = get_distance(i)
                    if dist is not None:
                        output_line += f"Ch{i}: {dist:4} mm | "
                    else:
                        output_line += f"Ch{i}:  ----  | "
                print(f"\r{output_line}", end="")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()
