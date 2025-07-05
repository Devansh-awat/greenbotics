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
    if not config.TOF_ENABLED:
        print("INFO: ToF sensors are disabled in config.")
        return True

    try:
        i2c = busio.I2C(board.SCL, board.SDA)
        mux = adafruit_tca9548a.TCA9548A(i2c)
    except Exception as e:
        print(f"FATAL: Error initializing I2C bus or Mux: {e}")
        return False
        
    print("Initializing ToF sensors...")
    for i in config.TOF_CHANNELS_TO_USE:
        try:
            sensor_obj = adafruit_vl53l0x.VL53L0X(mux[i])
            
            # Use a responsive timing budget. 33ms allows for ~30Hz updates
            # from the background thread, which is a great balance.
            sensor_obj.measurement_timing_budget = 33000
            
            sensors[i] = sensor_obj
            print(f"  - Sensor on channel {i} initialized with 33ms budget.")
        except Exception:
            print(f"  - Error: Sensor not found on channel {i}. Skipping.")
            sensors[i] = None

    if not any(sensors.values()):
        print("WARNING: No ToF sensors were successfully initialized.")
    
    print("INFO: ToF Sensors Initialized.")
    return True

def get_distance(channel):
    """
    Gets a single, RAW distance reading in mm from a sensor.
    This is a simple, blocking function meant to be called by the async reader.
    """
    if channel in sensors and sensors[channel] is not None:
        try:
            # The range property performs a measurement
            return sensors[channel].range
        except Exception:
            # Catch potential I/O errors during read
            return None
    return None

def cleanup():
    """No specific cleanup needed for these libraries."""
    print("--- Cleaning up ToF Sensors ---")
    pass

# Test routine to check hardware and raw sensor values
if __name__ == "__main__":
    print("--- Testing ToF Sensor Module (Raw Readings) ---")
    if not initialize():
        print("ToF test failed during initialization.")
    else:
        try:
            print("\nReading all configured sensors. Press Ctrl+C to stop.")
            while True:
                output_line = ""
                for i in sorted(sensors.keys()):
                    dist = get_distance(i)
                    if dist is not None:
                        output_line += f"Ch{i}: {dist:4} mm | "
                    else:
                        output_line += f"Ch{i}:  ----  | "
                print(f"\r{output_line}", end="")
                time.sleep(0.1) # Poll at 10Hz for the test
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()