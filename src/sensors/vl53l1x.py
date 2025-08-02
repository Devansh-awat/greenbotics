# src/sensors/vl53l1x.py
# Driver for the VL53L1X. Returns last known value ONLY if the sensor is busy.
# If the sensor reports an invalid reading (None), this function also returns None.

import time
import board
import busio
import adafruit_tca9548a
import adafruit_vl53l1x
from src.obstacle_challenge import config

# Module-level objects
i2c = None
mux = None
sensors = {}
# NEW: Dictionary to store the last known good distance for each channel
last_known_distances = {}

SENSOR_POSITIONS = {
    0: "Left  ",
    1: "Center",
    2: "Right "
}

def initialize():
    """Initializes sensors and the last_known_distances dictionary."""
    global i2c, mux, sensors, last_known_distances
    
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

    print(f"INFO: Probing for VL53L1X sensors on channels: {config.TOF_CHANNELS_TO_USE}...")
    for i in config.TOF_CHANNELS_TO_USE:
        try:
            sensor_obj = adafruit_vl53l1x.VL53L1X(mux[i])
            sensor_obj.distance_mode = 1
            sensor_obj.timing_budget = 50
            sensor_obj.start_ranging()
            sensors[i] = sensor_obj
            # Initialize the last known distance for this channel to None
            last_known_distances[i] = None
            print(f"  - SUCCESS: VL53L1X found and initialized on channel {i}.")
        except (ValueError, OSError) as e:
            sensors[i] = None
            print(f"  - ERROR: No VL53L1X sensor found on channel {i}. Check wiring. Error: {e}")

    print("INFO: VL53L1X Sensor initialization complete.")
    return True

def get_distance(channel):
    """
    NON-BLOCKING read. Returns distance in mm.
    If sensor.data_ready is False, returns last known value.
    If sensor.data_ready is True but reading is invalid (None), this also returns None.
    """
    if channel in sensors and sensors[channel] is not None:
        sensor = sensors[channel]
        
        if sensor.data_ready:
            try:
                distance_cm = sensor.distance
                sensor.clear_interrupt()
                
                # If the new reading is invalid (out of range), it's a real 'None'.
                # We must update our stored value and return None to signal wall loss.
                if distance_cm is None:
                    last_known_distances[channel] = None
                    return None
                
                # If the reading is valid, update our stored value and return it.
                new_distance_mm = distance_cm * 10.0
                last_known_distances[channel] = new_distance_mm
                return new_distance_mm
                
            except OSError:
                # On a communication error, it's safest to fall back to the old value.
                return last_known_distances.get(channel)
        else:
            # If data is not ready, the sensor is busy. Return the last known value.
            return last_known_distances.get(channel)
            
    return None # Sensor not initialized

def cleanup():
    """Stops ranging on all initialized sensors."""
    print("--- Cleaning up ToF Sensors (VL53L1X) ---")
    for sensor in sensors.values():
        if sensor is not None:
            sensor.stop_ranging()

# Test routine remains the same, as it doesn't use the non-blocking get_distance
if __name__ == "__main__":
    print("--- Testing VL53L1X Sensor Module ---")
    if not initialize():
        print("VL53L1X test failed during initialization.")
    else:
        # This test part would need a blocking version of get_distance to work well
        # We will leave it empty for now as it's not used by the main programs.
        print("Test complete. Run main programs to see sensor in action.")
        pass
    cleanup()
