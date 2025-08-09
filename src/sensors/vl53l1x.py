# src/sensors/vl53l1x.py
# New robust version with a filter for anomalous zero readings.

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
# Dictionary to store the last known good distance for each channel
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
            sensor_obj.timing_budget = 33
            sensor_obj.start_ranging()
            sensors[i] = sensor_obj
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
    If sensor is busy, returns last known value.
    If sensor reports an invalid reading (None or 0.0), this also returns None.
    """
    if channel in sensors and sensors[channel] is not None:
        sensor = sensors[channel]
        
        if sensor.data_ready:
            try:
                distance_cm = sensor.distance
                sensor.clear_interrupt()
                
                # NEW: Treat a reading of 0.0 as an invalid reading, just like None.
                if distance_cm is None or distance_cm == 0.0:
                    last_known_distances[channel] = None
                    return None
                
                # If the reading is valid, update our stored value and return it.
                new_distance_mm = distance_cm * 10.0
                last_known_distances[channel] = new_distance_mm
                return new_distance_mm
                
            except OSError:
                return last_known_distances.get(channel)
        else:
            # If data is not ready, the sensor is busy. Return the last known value.
            return last_known_distances.get(channel)
            
    return None

def get_new_distance(channel, timeout=1.0):
    """
    BLOCKING read that waits for a fresh sensor value. For testing.
    Returns distance in CENTIMETERS (cm), or None on error/timeout.
    """
    if channel in sensors and sensors[channel] is not None:
        sensor = sensors[channel]
        start_time = time.monotonic()
        while not sensor.data_ready:
            if time.monotonic() - start_time > timeout:
                return None
        try:
            distance_cm = sensor.distance
            sensor.clear_interrupt()
            last_known_distances[channel] = distance_cm * 10.0 if distance_cm else None
            return distance_cm
        except OSError:
            return None
    return None

def cleanup():
    """Stops ranging on all initialized sensors."""
    print("--- Cleaning up ToF Sensors (VL53L1X) ---")
    for sensor in sensors.values():
        if sensor is not None:
            try:
                sensor.stop_ranging()
            except OSError:
                print("Warning: I/O error during sensor cleanup. Ignoring.")

if __name__ == "__main__":
    print("--- Testing VL53L1X Sensor Module (Blocking Read for Test) ---")
    if not initialize():
        print("VL53L1X test failed during initialization.")
    else:
        try:
            print("\nReading data from all configured sensors. Press Ctrl+C to stop.")
            while True:
                output_line = ""
                sensor_order = [ch for ch in config.TOF_CHANNELS_TO_USE if sensors.get(ch)]

                for i in sensor_order:
                    dist_cm = get_new_distance(i)
                    pos_name = SENSOR_POSITIONS.get(i, f"Ch{i}")

                    if dist_cm is not None:
                        output_line += f"{pos_name}: {dist_cm:6.1f} cm | "
                    else:
                        output_line += f"{pos_name}:   ----   | "
                
                print(f"\r{output_line}", end="")
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()
