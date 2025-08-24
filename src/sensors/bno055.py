import board
import adafruit_bno055
import time
import numpy as np
from src.obstacle_challenge import config


sensor = None


def initialize():
    """Initializes the BNO055 sensor."""
    global sensor
    if not config.GYRO_ENABLED:
        print("INFO: Gyro is disabled in config.")
        return True

    try:
        i2c = board.I2C()
        sensor = adafruit_bno055.BNO055_I2C(i2c)
        time.sleep(1)
        print(f"INFO: Gyro (BNO055) Initialized. Temp: {sensor.temperature}°C")
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
    print("--- Testing BNO055 Gyro Module ---")
    if not initialize() or sensor is None:
        print("Gyro test failed. Is it connected and enabled in config?")
    else:
        try:
            print("Reading gyro heading. Press Ctrl+C to exit.")
            while True:
                heading = get_heading()
                if heading is not None:
                    print(f"\rCurrent Heading: {heading:7.2f}°", end="")
                else:
                    print("\rCould not read heading.", end="")
                time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nTest interrupted by user.")
        finally:
            cleanup()
