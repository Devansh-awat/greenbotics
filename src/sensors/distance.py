# -*- coding: utf-8 -*-
"""
Unified library for reading distance from DFRobot_URM09 and VL53L1X sensors
via a TCA9548A I2C multiplexer.
TODO
Add support for VL53L8CX
"""

import time
import board
import busio
import adafruit_tca9548a
import adafruit_vl53l1x
from adafruit_bus_device.i2c_device import I2CDevice

# --- Configuration ---
# These are the default values. They can be overridden in the initialise() function.
I2C_BUS = 1  # This is handled by board.SCL/SDA, but kept for context.
TCA_ADDRESS = 0x70
URM09_ADDRESS = 0x11
TOF_CHANNELS_TO_USE = range(8)  # By default, check all 8 channels

# --- Global Variables ---
_i2c = None
_mux = None
_sensors = {}
_sensor_types = {}


class DFRobot_URM09_IIC_CircuitPython:
    """
    CircuitPython-compatible I2C driver for the DFRobot URM09 sensor.
    Uses the adafruit_bus_device library for I2C communication, making it
    compatible with the adafruit_tca9548a multiplexer library.
    """
    _CMD_DISTANCE_MEASURE = 0x01
    _DIST_H_INDEX = 3
    _CMD_INDEX = 8

    def __init__(self, i2c_bus_channel, addr=URM09_ADDRESS):
        """
        :param i2c_bus_channel: An I2C bus object, e.g., from mux[i].
        :param addr: The I2C address of the sensor.
        """
        self.i2c_device = I2CDevice(i2c_bus_channel, addr)
        self._write_reg(7, [0x20])

    def _write_reg(self, reg, data):
        """Writes a list of bytes to a register."""
        buffer_to_write = bytearray([reg] + data)
        with self.i2c_device as i2c:
            i2c.write(buffer_to_write)

    def _read_reg(self, reg, length):
        """Reads a number of bytes from a register."""
        result = bytearray(length)
        with self.i2c_device as i2c:
            # First, write the register address we want to read from, then read.
            i2c.write_then_readinto(bytearray([reg]), result)
        return list(result) # Return as list for compatibility

    def get_distance(self):
        """Commands a measurement and reads the distance, converting it to millimeters."""
        try:
            # Command a measurement
            self._write_reg(self._CMD_INDEX, [self._CMD_DISTANCE_MEASURE])
            # Per datasheet/example, a delay is needed for measurement to complete
            time.sleep(0.1)
            # Read the 2-byte distance register
            rslt = self._read_reg(self._DIST_H_INDEX, 2)
            if not rslt or len(rslt) < 2:
                return None
            # The sensor returns the value in centimeters.
            distance_cm = (rslt[0] << 8) + rslt[1]

            # Per original code, values over 32767 are overflow/invalid
            if distance_cm >= 32768:
                return None
            
            # Convert from cm to mm before returning
            return distance_cm * 10
        except (IOError, IndexError, TypeError):
            # Catch I/O errors if sensor disconnects, or indexing errors
            return None


def initialise(i2c_bus_num=None, mux_address=TCA_ADDRESS, urm09_address=URM09_ADDRESS):
    """
    Initializes the I2C bus, multiplexer, and probes for sensors on all channels.

    :param i2c_bus_num: Not directly used, kept for compatibility. board.SCL/SDA are used.
    :param mux_address: The I2C address of the TCA9548A multiplexer (default: 0x70).
    :param urm09_address: The I2C address for URM09 sensors (default: 0x11).
    :return: True if initialization was successful, False otherwise.
    """
    global _i2c, _mux, _sensors, _sensor_types

    try:
        # Use board-defined I2C pins
        _i2c = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        print(f"FATAL: Could not initialize I2C bus. Error: {e}")
        return False

    try:
        _mux = adafruit_tca9548a.TCA9548A(_i2c, address=mux_address)
    except ValueError:
        print(f"FATAL: TCA9548A Mux not found at address 0x{mux_address:02X}.")
        return False

    print(f"INFO: Probing for sensors on channels: {list(TOF_CHANNELS_TO_USE)}...")
    for i in TOF_CHANNELS_TO_USE:
        # Each channel on the mux is treated as its own I2C bus.
        channel_bus = _mux[i]

        # First, try to initialize as a VL53L1X sensor
        try:
            vl53_sensor = adafruit_vl53l1x.VL53L1X(channel_bus)
            vl53_sensor.distance_mode = 1
            vl53_sensor.timing_budget = 100
            vl53_sensor.start_ranging()
            _sensors[i] = vl53_sensor
            _sensor_types[i] = 'VL53L1X'
            print(f"  - SUCCESS: VL53L1X found and initialized on channel {i}.")
            continue  # Sensor found, move to next channel
        except (ValueError, OSError):
            # This is expected if a VL53L1X isn't on this channel
            pass

        # If not a VL53L1X, try to initialize as a URM09 sensor
        try:
            # Use the CircuitPython-compatible driver
            urm09_sensor = DFRobot_URM09_IIC_CircuitPython(channel_bus, addr=urm09_address)
            # Try to get a reading to confirm it's a working URM09
            if urm09_sensor.get_distance() is not None:
                _sensors[i] = urm09_sensor
                _sensor_types[i] = 'URM09'
                print(f"  - SUCCESS: URM09 found and initialized on channel {i}.")
            else:
                 print(f"  - INFO: No sensor found on channel {i}.")
        except Exception:
             print(f"  - INFO: No sensor found on channel {i}.")

    print("INFO: Sensor initialization complete.")
    return True


def get_distance(channel):
    """
    Reads the distance from a sensor on a specific channel.

    :param channel: The multiplexer channel (0-7).
    :return: Distance in millimeters, or None if no sensor or invalid reading.
    """
    if channel not in _sensors:
        return None

    sensor = _sensors[channel]
    sensor_type = _sensor_types[channel]

    try:
        if sensor_type == 'VL53L1X':
            if sensor.data_ready:
                distance_cm = sensor.distance
                sensor.clear_interrupt()
                # The sensor can return None for distance
                return distance_cm * 10.0 if distance_cm is not None else None
            # Return None if not ready to avoid stale data
            return None
        elif sensor_type == 'URM09':
            # The get_distance method for URM09 includes the read command and delay
            return sensor.get_distance()
    except (OSError, IOError):
        # Catch errors if sensor is disconnected during operation
        return None

    return None


def cleanup():
    """Stops ranging on all initialized VL53L1X sensors."""
    print("\n--- Cleaning up Sensors ---")
    for channel, sensor in _sensors.items():
        if _sensor_types.get(channel) == 'VL53L1X':
            try:
                sensor.stop_ranging()
            except OSError:
                print(f"Warning: I/O error during cleanup of sensor on channel {channel}.")
    print("Cleanup complete.")

if __name__ == "__main__":
    print("--- Testing Distance Sensor Library ---")
    if not initialise():
        print("Test failed during initialization.")
    else:
        # Check if any sensors were actually found
        if not _sensors:
             print("No sensors were detected on any channel.")
        else:
            try:
                print("\nReading data from all detected sensors. Press Ctrl+C to stop.")
                while True:
                    output_line_parts = []
                    # Iterate through sorted channels for consistent output
                    for i in sorted(_sensors.keys()):
                        dist_mm = get_distance(i)
                        type_str = _sensor_types.get(i, "N/A")
                        if dist_mm is not None:
                            output_line_parts.append(f"Ch{i} ({type_str}): {dist_mm:6.0f} mm")
                        else:
                            output_line_parts.append(f"Ch{i} ({type_str}):   ----   ")
                    
                    print(f"\r{(' | '.join(output_line_parts))}", end="")
                    time.sleep(0.2) # A small delay to make output readable
            except KeyboardInterrupt:
                print("\nTest interrupted by user.")
            finally:
                cleanup()
