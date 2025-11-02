# -*- coding: utf-8 -*-
"""
Unified library for reading distance from DFRobot_URM09, VL53L1X, and VL53L8CX sensors
via a TCA9548A I2C multiplexer or direct connection.
"""

import time
import board
import busio
import adafruit_tca9548a
import adafruit_vl53l1x
from adafruit_bus_device.i2c_device import I2CDevice

# --- ADD VL53L8CX SUPPORT ---
# Assumes vl53l8cx_python.py and libvl53l8cx_uld.so are in the same directory
try:
    from src.sensors.vl53l8cx_python import VL53L8CX, VL53L8CX_RESOLUTION_4X4, VL53L8CX_RESOLUTION_8X8
    VL53L8CX_AVAILABLE = True
except (ImportError, OSError) as e:
    print(f"Warning: Could not import VL53L8CX library. Support is disabled. Error: {e}")
    VL53L8CX_AVAILABLE = False
# --- END ADD ---


# --- Configuration ---
I2C_BUS_MAIN = 1 # The bus with the multiplexer
TCA_ADDRESS = 0x70
URM09_ADDRESS = 0x11
TOF_CHANNELS_TO_USE = range(8)

# --- Global Variables ---
_i2c_main = None
_mux = None
_sensors = {}
_sensor_types = {}


class DFRobot_URM09_IIC_CircuitPython:
    # --- This class is unchanged ---
    _CMD_DISTANCE_MEASURE = 0x01
    _DIST_H_INDEX = 3
    _CMD_INDEX = 8

    def __init__(self, i2c_bus_channel, addr=URM09_ADDRESS):
        self.i2c_device = I2CDevice(i2c_bus_channel, addr)
        self._write_reg(7, [0x20])

    def _write_reg(self, reg, data):
        buffer_to_write = bytearray([reg] + data)
        with self.i2c_device as i2c:
            i2c.write(buffer_to_write)

    def _read_reg(self, reg, length):
        result = bytearray(length)
        with self.i2c_device as i2c:
            i2c.write_then_readinto(bytearray([reg]), result)
        return list(result)

    def get_distance(self):
        try:
            self._write_reg(self._CMD_INDEX, [self._CMD_DISTANCE_MEASURE])
            time.sleep(0.03)
            rslt = self._read_reg(self._DIST_H_INDEX, 2)
            if not rslt or len(rslt) < 2: return None
            distance_cm = (rslt[0] << 8) + rslt[1]
            if distance_cm >= 32768: return None
            return distance_cm * 10
        except (IOError, IndexError, TypeError):
            return None


def initialise(i2c_bus_num=I2C_BUS_MAIN, mux_address=TCA_ADDRESS, urm09_address=URM09_ADDRESS):
    """
    Initializes the I2C bus, multiplexer, probes for sensors on all channels,
    and checks for a standalone VL53L8CX on I2C bus 2.
    """
    global _i2c_main, _mux, _sensors, _sensor_types

    try:
        _i2c_main = busio.I2C(board.SCL, board.SDA)
    except Exception as e:
        print(f"FATAL: Could not initialize I2C bus {i2c_bus_num}. Error: {e}")
        return False

    try:
        _mux = adafruit_tca9548a.TCA9548A(_i2c_main, address=mux_address)
        print(f"INFO: TCA9548A Mux found at address 0x{mux_address:02X}.")
    except ValueError:
        print(f"WARNING: TCA9548A Mux not found at address 0x{mux_address:02X}. Will not scan mux channels.")
        _mux = None

    if _mux:
        print(f"INFO: Probing for sensors on mux channels: {list(TOF_CHANNELS_TO_USE)}...")
        for i in TOF_CHANNELS_TO_USE:
            channel_bus = _mux[i]
            
            try:
                vl53_sensor = adafruit_vl53l1x.VL53L1X(channel_bus)
                vl53_sensor.distance_mode = 1
                vl53_sensor.timing_budget = 50
                vl53_sensor.start_ranging()
                _sensors[i] = vl53_sensor
                _sensor_types[i] = 'VL53L1X'
                print(f"  - SUCCESS: VL53L1X found and initialized on channel {i}.")
                continue
            except (ValueError, OSError):
                pass

            try:
                urm09_sensor = DFRobot_URM09_IIC_CircuitPython(channel_bus, addr=urm09_address)
                if urm09_sensor.get_distance() is not None:
                    _sensors[i] = urm09_sensor
                    _sensor_types[i] = 'URM09'
                    print(f"  - SUCCESS: URM09 found and initialized on channel {i}.")
                else:
                    print(f"  - INFO: No sensor found on channel {i}.")
            except Exception:
                print(f"  - INFO: No sensor found on channel {i}.")

    # --- NEW: Probe for standalone VL53L8CX on I2C bus 2 ---
    if VL53L8CX_AVAILABLE:
        print("INFO: Probing for standalone VL53L8CX on I2C bus 2...")
        try:
            # Initialize on bus 2 with channel -1 (no mux)
            # The C library must be modified to use "/dev/i2c-2"
            vl53l8cx_bus2_sensor = VL53L8CX()
            vl53l8cx_bus2_sensor.resolution = VL53L8CX_RESOLUTION_4X4
            vl53l8cx_bus2_sensor.start_ranging()
            _sensors[-1] = vl53l8cx_bus2_sensor
            _sensor_types[-1] = 'VL53L8CX'
            print("  - SUCCESS: VL53L8CX found on I2C bus 2 (assigned to channel -1).")
        except (IOError, OSError):
            print("  - INFO: No VL53L8CX found on I2C bus 2.")
    # --- END NEW ---

    print("INFO: Sensor initialization complete.")
    return True


def get_distance(channel):
    """
    Reads the distance from a sensor on a specific channel.
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
                return distance_cm * 10.0 if distance_cm is not None else None
            return None
            
        elif sensor_type == 'URM09':
            return sensor.get_distance()

        elif sensor_type == 'VL53L8CX':
            results = sensor.get_data()
            if results:
                # Target status 5 and 9 are valid measurements
                # For 4x4, zone 5 is one of the center zones
                center_zone_idx = 5
                if results.target_status[center_zone_idx] in [5, 9]:
                    return float(results.distance_mm[center_zone_idx])
            return None

    except (OSError, IOError):
        print(f"\nI/O Error on channel {channel}. Sensor may be disconnected.")
        if channel in _sensors:
            del _sensors[channel]
            del _sensor_types[channel]
        return None

    return None


def cleanup():
    """Stops ranging on all initialized ToF sensors."""
    print("\n--- Cleaning up Sensors ---")
    for channel, sensor in _sensors.items():
        if _sensor_types.get(channel) in ['VL53L1X', 'VL53L8CX']:
            try:
                sensor.stop_ranging()
            except (OSError, AttributeError):
                print(f"Warning: Error during cleanup of sensor on channel {channel}.")
    print("Cleanup complete.")

if __name__ == "__main__":
    print("--- Testing Distance Sensor Library ---")
    if not initialise():
        print("Test failed during initialization.")
    else:
        if not _sensors:
             print("No sensors were detected on any channel.")
        else:
            try:
                print("\nReading data from all detected sensors. Press Ctrl+C to stop.")
                while True:
                    output_line_parts = []
                    # sorted() will correctly handle channel -1
                    for i in sorted(_sensors.keys()):
                        dist_mm = get_distance(i)
                        type_str = _sensor_types.get(i, "N/A")
                        if dist_mm is not None:
                            # Format for channel -1 specifically if you want
                            ch_str = f"Ch{i}" if i != -1 else "Bus2"
                            output_line_parts.append(f"{ch_str} ({type_str}): {dist_mm:6.0f} mm")
                        else:
                            ch_str = f"Ch{i}" if i != -1 else "Bus2"
                            output_line_parts.append(f"{ch_str} ({type_str}):   ----   ")
                    
                    print(f"\r{(' | '.join(output_line_parts))}", end="", flush=True)
                    time.sleep(1/60)
            except KeyboardInterrupt:
                print("\nTest interrupted by user.")
            finally:
                cleanup()
