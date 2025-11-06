# -*- coding: utf-8 -*-
"""
Python ctypes wrapper for the VL53L8CX ULD C library.
This file bridges the gap between the C shared library and the Python script
for a sensor connected directly to an I2C bus.

When run directly, this script will perform a test to display ambient light
readings from the sensor in a 4x4 grid.
"""

import ctypes
import os
import time

# --- Constants from the C headers ---
VL53L8CX_RESOLUTION_4X4 = 16
VL53L8CX_RESOLUTION_8X8 = 64
VL53L8CX_NB_TARGET_PER_ZONE = 1
VL53L8CX_TEMPORARY_BUFFER_SIZE = 1024
VL53L8CX_OFFSET_BUFFER_SIZE = 488
VL53L8CX_XTALK_BUFFER_SIZE = 776


# --- C Structure Definitions for ctypes ---

class VL53L8CX_Platform(ctypes.Structure):
    _fields_ = [
        ('address', ctypes.c_uint16),
        ('fd', ctypes.c_int),
    ]

class VL53L8CX_Configuration(ctypes.Structure):
    _fields_ = [
        ('platform', VL53L8CX_Platform),
        ('streamcount', ctypes.c_uint8),
        ('data_read_size', ctypes.c_uint32),
        ('default_configuration', ctypes.POINTER(ctypes.c_uint8)),
        ('default_xtalk', ctypes.POINTER(ctypes.c_uint8)),
        ('offset_data', ctypes.c_uint8 * VL53L8CX_OFFSET_BUFFER_SIZE),
        ('xtalk_data', ctypes.c_uint8 * VL53L8CX_XTALK_BUFFER_SIZE),
        ('temp_buffer', ctypes.c_uint8 * VL53L8CX_TEMPORARY_BUFFER_SIZE),
        ('is_auto_stop_enabled', ctypes.c_uint8),
        ('crc_checksum_for_results_pkt', ctypes.c_uint8),
    ]

class VL53L8CX_ResultsData(ctypes.Structure):
    _fields_ = [
        ('silicon_temp_degc', ctypes.c_int8),
        ('ambient_per_spad', ctypes.c_uint32 * 64),
        ('nb_target_detected', ctypes.c_uint8 * 64),
        ('nb_spads_enabled', ctypes.c_uint32 * 64),
        ('signal_per_spad', ctypes.c_uint32 * (64 * VL53L8CX_NB_TARGET_PER_ZONE)),
        ('range_sigma_mm', ctypes.c_uint16 * (64 * VL53L8CX_NB_TARGET_PER_ZONE)),
        ('distance_mm', ctypes.c_int16 * (64 * VL53L8CX_NB_TARGET_PER_ZONE)),
        ('reflectance', ctypes.c_uint8 * (64 * VL53L8CX_NB_TARGET_PER_ZONE)),
        ('target_status', ctypes.c_uint8 * (64 * VL53L8CX_NB_TARGET_PER_ZONE)),
        ('motion_indicator', ctypes.c_uint8 * 144)
    ]


# --- Python Wrapper Class ---

class VL53L8CX:
    """
    Python class for the VL53L8CX sensor, wrapping the ULD C library
    for a direct I2C connection.
    """
    def __init__(self, i2c_bus_path="/dev/i2c-2"):
        """
        Initializes the sensor by loading the C library and connecting to the specified I2C bus.
        
        NOTE: The C library's platform.c file must be compiled to use this same bus path.
        
        :param i2c_bus_path: The device path for the C library to open (e.g., "/dev/i2c-2").
        """
        self.i2c_bus_path_str = i2c_bus_path # Store for user feedback
        
        lib_path = os.path.join(os.path.dirname(__file__), 'libvl53l8cx_uld.so')
        self.uld_lib = ctypes.CDLL(lib_path)
        self._define_c_functions()
        
        self.dev = VL53L8CX_Configuration()
        self.p_dev = ctypes.byref(self.dev)

        self._is_ranging = False

        status = self.uld_lib.vl53l8cx_comms_init(self.p_dev)
        if status != 0:
            raise IOError(f"C lib failed to open I2C bus: {self.i2c_bus_path_str}. Check C code & permissions.")

        is_alive = ctypes.c_uint8(0)
        status = self.uld_lib.vl53l8cx_is_alive(self.p_dev, ctypes.byref(is_alive))
        if status != 0 or is_alive.value == 0:
            self.uld_lib.vl53l8cx_comms_close(self.p_dev)
            raise IOError(f"VL53L8CX sensor not found on bus {self.i2c_bus_path_str}.")
        
        is_firmware_running = self.uld_lib.vl53l8cx_is_firmware_running(self.p_dev)
        
        if is_firmware_running == 1:
            print("INFO: VL53L8CX firmware already running. Performing fast re-init.")
            # Stop any ranging that might be active from a previous run
            self.stop_ranging()
            # Reset streamcount to ensure get_data() works correctly
            self.dev.streamcount = 255
            status = 0 # Success
        else:
            print("INFO: VL53L8CX firmware not detected. Performing full, one-time initialization...")
            status = self.uld_lib.vl53l8cx_init(self.p_dev)

        if status != 0:
            self.uld_lib.vl53l8cx_comms_close(self.p_dev)
            raise IOError(f"Failed to initialize VL53L8CX sensor, status {status}")

        self.set_ranging_frequency_hz(60)

        self._resolution = VL53L8CX_RESOLUTION_4X4

    def _define_c_functions(self):
        """ Set argtypes and restype for all C functions we will call. """
        # ... (This function is unchanged) ...
        self.uld_lib.vl53l8cx_is_alive.argtypes = [ctypes.POINTER(VL53L8CX_Configuration), ctypes.POINTER(ctypes.c_uint8)]
        self.uld_lib.vl53l8cx_is_alive.restype = ctypes.c_uint8
        self.uld_lib.vl53l8cx_is_firmware_running.argtypes = [ctypes.POINTER(VL53L8CX_Configuration)]
        self.uld_lib.vl53l8cx_is_firmware_running.restype = ctypes.c_uint8
        self.uld_lib.vl53l8cx_init.argtypes = [ctypes.POINTER(VL53L8CX_Configuration)]
        self.uld_lib.vl53l8cx_init.restype = ctypes.c_uint8
        self.uld_lib.vl53l8cx_start_ranging.argtypes = [ctypes.POINTER(VL53L8CX_Configuration)]
        self.uld_lib.vl53l8cx_start_ranging.restype = ctypes.c_uint8
        self.uld_lib.vl53l8cx_stop_ranging.argtypes = [ctypes.POINTER(VL53L8CX_Configuration)]
        self.uld_lib.vl53l8cx_stop_ranging.restype = ctypes.c_uint8
        self.uld_lib.VL53L8CX_wait_for_dataready.argtypes = [ctypes.POINTER(VL53L8CX_Platform)]
        self.uld_lib.VL53L8CX_wait_for_dataready.restype = ctypes.c_uint8
        self.uld_lib.vl53l8cx_get_ranging_data.argtypes = [ctypes.POINTER(VL53L8CX_Configuration), ctypes.POINTER(VL53L8CX_ResultsData)]
        self.uld_lib.vl53l8cx_get_ranging_data.restype = ctypes.c_uint8
        self.uld_lib.vl53l8cx_set_resolution.argtypes = [ctypes.POINTER(VL53L8CX_Configuration), ctypes.c_uint8]
        self.uld_lib.vl53l8cx_set_resolution.restype = ctypes.c_uint8
        self.uld_lib.vl53l8cx_comms_init.argtypes = [ctypes.POINTER(VL53L8CX_Configuration)]
        self.uld_lib.vl53l8cx_comms_init.restype = ctypes.c_int32
        self.uld_lib.vl53l8cx_comms_close.argtypes = [ctypes.POINTER(VL53L8CX_Configuration)]
        self.uld_lib.vl53l8cx_comms_close.restype = ctypes.c_int32

    @property
    def resolution(self):
        return self._resolution

    @resolution.setter
    def resolution(self, value):
        if value not in [VL53L8CX_RESOLUTION_4X4, VL53L8CX_RESOLUTION_8X8]:
            raise ValueError("Invalid resolution. Use VL53L8CX_RESOLUTION_4X4 or VL53L8CX_RESOLUTION_8X8.")
        status = self.uld_lib.vl53l8cx_set_resolution(self.p_dev, value)
        if status != 0:
            raise IOError(f"Failed to set resolution, status {status}")
        self._resolution = value

    def set_ranging_frequency_hz(self, frequency):
        status = self.uld_lib.vl53l8cx_set_ranging_frequency_hz(self.p_dev, int(frequency))
        if status != 0:
            raise IOError(f"Failed to set ranging frequency, status {status}")
        
    def start_ranging(self):
        status = self.uld_lib.vl53l8cx_start_ranging(self.p_dev)
        if status != 0:
            raise IOError(f"Failed to start ranging, status {status}")
        self._is_ranging = True
    
    def stop_ranging(self):
        if self._is_ranging:
            status = self.uld_lib.vl53l8cx_stop_ranging(self.p_dev)
            if status != 0:
                print(f"Warning: Failed to stop ranging cleanly, status {status}")
            self._is_ranging = False
    
    def get_data(self):
        results = VL53L8CX_ResultsData()
        is_ready = self.uld_lib.VL53L8CX_wait_for_dataready(ctypes.byref(self.dev.platform))
        if is_ready:
            status = self.uld_lib.vl53l8cx_get_ranging_data(self.p_dev, ctypes.byref(results))
            if status == 0:
                return results
        return None

    def __del__(self):
        try:
            self.stop_ranging()
            self.uld_lib.vl53l8cx_comms_close(self.p_dev)
        except (IOError, AttributeError):
            pass

# --- MAIN EXECUTION BLOCK ---
if __name__ == "__main__":
    # --- Configuration for the standalone test ---
    I2C_BUS_PATH = "/dev/i2c-2" # IMPORTANT: This must match the path compiled into your C library
    TEST_RESOLUTION = VL53L8CX_RESOLUTION_4X4
    
    print("--- VL53L8CX Ambient Light Test (Direct Connection) ---")
    print(f"Attempting to initialize sensor on I2C bus: {I2C_BUS_PATH}...")

    sensor = None
    try:
        sensor = VL53L8CX(i2c_bus_path=I2C_BUS_PATH)
        sensor.resolution = TEST_RESOLUTION
        sensor.start_ranging()
        
        print(f"Sensor initialized. Now reading ambient light data (in kcps/spad).")
        print("Press Ctrl+C to stop.")
        
        grid_size = int(TEST_RESOLUTION**0.5)
        
        while True:
            results = sensor.get_data()
            if results:
                print(f"Frame: {sensor.dev.streamcount}, Temp: {results.silicon_temp_degc}°C")
                for y in range(grid_size):
                    line = " | "
                    for x in range(grid_size):
                        index = y * grid_size + x
                        ambient_kcps = results.ambient_per_spad[index] / 2048.0
                        line += f"{ambient_kcps:6.2f}"
                    line += " |"
                    print(line)
                
                print(f"\033[F" * (grid_size + 1), end="")
                
            time.sleep(0.1)

    except KeyboardInterrupt:
        print("\nStopping test.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        if sensor:
            print("\nStopping ranging...")
            sensor.stop_ranging()
