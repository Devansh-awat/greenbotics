# src/obstacle_challenge/utils.py
import time
import cv2
import threading
import collections
import numpy as np


class FPSCounter:
    """A simple class to measure and display frames per second."""

    def __init__(self):
        self._start_time = None
        self._num_frames = 0
        self.fps = 0

    def start(self):
        """Starts the counter."""
        self._start_time = time.time()
        return self

    def update(self):
        """Updates the counter, must be called once per frame."""
        self._num_frames += 1

    def get_fps(self):
        """Calculates and returns the current FPS."""
        elapsed_time = time.time() - self._start_time
        # Avoid division by zero
        if elapsed_time > 0:
            self.fps = self._num_frames / elapsed_time
        return self.fps

    def display_on_frame(self, frame, alert_threshold=None):
        """
        Draws the FPS count on the top-left corner of a frame.
        Optionally changes color if FPS drops below a threshold.
        """
        self.get_fps()

        text = f"FPS: {self.fps:.2f}"
        text_color = (0, 255, 0)  # Green for good

        if alert_threshold and self.fps < alert_threshold:
            text_color = (0, 0, 255)  # Red for alert

        cv2.putText(
            frame,
            text,
            (10, 30),  # Position on the frame
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,  # Font scale
            text_color,
            2,  # Thickness
        )


class AsyncSensorReader:
    """
    Runs a sensor reading function in a separate thread to avoid blocking the main loop.
    This is ideal for I/O-bound tasks like reading from a ToF sensor.
    Includes new functionality for providing a rolling average of readings.
    """

    def __init__(self, reading_func, *args, averaging_window=1):
        """
        :param reading_func: The function to call to get a sensor reading.
        :param args: The arguments to pass to the reading_func (e.g., channel number).
        :param averaging_window: The number of readings to average. If 1, no averaging is done.
        """
        self._reading_func = reading_func
        self._args = args
        self._averaging_window = averaging_window
        
        if self._averaging_window > 1:
            # Use a deque for efficient, fixed-size storage of recent values
            self.latest_values = collections.deque(maxlen=self._averaging_window)
        else:
            self.latest_value = None

        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

    def _worker_loop(self):
        """The private function that runs in the background thread."""
        while self._running:
            value = self._reading_func(*self._args)

            with self._lock:
                if self._averaging_window > 1:
                    self.latest_values.append(value)
                else:
                    self.latest_value = value

            # Sleep to prevent this thread from hogging the CPU and to set sampling rate
            time.sleep(0.01) # Approx 100Hz sampling

    def read(self):
        """
        Returns the most recent value from the sensor.
        If averaging is enabled, returns the mean of the last N valid readings.
        This is a non-blocking call.
        """
        with self._lock:
            if self._averaging_window > 1:
                # Filter out any None values (e.g., from sensor errors) before averaging
                valid_readings = [v for v in self.latest_values if v is not None]
                if not valid_readings:
                    return None
                return np.mean(valid_readings)
            else:
                return self.latest_value

    def stop(self):
        """Signals the thread to stop and waits for it to finish."""
        print("Stopping async sensor reader thread...")
        self._running = False
        self._thread.join()
class SafetyMonitor:
    """
    Runs in a background thread to monitor the robot for unsafe conditions,
    like being picked up or tilted over.
    """
    def __init__(self, sensor_obj, tilt_threshold_deg):
        """
        :param sensor_obj: The actual adafruit_bno055 sensor object.
        :param tilt_threshold_deg: The angle in degrees to trigger the stop.
        """
        self._sensor = sensor_obj
        self._tilt_threshold = tilt_threshold_deg
        self._stop_event = threading.Event() # An event to signal an emergency stop
        self._running = True
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)

        if self._sensor:
            self._thread.start()
            print("INFO: Safety Monitor thread started.")
        else:
            print("WARNING: Gyro not available, Safety Monitor is disabled.")

    def _worker_loop(self):
        """The private function that runs in the background."""
        while self._running and not self._stop_event.is_set():
            try:
                # Read Euler angles: heading, roll, pitch
                _, roll, pitch = self._sensor.euler
                if roll is not None and pitch is not None:
                    # Check if the absolute roll or pitch exceeds the threshold
                    if abs(roll) > self._tilt_threshold or abs(pitch) > self._tilt_threshold:
                        print(f"\nFATAL: TILT DETECTED! Roll={roll:.1f}, Pitch={pitch:.1f}. TRIGGERING E-STOP.")
                        self._stop_event.set() # Set the event to signal a stop
                        break # Exit the loop
            except Exception:
                # Ignore read errors, we'll just try again
                pass
            time.sleep(0.05) # Check ~20 times per second

    def is_triggered(self):
        """Returns True if the emergency stop has been triggered."""
        return self._stop_event.is_set()

    def stop(self):
        """Signals the thread to stop and waits for it to finish."""
        self._running = False
        if self._thread.is_alive():
            self._thread.join()