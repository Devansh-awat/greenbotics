# src/obstacle_challenge/utils.py
import time
import cv2
import threading


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
    """

    def __init__(self, reading_func, *args):
        """
        :param reading_func: The function to call to get a sensor reading (e.g., vl53l0x.get_distance)
        :param args: The arguments to pass to the reading_func (e.g., the channel number)
        """
        self.latest_value = None
        self._reading_func = reading_func
        self._args = args
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()

    def _worker_loop(self):
        """The private function that runs in the background thread."""
        while self._running:
            # Call the provided sensor function
            value = self._reading_func(*self._args)

            # Use the lock to safely update the shared value
            with self._lock:
                self.latest_value = value

            # Sleep for a tiny amount to prevent this thread from hogging the CPU
            time.sleep(0.01)

    def read(self):
        """Returns the most recent value from the sensor. This is non-blocking."""
        with self._lock:
            return self.latest_value

    def stop(self):
        """Signals the thread to stop and waits for it to finish."""
        print("Stopping async sensor reader thread...")
        self._running = False
        self._thread.join()
