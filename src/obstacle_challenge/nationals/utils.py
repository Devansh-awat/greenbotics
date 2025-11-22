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
        text_color = (0, 255, 0)

        if alert_threshold and self.fps < alert_threshold:
            text_color = (0, 0, 255)

        cv2.putText(
            frame,
            text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            text_color,
            2,
        )


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
        self._stop_event = threading.Event()
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
                _, roll, pitch = self._sensor.euler
                if roll is not None and pitch is not None:
                    if (
                        abs(roll) > self._tilt_threshold
                        or abs(pitch) > self._tilt_threshold
                    ):
                        print(
                            f"\nFATAL: TILT DETECTED! Roll={roll:.1f}, "
                            f"Pitch={pitch:.1f}. TRIGGERING E-STOP."
                        )
                        self._stop_event.set()
                        break
            except Exception:
                pass
            time.sleep(0.05)

    def is_triggered(self):
        """Returns True if the emergency stop has been triggered."""
        return self._stop_event.is_set()

    def stop(self):
        """Signals the thread to stop and waits for it to finish."""
        self._running = False
        if self._thread.is_alive():
            self._thread.join()
