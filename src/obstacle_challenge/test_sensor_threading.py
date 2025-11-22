# ...existing code...
from src.sensors import distance
import threading
import time
from src.sensors import bno055, distance
from src.obstacle_challenge.main_v3 import SensorThread

if __name__ == "__main__":
    sensors_initialized_event = threading.Event()
    sensor_thread = SensorThread(bno055, distance, sensors_initialized_event)
    sensor_thread.start()
    print("MainThread: Waiting for sensors to initialize...")
    sensors_initialized_event.wait() 
    print("MainThread: Sensors are ready. Proceeding with main logic.")  
    time.sleep(1)
    for _ in range(700):
        d0 = sensor_thread.get_readings()['distance_left']
        d3 = sensor_thread.get_readings()['distance_right']
        d2 = sensor_thread.get_readings()['distance_center']
        db = sensor_thread.get_readings()['distance_back']
        print(f"SensorThread: Distance readings - Left: {d0}, Center: {d2}, Right: {d3}, Back: {db}")
        time.sleep(0.01)
