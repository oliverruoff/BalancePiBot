import time
import requests
import json
import threading


class transmittor:

    def __init__(self, server_url, min_transmission_time_seconds):
        self.server_url = server_url
        self.min_transmission_time_seconds = min_transmission_time_seconds
        self.last_transmission_time = time.time()
        self.telemetry_list = []

    def post_telemetry(self):
        try:
            requests.post(self.server_url,
                          json=json.dumps(self.telemetry_list))
        except Exception as e:
            print(e)

    def collect_telemetry(self, comp_angle, gyro_angle, accel_angle, control, frequency):
        now = time.time()
        data = {
            'time': now,
            'comp_angle': comp_angle,
            'gyro_angle': gyro_angle,
            'accel_angle': accel_angle,
            'control': control,
            'frequency': frequency
        }
        self.telemetry_list.append(data)
        if (now - self.last_transmission_time) >= self.min_transmission_time_seconds:
            print('Telemetry data sent!')
            t1 = threading.Thread(target=self.post_telemetry)
            t1.start()
            self.last_transmission_time = now
            self.telemetry_list = []
