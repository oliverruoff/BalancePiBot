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
        self.config_dict = {"Kp": 50, "Ki": 0, "Kd": 0.1}

    def get_configs(self):
        return self.config_dict

    def post_telemetry(self, endpoint='/telemetry'):
        try:
            requests.post('/'.join([self.server_url, endpoint]),
                          json=self.telemetry_list)
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

    def start_config_synchronization(self):
        t1 = threading.Thread(target=self.sync_with_telemetry_server)
        t1.start()

    def sync_with_telemetry_server(self, endpoint='sync'):
        print('Syncing with telemetry server')
        resp = requests.get(url='/'.join([self.server_url, endpoint]))
        jsn = json.loads(resp.content)
        self.config_dict = jsn
        time.sleep(1)
        # self.sync_with_telemetry_server()
