import requests
import time
import json

SERVER_URL = 'http://192.168.178.32:5000/telemetry'


def post_telemetry(server_url, timestampms, comp_angle, gyro_angle, accel_angle, control, frequency):
    try:
        requests.post(server_url,
                      json={'time': timestampms,
                            'comp_angle': comp_angle,
                            'gyro_angle': gyro_angle,
                            'accel_angle': accel_angle,
                            'control': control,
                            'frequency': frequency})
    except Exception as e:
        print(e)


def sync_with_telemetry_server_test():
    resp = requests.get('http://192.168.178.32:5000/sync')
    jsn = json.loads(resp.content)
    return jsn


# post_telemetry(SERVER_URL, time.time(), 1, 2, 3, 4, 5)

print(sync_with_telemetry_server_test())