import requests


def post_telemetry(server_url, timestampms, comp_angle, gyro_angle, accel_angle, control, frequency):
    requests.post(server_url,
                  data={'time': timestampms,
                        'comp_angle': comp_angle,
                        'gyro_angle': gyro_angle,
                        'accel_angle': accel_angle,
                        'control': control,
                        'frequency': frequency})
