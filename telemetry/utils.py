import numpy as np
import pandas as pd
import time


def safe_arange(start, stop, step):
    return step * np.arange(start / step, stop / step)


def generate_empty_feature_df_with_size(last_n_seconds, features=['comp_angle', 'gyro_angle', 'accel_angle', 'control', 'frequency']):
    start = int(time.time()-last_n_seconds)
    time_steps = safe_arange(start, start+last_n_seconds, 0.005)
    time_steps = [float('.'.join(
        [str(i).split('.')[0], str(i).split('.')[1][:3]])) for i in time_steps]
    time_step_dict = [{'time': t} for t in time_steps]
    time_step_df = pd.DataFrame(time_step_dict)

    prepared_dict_list = []
    for idx, row in time_step_df.iterrows():
        for f in features:
            prepared_dict_list.append(
                {'time': row.time, 'feature': f, 'value': 0})
    prepared_df = pd.DataFrame(prepared_dict_list)
    return prepared_df


def transform_df_to_single_features(df):
    prepared_data = []
    for idx, line in df.iterrows():
        if line['frequency'] > 1000:
            continue
        prepared_data.append({
            'time': line['time_step'],
            'value': line['frequency'],
            'feature': 'frequency'
        })
        prepared_data.append({
            'time': line['time_step'],
            'value': line['comp_angle'],
            'feature': 'comp_angle'
        })
        prepared_data.append({
            'time': line['time_step'],
            'value': line['control'],
            'feature': 'control'
        })
        prepared_data.append({
            'time': line['time_step'],
            'value': line['gyro_angle'],
            'feature': 'gyro_angle'
        })
        prepared_data.append({
            'time': line['time_step'],
            'value': line['accel_angle'],
            'feature': 'accel_angle'
        })
    return pd.DataFrame(prepared_data)


def prepare_input_data(old_df: pd.DataFrame, data: dict, last_n_seconds: int, moving_window_size):
    """Taking data dict from received json. 
    - Generating DataFrame out of it
    - Concatening it with old DataFrame
    - Takes last n (moving_windows_size) elements of the concatted dataframe
    - Applying defined time grid on the dataframe, so that plotly has no problems to animate it.
    - Changing column structure to be able to display it on graphs

    Args:
        old_df (pd.DataFrame): Old prepared DataFrame
        data (dict): new received json parsed telemetry object
        moving_window_size (int): Last n elements to take for new concatted dataframe
    """

    telemetry_df = pd.DataFrame(data)
    # creating rounded time column
    telemetry_df['time_step'] = telemetry_df.apply(
        lambda row: round(row.time, 2), axis=1)

    # creating new DataFrame containing stepped time column
    start = int(max(telemetry_df.time)-last_n_seconds)
    time_steps = safe_arange(start, start+last_n_seconds, 0.005)
    time_steps = [float('.'.join(
        [str(i).split('.')[0], str(i).split('.')[1][:3]])) for i in time_steps]
    time_step_dict = [{'time_step': t} for t in time_steps]
    time_step_df = pd.DataFrame(time_step_dict)

    telemetry_df = pd.merge(time_step_df, telemetry_df, on="time_step", how="left").drop_duplicates(
        subset=['time_step']).interpolate()
    print('len_Df123:', len(telemetry_df))
    telemetry_df.dropna(subset=["time"], inplace=True)
    print('len_Df$%&:', len(telemetry_df))

    # Bringing dataframe in new "feature" form
    telemetry_df = transform_df_to_single_features(telemetry_df)

    # Cuttin to defined length (moving_window_size)
    telemetry_df = pd.concat([old_df, telemetry_df])
    if len(telemetry_df) > moving_window_size:
        telemetry_df = telemetry_df.tail(moving_window_size)

    return telemetry_df
