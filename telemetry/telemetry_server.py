from time import time

import pandas as pd
from pandas.core.dtypes.dtypes import PeriodDtype
import dash
from dash.dependencies import Output, Input, State
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import json
from flask import request

import utils

# number of data rows shown
NUMBER_OF_LAST_N_SECONDS_TO_BE_DISPLAYED = 20
FEATURES_TRANSMITTED = 5
AVERAGE_SENSOR_FREQUENCY_HERTZ = 170
MOVING_WINDOW_SIZE = NUMBER_OF_LAST_N_SECONDS_TO_BE_DISPLAYED * \
    FEATURES_TRANSMITTED * AVERAGE_SENSOR_FREQUENCY_HERTZ

# PID values
Kp = 50
Ki = 0
Kd = 0.1

telemetry_df = utils.generate_empty_feature_df_with_size(
    NUMBER_OF_LAST_N_SECONDS_TO_BE_DISPLAYED)

app = dash.Dash(__name__)

app.layout = html.Div(
    [dcc.Input(id="Kp", type="number", value=50),
        dcc.Input(id="Ki", type="number", value=0),
        dcc.Input(id="Kd", type="number", value=0.1),
        html.Button('Submit', id='submit-val', n_clicks=0),
        dcc.Graph(id='frequency-graph', animate=True),
        dcc.Graph(id='live-graph', animate=True),
        dcc.Interval(
            id='graph-update',
            interval=1000,
            n_intervals=0
    ),
        html.Div(id='container-button-basic',
                 children='Enter a value and press submit')
    ]
)


@app.callback(
    Output('container-button-basic', 'children'),
    [Input('submit-val', 'n_clicks')],
    [State("Kp", "value"),
     State("Ki", "value"),
     State("Kd", "value")]
)
def update_pid(n_clicks, P, I, D):
    print('Setting new PID values')
    global Kp
    global Ki
    global Kd
    Kp = P
    Ki = I
    Kd = D


@app.server.route("/sync", methods=["GET"])
def get_pid_values():
    pid_dict = {'Kp': Kp, 'Ki': Ki, 'Kd': Kd}
    return json.dumps(pid_dict)


@app.server.route("/telemetry", methods=["POST"])
def telemetry():
    global telemetry_df
    data = json.loads(request.data)
    telemetry_df = utils.prepare_input_data(
        telemetry_df, data, NUMBER_OF_LAST_N_SECONDS_TO_BE_DISPLAYED,
        MOVING_WINDOW_SIZE)
    return "200"


@app.callback(
    Output('frequency-graph', 'figure'),
    [Input('graph-update', 'n_intervals')]
)
def update_graph_telemetry_frequency(n):
    frequency_df = telemetry_df.loc[telemetry_df['feature'] == 'frequency']

    if len(frequency_df) == 0:
        return "Data Missing"

    frequency_df = frequency_df.tail(
        NUMBER_OF_LAST_N_SECONDS_TO_BE_DISPLAYED*(AVERAGE_SENSOR_FREQUENCY_HERTZ-20))

    # Getting boundings of graph
    min_x = min(frequency_df.time) if len(frequency_df.time) > 0 else 0
    max_x = max(frequency_df.time) if len(frequency_df.time) > 0 else 10
    min_y = min(frequency_df.value) if len(frequency_df.value) > 0 else -50
    max_y = max(frequency_df.value) if len(frequency_df.value) > 0 else 50

    data = px.line(frequency_df, x="time", y="value", color="feature",
                   title='FALL-E Telementry Data - Frequency', range_x=[min_x, max_x], range_y=[min_y, max_y])
    return data


@app.callback(
    Output('live-graph', 'figure'),
    [Input('graph-update', 'n_intervals')]
)
def update_graph_telemetry_stabilisation(n):
    if len(telemetry_df) == 0:
        return "Data Missing"

    feature_df = telemetry_df.loc[telemetry_df['feature'] != 'frequency']

    # Getting boundings of graph
    min_x = min(feature_df.time) if len(feature_df.time) > 0 else 0
    max_x = max(feature_df.time) if len(feature_df.time) > 0 else 10
    min_y = min(feature_df.value) if len(feature_df.value) > 0 else -50
    max_y = max(feature_df.value) if len(feature_df.value) > 0 else 50

    data = px.line(feature_df, x="time", y="value", color="feature",
                   title='FALL-E Telementry Data - Stabilisation', range_x=[min_x, max_x], range_y=[min_y, max_y])

    return data


if __name__ == '__main__':
    app.run_server(port=5000, host='0.0.0.0')
