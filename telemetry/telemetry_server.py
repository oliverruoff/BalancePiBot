from time import time

from pandas.core.dtypes.dtypes import PeriodDtype
import dash
from dash.dependencies import Output, Input, State
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import plotly.graph_objs as go
import json
from flask import request

# number of data rows shown
MOVING_WINDOW_SIZE = 3000

# PID values
Kp = 50
Ki = 0
Kd = 0.1

telemetry_list = []

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
def number_render(n_clicks, P, I, D):
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
    global telemetry_list
    data = json.loads(request.data)

    telemetry_list += data
    if len(telemetry_list) > MOVING_WINDOW_SIZE:
        telemetry_list = telemetry_list[-MOVING_WINDOW_SIZE:]
    print('Received data.')
    return "200"


@app.callback(
    Output('frequency-graph', 'figure'),
    [Input('graph-update', 'n_intervals')]
)
def update_graph_telemetry_frequency(n):
    if len(telemetry_list) == 0:
        return "Data Missing"
    prepared_data = []
    for line in telemetry_list:
        if line['frequency'] > 1000:
            continue
        prepared_data.append({
            'time': line['time'],
            'value': line['frequency'],
            'feature': 'frequency'
        })

    time_col = [i['time'] for i in prepared_data]
    val_col = [i['value'] for i in prepared_data]

    min_x = min(time_col) if len(time_col) > 0 else 0
    max_x = max(time_col) if len(time_col) > 0 else 10
    min_y = min(val_col) if len(val_col) > 0 else -50
    max_y = max(val_col) if len(val_col) > 0 else 50

    data = px.line(prepared_data, x="time", y="value", color="feature",
                   title='FALL-E Telementry Data - Frequency', range_x=[min_x, max_x], range_y=[min_y, max_y])

    return data


@app.callback(
    Output('live-graph', 'figure'),
    [Input('graph-update', 'n_intervals')]
)
def update_graph_telemetry_stabilisation(n):
    if len(telemetry_list) == 0:
        return "Data Missing"
    prepared_data = []
    for line in telemetry_list:
        prepared_data.append({
            'time': line['time'],
            'value': line['comp_angle'],
            'feature': 'comp_angle'
        })
        prepared_data.append({
            'time': line['time'],
            'value': line['control'],
            'feature': 'control'
        })
        prepared_data.append({
            'time': line['time'],
            'value': line['gyro_angle'],
            'feature': 'gyro'
        })
        prepared_data.append({
            'time': line['time'],
            'value': line['accel_angle'],
            'feature': 'accel_angle'
        })

    time_col = [i['time'] for i in prepared_data]
    val_col = [i['value'] for i in prepared_data]

    min_x = min(time_col) if len(time_col) > 0 else 0
    max_x = max(time_col) if len(time_col) > 0 else 10
    min_y = min(val_col) if len(val_col) > 0 else -50
    max_y = max(val_col) if len(val_col) > 0 else 50

    data = px.line(prepared_data, x="time", y="value", color="feature",
                   title='FALL-E Telementry Data - Stabilisation', range_x=[min_x, max_x], range_y=[min_y, max_y])

    return data


if __name__ == '__main__':
    app.run_server(port=5000, host='0.0.0.0')
