from time import time

from pandas.core.dtypes.dtypes import PeriodDtype
import dash
from dash.dependencies import Output, Input
import dash_core_components as dcc
import dash_html_components as html
import plotly.express as px
import plotly.graph_objs as go
import plotly
import json
from flask import request

telemetry_list = []

app = dash.Dash(__name__)

app.layout = html.Div(
    [
        dcc.Graph(id='live-graph', animate=True),
        dcc.Interval(
            id='graph-update',
            interval=1000,
            n_intervals=0
        ),
    ]
)


@app.server.route("/telemetry", methods=["POST"])
def telemetry():
    global telemetry_list
    data = json.loads(request.data)

    telemetry_list += data
    if len(telemetry_list) > 3000:
        telemetry_list = telemetry_list[-3000:]
    print('Received data.')
    return "200"


@app.callback(
    Output('live-graph', 'figure'),
    [Input('graph-update', 'n_intervals')]
)
def update_graph_scatter(n):

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
                   title='FALL-E Telementry Data', range_x=[min_x, max_x], range_y=[min_y, max_y])

    return data


if __name__ == '__main__':
    app.run_server(port=5000, host='0.0.0.0')
