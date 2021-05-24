import dash
from dash.dependencies import Output, Input
import dash_core_components as dcc
import dash_html_components as html
import plotly
import random
import plotly.graph_objs as go
from collections import deque
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

    time_col = [i['time'] for i in telemetry_list]
    comp_angle_col = [i['comp_angle'] for i in telemetry_list]

    data = plotly.graph_objs.Line(
        x=time_col,
        y=comp_angle_col,
        name='Scatter',
        mode='lines'
    )

    min_x = min(time_col) if len(time_col) > 0 else 0
    max_x = max(time_col) if len(time_col) > 0 else 10
    min_y = min(comp_angle_col) if len(comp_angle_col) > 0 else -50
    max_y = max(comp_angle_col) if len(comp_angle_col) > 0 else 50

    return {'data': [data],
            'layout': go.Layout(xaxis=dict(range=[min_x, max_x]), yaxis=dict(range=[min_y, max_y]),)}


if __name__ == '__main__':
    app.run_server(port=5000, host='0.0.0.0')
