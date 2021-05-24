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

    print(type(data))
    print('DATA', data)

    telemetry_list += data
    if len(telemetry_list) > 20:
        telemetry_list.pop(0)
    print('RECEIVED:', telemetry_list)
    print('Received data.')
    return "200"


@app.callback(
    Output('live-graph', 'figure'),
    [Input('graph-update', 'n_intervals')]
)
def update_graph_scatter(n):

    data = plotly.graph_objs.Scatter(
        x=[i['timestampms'] for i in telemetry_list],
        y=[i['comp_angle'] for i in telemetry_list],
        name='Scatter',
        mode='lines+markers'
    )

    return {'data': [data],
            'layout': go.Layout(xaxis=dict(range=[min([i['timestampms'] for i in telemetry_list]), max([i['timestampms'] for i in telemetry_list])]), yaxis=dict(range=[min([i['comp_angle'] for i in telemetry_list]), max([i['comp_angle'] for i in telemetry_list])]),)}


if __name__ == '__main__':
    app.run_server(port=5000, host='0.0.0.0')
