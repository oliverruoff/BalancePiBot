#!flask/bin/python
from flask import Flask, request, Response, render_template
import json

telemetry_list = []

app = Flask(__name__)


@app.route("/telemetry", methods=["POST"])
def telemetry():
    global telemetry_list
    data = json.loads(request.data)
    telemetry_list += data
    if len(telemetry_list) > 20:
        telemetry_list.pop(0)
    print('Received data.')
    return "200"


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
