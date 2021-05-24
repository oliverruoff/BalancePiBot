#!flask/bin/python
from flask import Flask, request
import json

app = Flask(__name__)


@app.route("/telemetry", methods=["POST"])
def index():
    data = json.loads(request.data)
    print('Received:', data)
    return "Hello, World!"


if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0')
