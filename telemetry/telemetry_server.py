#!flask/bin/python
from flask import Flask, request

app = Flask(__name__)


@app.route('/telemetry')
def index():
    data = request.data
    print(data)
    return "Hello, World!"


if __name__ == '__main__':
    app.run(debug=True)
