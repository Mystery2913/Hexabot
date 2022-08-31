import ctypes
from flask import Flask, render_template, request
import firmware
import multiprocessing as mp

walking = mp.Value(ctypes.c_bool, False)
speed = mp.Value(ctypes.c_float, 0)
queue = firmware.stackExecute()

process = mp.Process(target=queue.queue, args=(walking,speed))
process.start()

robot = firmware.Hexabot()

app = Flask(__name__)

@app.route('/', methods=["GET", "POST"])
def index():
    return render_template("index.html")
    
@app.route('/data', methods=["GET", "POST"])
def data():
    if request.method == "POST":
        jsonData = request.get_json()
        print(jsonData)
        if jsonData["type"] == "walk":
            y = int(jsonData['y'])
            if y > 0:
                speed.value = y / 100
                walking.value = True
            return {
                'response' : 'I am the response'
            }
        elif jsonData["type"] == "stand":
            for leg in robot.legs:
                leg.moveLeg(-80,-120, 0, 0.5)
            return {
                'response' : 'I am the response'
            }
    return render_template('dataButton.html')

@app.route('/joy')
def joy():
    return render_template("joy.html")