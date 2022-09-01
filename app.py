"""
GNU General Public License v3.0 (GPL-3.0)

This file is part of the Hexabot Project (https://github.com/Mystery2913/Hexabot).
Copyright (C) 2022 Dylan Sharm (Mystery2913).

Copyright (C) 2007 Free Software Foundation, Inc. https://fsf.org/
Everyone is permitted to copy and distribute verbatim copies of this license document, but changing it is not allowed.

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation, version 3 of the License. This program is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details. You should have received a copy of the GNU General Public License along
with this program. If not, see <https://www.gnu.org/licenses/>.
"""

from flask import Flask, render_template, request
import firmware
import multiprocessing as mp
import ctypes

# Initialise the walking and speed shared memory variables and the stackExecute class.
walking = mp.Value(ctypes.c_bool, False)
speed = mp.Value(ctypes.c_float, 0)
queue = firmware.stackExecute()

# Initialise the process with the queue method and pass it `walking` and `speed` arguments, then start the process.
process = mp.Process(target=queue.queue, args=(walking,speed))
process.start()

robot = firmware.Hexabot()

app = Flask(__name__)

@app.route('/', methods=["GET", "POST"])
def index():
    return render_template("index.html")

# Post request for passing data to the backend.
@app.route('/data', methods=["GET", "POST"])
def data():
    if request.method == "POST":
        # Unpack data
        jsonData = request.get_json()
        print(jsonData)
        # If the data type is walk, then set the walking value to True. This will cause the queue process on call Hexabot.walkForward().
        if jsonData["type"] == "walk":
            y = int(jsonData['y'])
            if y > 0:
                # Scale the y value of the joystick to the speed passed to stackExecute.queue() and then to Hexabot.walkForward().
                speed.value = y / 100
                walking.value = True
            return {
                'response' : 'I am the response'
            }
        # If the data type is stand, run the moveLeg method to make Hexabot stand.
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