#!/usr/bin/env python

import rospy
import socketio
import eventlet
import eventlet.wsgi
import time
from flask import Flask, render_template

from bridge import Bridge
from conf import conf

sio = socketio.Server()
app = Flask(__name__)
bridge = Bridge(conf)
msgs = []

dbw_enable = False

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)

def send(topic, data):
    msgs.append((topic, data))

bridge.register_server(send)

@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable
    global count
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        bridge.publish_dbw_status(dbw_enable)
        #rospy.logerr("======  telemetry: dbw_enable %s", dbw_enable)
    bridge.publish_odometry(data)

    for i in range(len(msgs)):
        topic, data = msgs.pop(0)
        #rospy.logerr("======  telemetry: %s %s", topic, data)
        sio.emit(topic, data=data, ignore_queue=True)

@sio.on('control')
def control(sid, data):
    #bridge.publish_controls(data)
    pass

@sio.on('obstacle')
def obstacle(sid, data):
    #bridge.publish_obstacles(data)
    pass

@sio.on('lidar')
def lidar(sid, data):
    #bridge.publish_lidar(data)
    pass

@sio.on('trafficlights')
def trafficlights(sid, data):
    #rospy.logerr("server.py.trafficlights data: %s", data)
    bridge.publish_traffic(data)
    pass

@sio.on('image')
def image(sid, data):
    #bridge.publish_camera_signal(dbw_enable)
    # TODO: add back the actual camera image publisher
    #bridge.publish_camera(data)
    pass


def send_control(steering_angle, throttle, brake):
    sio.emit('steer', data={'steering_angle': str(steering_angle)}, skip_sid=True, ignore_queue=True)
    sio.emit('throttle', data={'throttle': str(throttle)}, skip_sid=True, ignore_queue=True)
    sio.emit('brake', data={'brake': str(brake)}, skip_sid=True, ignore_queue=True)


if __name__ == '__main__':

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    # increase backlog value due to socket performance issues
    eventlet.wsgi.server(eventlet.listen(('', 4567), backlog=1000), app)
