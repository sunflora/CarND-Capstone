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
msgs = {}

dbw_enable = False

@sio.on('connect')
def connect(sid, environ):
    print("connect ", sid)

def send(topic, data):
    s = 1
    msgs[topic] = data
    #rospy.logerr("======  send: topic %s", topic)
    #sio.emit(topic, data=json.dumps(data), skip_sid=True)

bridge.register_server(send)

@sio.on('telemetry')
def telemetry(sid, data):
    global dbw_enable
    if data["dbw_enable"] != dbw_enable:
        dbw_enable = data["dbw_enable"]
        bridge.publish_dbw_status(dbw_enable)
        #rospy.logerr("======  telemetry: dbw_enable %s", dbw_enable)
    bridge.publish_odometry(data)
    for i in range(len(msgs)):
        topic, data = msgs.popitem()
        #rospy.logerr("======  telemetry: %s",topic)
        sio.emit(topic, data=data, skip_sid=True)

@sio.on('control')
def control(sid, data):
    bridge.publish_controls(data)

@sio.on('obstacle')
def obstacle(sid, data):
    bridge.publish_obstacles(data)

@sio.on('lidar')
def obstacle(sid, data):
    bridge.publish_lidar(data)

@sio.on('trafficlights')
def trafficlights(sid, data):
    #bridge.publish_traffic(data)
    pass

@sio.on('image')
def image(sid, data):
    #bridge.publish_camera(data)
    pass

if __name__ == '__main__':

    # wrap Flask application with engineio's middleware
    app = socketio.Middleware(sio, app)

    # deploy as an eventlet WSGI server
    #eventlet.wsgi.server(eventlet.listen(('127.0.0.1', 4567)), app)
    eventlet.wsgi.server(eventlet.listen(('', 4567)), app)
