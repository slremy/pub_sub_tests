#!/usr/bin/env python
'''
 Copyright (c) 2012 Sekou Remy
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
'''
'''
	This node subscribes to a ROS message (/odom) and publishes the data to a mqtt broker

'''

import roslib;
roslib.load_manifest("sensor_msgs")
roslib.load_manifest("message_filters")
import urllib
import urllib2
import rospy
import message_filters
from nav_msgs.msg import Odometry


import paho.mqtt.client as mqtt

olddata="0"
myQosLevel = 1  
state = ""

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed: " + str(mid) + " " + str(granted_qos))

def on_log(mosq, obj, level, string):
    #print(string)
    pass

def on_connect(mosq, obj, rc):
    #print("Subscribing")
    #mosq.subscribe("uisgroup/plant_state", myQosLevel)
    #print("rc: " + str(rc))
    pass

def on_message(mosq, obj, msg):
    global state
    state = msg.payload
    #print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))

def on_publish(mosq, obj, mid):
    #print("mid: " + str(mid))
    pass

def on_disconnect(pahoClient, obj, rc):
    print "Disconnected"

try:
    controllerclient = mqtt.Client()
    # Assign event callbacks
    controllerclient.on_message = on_message
    controllerclient.on_connect = on_connect
    controllerclient.on_log = on_log
    controllerclient.on_publish = on_publish
    controllerclient.on_subscribe = on_subscribe
    controllerclient.on_disconnect = on_disconnect
except Exception, err:
    print("Device connection failed", "Error thrown: %s\n" % err)

def initialize_handshake(HOST, PORT, INTERVAL): 
    #connect to a [public] MQTT broker
    controllerclient.connect(HOST, PORT, INTERVAL);print "connected",HOST, PORT


def process(HOST, PORT, DATA ,data_topic):
    global controllerclient
    try:
        controllerclient.loop();
        controllerclient.publish(data_topic, DATA, myQosLevel);
    except Exception, err:
        print("Error thrown: %s with %s\n" % (err,DATA))
    return


class mqtt_sink_get_node:
	def __init__(self, host, port):
		rospy.init_node('mqtt_sink', anonymous=True);
		self.data_topic = rospy.get_param("~data_topic","/odom");
		self.data_interval = rospy.get_param("~data_interval",30);
                self.host= host;
                self.port= port;
                initialize_handshake(self.host,self.port,self.data_interval)

		rospy.Subscriber(self.data_topic, Odometry, self.callback)
		rospy.spin()
		
	def callback(self,msg):
                global olddata
		# data is presumed appropriately encoded in requestor 
		# e.g. <<<< encoded_data = urllib.urlencode({'chl':data});  >>>>
                data=str(msg.header.stamp)+','+str(msg.pose.pose.position.x)+','+str(msg.pose.pose.position.y)+','+str(msg.pose.pose.position.z)+','+str(msg.pose.pose.orientation.x)+','+str(msg.pose.pose.orientation.y)+','+str(msg.pose.pose.orientation.z)+','+str(msg.pose.pose.orientation.w)

                lenSameValues=len([i for i, j in zip(olddata.split(",")[1:], data.split(",")[1:]) if i == j])

                if lenSameValues != len(data.split(",")[1:]):
                     process(self.host,self.port,data,self.data_topic)
                     olddata=data;

if __name__ == '__main__':
	object = mqtt_sink_get_node("cybertiger.clemson.edu",1883);
