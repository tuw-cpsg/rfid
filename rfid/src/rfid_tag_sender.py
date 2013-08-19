#!/usr/bin/python
# coding=utf-8

############################################################
# author: Juergen Maier
# 25.3.2013
# project: RFID reader implementation for robot
############################################################
# this is the module publishing the received IDs in a topic
# in the ROS
############################################################

import roslib; roslib.load_manifest('rfid')
import rospy
from std_msgs.msg import String
from rfid.msg import *

pub=""

############################################################
# opens publisher in ROS with defined Name and Theme
############################################################

def openPublisher(NameTheme,NameNode) :
    global pub

    pub = rospy.Publisher(NameTheme,rfid_rcv_tag)
    rospy.init_node(NameNode)

############################################################
# publish the IDs in the parameter
# parameter is string, ID and RSSI value alternate separated
# by blanks, an ID and RSSI value are always published
# together
############################################################

def publishMessage(msg):
    global pub

    i=1
    parts=msg.split(' ')

    while i < len(parts) :
        data=rfid_rcv_tag()
        data.tagID=parts[i-1]
        data.rssi=int(parts[i],16)
        pub.publish(data)

        i += 2

############################################################
