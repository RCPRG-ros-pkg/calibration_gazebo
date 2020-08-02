#!/usr/bin/env python

import functools
import rospy

from sensor_msgs.msg import Image,CameraInfo
from dynamic_reconfigure.server import Server
from calibration_gazebo.cfg import CameraDelayConfig

latency = 0.1

def callback(config, level):
    global latency
    rospy.loginfo("Reconfigure Request: {latency}".format(**config))
    latency = config.latency
    return config


rospy.init_node("camera_delay")

srv = Server(CameraDelayConfig, callback)

pub_im = rospy.Publisher("/camera_delay/image", Image, queue_size=4)
pub_ci = rospy.Publisher("/camera_delay/camera_info", CameraInfo, queue_size=4)

def delayed_callback_im(msg, event=None):
    msg.header.stamp = rospy.Time.now()
    pub_im.publish(msg)

def delayed_callback_ci(msg, event=None):
    msg.header.stamp = rospy.Time.now()
    pub_ci.publish(msg)

def callback_im(msg):
    if latency > 0.001:
        _ = rospy.Timer(rospy.Duration(latency),
                        functools.partial(delayed_callback_im, msg),
                        oneshot=True)
    else:
        delayed_callback_im(msg)

def callback_ci(msg):
    if latency > 0.001:
        _ = rospy.Timer(rospy.Duration(latency),
                        functools.partial(delayed_callback_ci, msg),
                        oneshot=True)
    else:
        delayed_callback_ci(msg)

sub_im = rospy.Subscriber("/camera/image", Image, callback_im, queue_size=4)
sub_ci = rospy.Subscriber("/camera/camera_info", CameraInfo, callback_ci, queue_size=4)

rospy.spin()