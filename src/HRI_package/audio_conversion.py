#!/usr/bin/env python

import rospy
import numpy as np
import std_msgs.msg
from naoqi_bridge_msgs.msg import AudioBuffer
from audio_common_msgs.msg import AudioData

def getAudio(msg):
	x = msg.data
	y = AudioData()
	y.data = [int((float(i)/32768+1)/2*255) for i in x]
	print("NAO")
	print(msg.header.stamp)
	print("ROS")
	print(rospy.Time.now())
	pub.publish(y)

pub = rospy.Publisher('/microphone',AudioData,queue_size=10)

print ("Initializing conversion...")
rospy.init_node('audio_conversion')

sub = rospy.Subscriber('/nao_robot/microphone/naoqi_microphone/audio_raw',AudioBuffer,getAudio)


while not rospy.is_shutdown():
	pass
