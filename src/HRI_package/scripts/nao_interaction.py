#!/usr/bin/env python

# ROS Node for enabling interaction, face tracking and goal setting

# [MISSING TODO]:
# Map environment and set according goal points

# Possible improvements TODO:
# Get video frames from "ALVideoDevice" instead of ros topic (less delay)
# Perform Speech Recognition with external tool instead of NAO's default

# NAO interface library
from naoqi import ALProxy
# OpenCV library (for face detection) and ROS interface
import cv2
from cv_bridge import CvBridge, CvBridgeError
# ROS libraries
import rospy
from sensor_msgs.msg import Image, JointState
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty
# Other libraries
import numpy as np
import argparse
import imutils
from imutils.video import VideoStream
import time

# NAO IP
NAO_IP = "192.168.0.100"
Port = 9559

# Establish Speech Recognition and Text-to-Speech Proxies
asr = ALProxy("ALSpeechRecognition", NAO_IP, Port)
tts = ALProxy("ALTextToSpeech", NAO_IP, Port)
mem = ALProxy("ALMemory", NAO_IP, Port)

# Set keywords vocabulary for associating with target places
vocabulary = ["mobile","door","seminar","ladies","gents","underwater","coffee"]
# More elaborate vocabulary that should be spoken by NAO
phrases = ["mobile robotics laboratory","door","seminar room","ladies washroom",
"mens washroom","underwater robotics laboratory","coffee machines"]
# Corresponding locations for Turtlebot navigation (x,y)
locations = [[-1.9,0.0],[-7.3,-3.0],[-9.1,4.2],[-10.1,-9.1],[-5.7,-16.2],[-1.3,-18.8],[-21.6,-20.0]]

# Load vocabulary and enable Speech Recognition
asr.setVocabulary(vocabulary,True)
asr.subscribe("Test_ASR")

# Robot interface class definition
class robot_interface:

	def __init__(self):
		# Parameters for face tracking through cv_bridge
		self.image_pub = rospy.Publisher("/face_tracking/image",Image,queue_size=1)
		self.bridge = CvBridge()
		self.image_sub = rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.trackface_callback)
		# Setting NAO stiffness (but only on the first two joints)
		self.stiff_pub = rospy.Publisher("/joint_stiffness",JointState,queue_size=1)
		print("Loading face detection model...")
		self.net = cv2.dnn.readNetFromCaffe("/home/$User/Nao_Turtle_ws/src/HRI_package/scripts/deploy.prototxt.txt",
		"/home/$User/Nao_Turtle_ws/src/HRI_package/scripts/res10_300x300_ssd_iter_140000.caffemodel")
		# Interaction parameters (robot state and word recognition confidence)
		self.state = 0 
		self.confidence = 0.0
		# Parameters for NAO head control (for tracking human face) 
		self.pubNao = rospy.Publisher("joint_angles", JointAnglesWithSpeed, queue_size=10)
		self.cmdNao = JointAnglesWithSpeed()
		self.cmdNao.joint_names = ["HeadYaw", "HeadPitch"]
		self.cmdNao.speed = 0.05
		self.yaw = 0.0
		self.pitch = 0.0
		self.dx = 0
		self.dy = 0
		self.stiff = False
		# Parameters for controlling Turtlebot
		self.goal = [0,0]
		self.pubTurtle = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
		self.cmdTurtle = PoseStamped()
		self.cmdTurtle.header.frame_id = "map"
		self.cmdTurtle.pose.orientation.w = 1.0

	def trackface_callback(self,data):
		#if(not self.stiff):
		#	StiffMsg = JointState()
		#	StiffMsg.name = ["HeadYaw", "HeadPitch"]
		#	StiffMsg.effort = [1.0, 1.0]
		#	self.stiff_pub.publish(StiffMsg)
		#	self.stiff = True
		# Control variable that returns whether a face was detected
		face_detected = False
		# Face tracking correction (default if no face is seen)
		self.dx = 0
		self.dy = 0
		# Convert frame ROS msg to cv2 type and resize
		try:
			frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)
		frame = imutils.resize(frame, width=400)
		# Grab the frame dimensions and convert it to a blob
		(h, w) = frame.shape[:2]
		blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 1.0,
			(300, 300), (104.0, 177.0, 123.0))
		# Pass the blob through network and obtain detections and predictions
		self.net.setInput(blob)
		detections = self.net.forward()
		# Loop over the detections
		for i in range(0, detections.shape[2]):
			# Extract confidence (i.e., probability) associated with prediction
			confidence = detections[0, 0, i, 2]
			# Filter out weak detections by ensuring a minimum confidence level
			if confidence < 0.5:
				continue
			face_detected = True
			# If any face is detected, proceed to next state (interaction)
			if(self.state==0):
				self.state = 1
			# Compute (x,y)-coordinates of the bounding box for the object
			box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
			(startX, startY, endX, endY) = box.astype("int")
			self.dx = (startX+endX)/2-300
			self.dy = (startY+endY)/2-225
			# Draw bounding box of the face along with associated probability
			text = "{:.2f}%".format(confidence * 100)
			y = startY - 10 if startY - 10 > 10 else startY + 10
			cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)
			cv2.putText(frame, text, (startX, y),
				cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 2)
		# Control head pose according to face (if detected)
		self.move_head()
		# Call interaction routine, which will depend on robot state
		self.run_dialogue()
		# Set target to Turtlebot if it is the case
		if(self.state==3):
			self.cmdTurtle.pose.position.x = self.goal[0]
			self.cmdTurtle.pose.position.y = self.goal[1]
			self.pubTurtle.publish(self.cmdTurtle)
			print("Goal set to (" + str(self.goal[0]) + "," + str(self.goal[1])  + ")")
			# Reset state (TODO actually should go to 4... then when reaches goal go to 0)
			self.state = 0 

		# Display frame with tracking features
		cv2.imshow("Face tracking", frame)
		cv2.waitKey(3)
		# Publish frame in a ROS topic as well
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
		except CvBridgeError as e:
			print(e)
	
	# Nao head movement routine
	def move_head(self):
		# Update robot head position
		if(self.dy>30):
			self.pitch = self.pitch + 0.03
		elif(self.dy<-30):
			self.pitch = self.pitch - 0.03
		if(self.dx>30):
			self.yaw = self.yaw - 0.03
		elif(self.dx<-30):
			self.yaw = self.yaw + 0.03
		# Check if joints will be outside the bounds
		if(self.pitch>0.4):
			self.pitch = 0.4
		elif(self.pitch<-0.6):
			self.pitch = -0.6
		if(self.yaw>1.1):
			self.yaw = 1.1
		elif(self.yaw<-1.1):
			self.yaw = -1.1
		# Publish Nao joint commands
		self.cmdNao.joint_angles = [self.yaw, self.pitch]
		self.pubNao.publish(self.cmdNao)
	
	# Interaction routine
	def run_dialogue(self):
		# If the robot tracks someone for the first time: ask question
		if(self.state==1):
			tts.say("Hello! \\eos=1\\ Do you need to go somewhere?")
			self.state = 2
		# Otherwise: waiting for response
		elif(self.state==2):
			# Get last word spoken by person
			keyword = mem.getData("WordRecognized")
			self.confidence = keyword[1]
			# Check word confidence
			if(self.confidence<0.5):
				tts.say("")
				#tts.say("Sorry. \\eos=1\\ I didn't understand. \\eos=1\\ Could you say again?")
			else:
				words = keyword[0].split()
				word = words[1]
				tag = vocabulary.index(word)
				tts.say("Alright! \\eos=1\\ Let's go to the " + phrases[tag])
				self.state = 3 # New state: goal location set
				self.goal = locations[tag]

# Default shutdown script
def node_shutdown():
	# Close CV window
	cv2.destroyAllWindows()
	# Disable NAO Stiffness
	rospy.wait_for_service('body_stiffness/disable')
	try:
		stiff_disable = rospy.ServiceProxy('body_stiffness/disable', Empty)
		stiff_disable()
		print("NAO stiffness disabled")
	except rospy.ServiceException, e:
		print(e)
	# Unsubscribe to Speech Recognition
	asr.unsubscribe("Test_ASR")
	print("Unsubscribed to NAO Speech Recognition engine")

# ROS main node
def nao_interaction():
	# Initialize image converter object
	O = robot_interface()
	# Initialize node
	print("Initializing node...")
	rospy.init_node('nao_interaction', anonymous=True)
	rospy.on_shutdown(node_shutdown)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down...")

if __name__ == '__main__':
	try:
		nao_interaction()
	except rospy.ROSInterruptException:
		pass
