#!/usr/bin/env python

# ROS Node for enabling interaction, face detection and Turtlebot goal setting

# NAO interface library
from naoqi import ALProxy
# ROS libraries
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import Empty
# Other libraries
import numpy as np
import time

# NAO IP (set) and Port (default)
NAO_IP = "192.168.1.102"
Port = 9559

# Establish Speech Recognition and Text-to-Speech Proxies
asr = ALProxy("ALSpeechRecognition", NAO_IP, Port)
tts = ALProxy("ALTextToSpeech", NAO_IP, Port)
faceProxy = ALProxy("ALFaceDetection", NAO_IP, Port)
mem = ALProxy("ALMemory", NAO_IP, Port)
# Disable autonomous movements
move = ALProxy("ALAutonomousMoves", NAO_IP, Port)
move.setExpressiveListeningEnabled(False)
# Enable stiffness
motionProxy = ALProxy("ALMotion", NAO_IP, Port)
motionProxy.stiffnessInterpolation("Body", 1.0, 1.0)

# Set keywords vocabulary for associating with target places
vocabulary = ["mobile","door","seminar","ladies","gents","underwater","coffee"]
# More elaborate vocabulary that should be spoken by robot
phrases = ["mobile robotics laboratory","door","seminar room","ladies washroom",
"mens washroom","underwater robotics laboratory","coffee machine"]
# Corresponding locations for Turtlebot navigation (x,y) in the map
locations = [[-1.9,0.0],[-7.3,-3.0],[-9.1,4.2],[-10.1,-9.1],[-5.7,-16.2],[-1.3,-18.8],[-21.6,-20.0]]

# Load vocabulary and enable Speech Recognition
asr.setVocabulary(vocabulary,True)
asr.subscribe("Test_ASR")

# Face recognition 
period = 500 # ms
faceProxy.subscribe("Test_Face", period, 0.0)

# Robot interface class definition
class robot_interface:

	def __init__(self):
		# Interaction parameters (robot state and word recognition confidence)
		self.state = 0 
		self.confidence = 0.0
		# Parameters for controlling Turtlebot
		self.goal = [0,0]
		self.pubTurtle = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
		self.subStatus = rospy.Subscriber("/move_base/status", GoalStatusArray, self.checkGoal)
		self.cmdTurtle = PoseStamped()
		self.cmdTurtle.header.frame_id = "map"
		self.cmdTurtle.pose.orientation.w = 1.0
		# Initialize node
		print("Initializing node...")
		rospy.init_node('nao_interaction', anonymous=True)
		rospy.on_shutdown(node_shutdown)
		# Call interaction routine
		self.run_interaction()
		
	def checkGoal(self,msg):
		if(self.state==4 and len(msg.status_list)==1):
			if(msg.status_list[0].status==3):
				print("Goal reached")
				tts.say("Here we are!")
				self.state = 0
				asr.subscribe("Test_ASR")
				faceProxy.subscribe("Test_Face")
			elif(msg.status_list[0].status==4):
				print("Plan aborted")
				tts.say("Sorry, I could not find the place for you.")
				self.state = 0
				asr.subscribe("Test_ASR")
				faceProxy.subscribe("Test_Face")
		
	def run_interaction(self):
		print("Running interaction")
		r = rospy.Rate(10) # 10hz
		while(not rospy.is_shutdown()):
			# Starting, check if face was detected
			if(self.state==0):
				face_detected = mem.getData("FaceDetected")
				if(face_detected): # Update state if case
					print("Face Detected")
					face_detected = False
					self.state = 1
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
					pass#tts.say("")
				else:
					words = keyword[0].split()
					word = words[1]
					tag = vocabulary.index(word)
					tts.say("Alright! \\eos=1\\ Let's go to the " + phrases[tag])
					self.state = 3 # New state: goal location set
					self.goal = locations[tag]
			# Set target to Turtlebot if it is the case
			elif(self.state==3):
				faceProxy.unsubscribe("Test_Face")
				asr.unsubscribe("Test_ASR")
				print("Publishing to Turtlebot...")
				self.cmdTurtle.pose.position.x = self.goal[0]
				self.cmdTurtle.pose.position.y = self.goal[1]
				self.pubTurtle.publish(self.cmdTurtle)
				print("Goal set to (" + str(self.goal[0]) + "," + str(self.goal[1])  + ")")
				# Set state waiting for goal
				self.state = 4 
			r.sleep()

# Default shutdown script
def node_shutdown():
	print("Shutting down...")
	asr.unsubscribe("Test_ASR")
	faceProxy.unsubscribe("Test_Face")
	print("Unsubscribed to Face Detection and Speech Recognition engines.")
	motionProxy.stiffnessInterpolation("Body", 0.0, 1.0)
	print("Body stiffness disabled")

# ROS main node
def nao_interaction():
	# Initialize interface object
	O = robot_interface()


if __name__ == '__main__':
	try:
		nao_interaction()
	except rospy.ROSInterruptException:
		pass
