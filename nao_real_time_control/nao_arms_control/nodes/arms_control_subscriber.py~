#!/usr/bin/env python

# ROS Node to control Nao arms with Oculus Rift CV1 blablabla

import rospy

from naoqi_driver.naoqi_node import NaoqiNode

from std_msgs.msg import String
from nao_arms_control.msg import ArmsControl

class NaoArmsControl(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'nao_arms_control')

        self.connectNaoQi()
	#rate = rospy.Rate(1) # 1 Hz
	firstCall = True

        rospy.Subscriber("nao_arms_rtc", ArmsControl, self.handleArmsControl, queue_size=1)

        # Create ROS publisher for speech
        self.pub = rospy.Publisher("speech", String, latch = True, queue_size=1)

        self.say("Nao Arms Control online")



    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)
	self.postureProxy = self.get_proxy("ALRobotPosture")
        if self.postureProxy is None:
            exit(1)



    def say(self, text):
        self.pub.publish(text)

    def handleArmsControl(self, data):
        rospy.logdebug("Right Arm target position request: position: %f %f %f orientation: %f %f %f \n", data.position_right_arm[0], data.position_right_arm[1], data.position_right_arm[2], data.position_right_arm[3], data.position_right_arm[4], data.position_right_arm[5])
        rospy.logdebug("Left Arm target position request: position: %f %f %f orientation: %f %f %f \n", data.position_left_arm[0], data.position_left_arm[1], data.position_left_arm[2], data.position_left_arm[3], data.position_left_arm[4], data.position_left_arm[5])
	#if (firstCall): firstCall = False 
	#rospy.loginfo("firstcall: %b", firstCall)
	#self.postureProxy.goToPosture("StandInit", 0.5)

        try:
		#Right arm
    		chainName = "RArm"
    		frame     = 2 #FRAME_TORSO {FRAME_TORSO = 0, FRAME_WORLD = 1, FRAME_ROBOT = 2}
    		useSensor = False
			# AXIS_MASK_X 1
			# AXIS_MASK_Y 2
			# AXIS_MASK_Z 4
			# AXIS_MASK_WX 8
			# AXIS_MASK_WY 16
			# AXIS_MASK_WZ 32
    		axisMask         = 7+8 # (7 for position only, 56 for rotation only and 63 for both)
    		fractionMaxSpeed = 0.5
	
		Rtarget = [data.position_right_arm[0], data.position_right_arm[1], data.position_right_arm[2], data.position_right_arm[3], data.position_right_arm[4], data.position_right_arm[5]] # Absolute Position


		rospy.loginfo("Doing the right arm thingy! position: %f %f %f orientation: %f %f %f \n", data.position_right_arm[0], data.position_right_arm[1], data.position_right_arm[2], data.position_right_arm[3], data.position_right_arm[4], data.position_right_arm[5])
		self.motionProxy.setPositions(chainName, frame, Rtarget, fractionMaxSpeed, axisMask)
		#Left arm
    		chainName = "LArm"
		
		Ltarget = [data.position_left_arm[0], data.position_left_arm[1], data.position_left_arm[2], data.position_left_arm[3], data.position_left_arm[4], data.position_left_arm[5]] # Absolute Position

		rospy.loginfo("Doing the left arm thingy! position: %f %f %f orientation: %f %f %f \n", data.position_left_arm[0], data.position_left_arm[1], data.position_left_arm[2], data.position_left_arm[3], data.position_left_arm[4], data.position_left_arm[5])
		self.motionProxy.setPositions(chainName, frame, Ltarget, fractionMaxSpeed, axisMask)
        except RuntimeError,e:
            # We have to assume there's no NaoQI running anymore => exit!
            rospy.logerr("Exception caught in handleArmsControl:\n%s", e)
            rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    naoArmsControl = NaoArmsControl()
    rospy.loginfo("nao_arms_control running...")
    rospy.spin()
    rospy.loginfo("nao_arms_control stopping...")
    rospy.loginfo("nao_arms_control stopped.")
    exit(0)
