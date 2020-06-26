#!/usr/bin/env python

# ROS Node to control Nao head with Oculus Rift CV1 blablabla

import rospy

from naoqi_driver.naoqi_node import NaoqiNode

from std_msgs.msg import String
from nao_head_control.msg import HeadControl

class NaoHeadControl(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'nao_head_control')

        self.connectNaoQi()
	#rate = rospy.Rate(1) # 1 Hz

        rospy.Subscriber("nao_head_rtc", HeadControl, self.handleHeadControl, queue_size=1)

        # Create ROS publisher for speech
        self.pub = rospy.Publisher("speech", String, latch = True, queue_size=1)

        self.say("Nao Head Control online")



    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)


    def say(self, text):
        self.pub.publish(text)

    def handleHeadControl(self, data):
        rospy.logdebug("Head Orientation request: HeadYaw: %f HeadPitch: %f Speed: %f\n", data.joint_angles[0], data.joint_angles[1], data.speed)
	self.motionProxy.setStiffnesses("Head", 1.0)
        try:
            if (data.joint_angles[0] <= 2.0857 and data.joint_angles[0] >= -2.0857 and
		data.joint_angles[1] <= 0.5149 and data.joint_angles[1] >= -0.6720):
		self.motionProxy.setAngles(data.joint_names, data.joint_angles, data.speed)
		rospy.loginfo("Doing the thing: HeadYaw: %f HeadPitch: %f Speed: %f\n", data.joint_angles[0], data.joint_angles[1], data.speed)
		#time.sleep(3.0)
		#rate.sleep() # Sleeps for 1/rate sec
		self.motionProxy.setStiffnesses("Head", 0.0)
            else:
                rospy.loginfo("Something went wrong, fuck my life\n")
		rospy.logdebug("---SOMETHING WENT WORNG--- Head Orientation request: HeadYaw: %f HeadPitch: %f Speed: %f ---SOMETHING WENT WORNG---\n", data.joint_angles[0], data.joint_angles[1], data.speed)
        except RuntimeError,e:
            # We have to assume there's no NaoQI running anymore => exit!
            rospy.logerr("Exception caught in handleHeadControl:\n%s", e)
            rospy.signal_shutdown("No NaoQI available anymore")

if __name__ == '__main__':
    naoHeadControl = NaoHeadControl()
    rospy.loginfo("nao_head_control running...")
    rospy.spin()
    rospy.loginfo("nao_head_control stopping...")
    rospy.loginfo("nao_head_control stopped.")
    exit(0)
