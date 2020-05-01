#!/usr/bin/env python

# ROS Node to control Nao hands with Oculus Rift CV1 blablabla

import rospy

from naoqi_driver.naoqi_node import NaoqiNode

from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyResponse
from nao_hands_control.srv import HandsControl

class NaoHandsControl(NaoqiNode):
    def __init__(self):
        NaoqiNode.__init__(self, 'nao_hands_control')

        self.connectNaoQi()
	#rate = rospy.Rate(1) # 1 Hz

        # start services / actions:
        self.openHandsControlSrv = rospy.Service("nao_hands_control/open_hand", HandsControl, self.handleOpenHandSrv)
        self.closeHandsControlSrv = rospy.Service("nao_hands_control/close_hand", HandsControl, self.handleCloseHandSrv)
        # Create ROS publisher for speech
        self.pub = rospy.Publisher("speech", String, latch = True, queue_size=1)

        self.say("Nao Hands Control online")



    def connectNaoQi(self):
        '''(re-) connect to NaoQI'''
        rospy.loginfo("Connecting to NaoQi at %s:%d", self.pip, self.pport)

        self.motionProxy = self.get_proxy("ALMotion")
        if self.motionProxy is None:
            exit(1)


    def say(self, text):
        self.pub.publish(text)


    def handleOpenHandSrv(self, req):
        try:
            if (req.hand is False): handName = 'RHand'
	    else: handName = 'LHand'
	    self.motionProxy.openHand(handName)
            rospy.loginfo("Service request recived: Open Hand %s \n", handName)
            return ()
        except RuntimeError, e:
            rospy.logerr("Exception while handling hand open request (hehe get it?):\n%s", e)
            return None


    def handleCloseHandSrv(self, req):
        try:
            if (req.hand is False): handName = 'RHand'
	    else: handName = 'LHand'
	    self.motionProxy.closeHand(handName)
            rospy.loginfo("Service request recived: Close Hand %s \n", handName)
            return ()
        except RuntimeError, e:
            rospy.logerr("Exception while handling hand close request (hehe get it?:)\n%s", e)
            return None

if __name__ == '__main__':
    naoHandsControl = NaoHandsControl()
    rospy.loginfo("nao_hands_control running...")
    rospy.spin()
    rospy.loginfo("nao_hands_control stopping...")
    rospy.loginfo("nao_hands_control stopped.")
    exit(0)
