#!/usr/bin/env python
import rospy
from software_driving_interface.msg import HDI_state

# Print output
def callback(msg):
   rospy.loginfo(rospy.get_name() + " received the following: wa: %f wf: %f v: %f" % (msg.wheel_angle, msg.wheel_force, msg.vibration))


################################################################
# Test Listener for HDI to verify communication:
#   prints values received from SDI using HDI_feedback messages
################################################################
def listener():
   rospy.init_node('HDI_feedback_listener')
   rospy.Subscriber("HDI_feedback", HDI_state, callback)
   rospy.spin()


# Main Method
if __name__ == '__main__':
   listener()
