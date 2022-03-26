#!/usr/bin/env python3
import warnings
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped, PoseArray, Pose, Quaternion, PoseStamped, PoseWithCovarianceStamped, \
    Pose2D

RATE = 40


class ControllerNode(object):
    def __init__(self):
        self.pub_stm_command = rospy.Publisher("stm/command", String, queue_size=1)
        rospy.Subscriber("/secondary_robot/stm/response", String, self.callback_response, queue_size=1)
        rospy.Subscriber("/secondary_robot/filtered_coords", PoseWithCovarianceStamped, self.callback_response, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1. / RATE), self.tcallback_speed)

    def callback_response(self, data):
        rospy.logwarn(data.data)

    def tcallback_speed(self, event):
        data = "0 9"
        self.pub_stm_command.publish(data)

    def LQR(self):
        self.K_fsfb = np.genfromtxt('K_fsf.csv', delimiter=',')
        self.K_int = np.genfromtxt('K_int.csv', delimiter=',')

    def integrals(self):
        msg = 
        self.X_int += msg.pose.pose.position.x
        self.Y_int += msg.pose.pose.position.y

        
def shutdown():
    rospy.logwarn("Controller shutting down")


if __name__ == '__main__':
    try:
        rospy.on_shutdown(shutdown)
        rospy.init_node('controller_node', anonymous=True)
        warnings.filterwarnings("error")
        controller_node = ControllerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
