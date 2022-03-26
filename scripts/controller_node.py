#!/usr/bin/env python3
import warnings
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

RATE = 40

def wrap_angle(angle):

    return (angle + np.pi) % (2 * np.pi) - np.pi

class ControllerNode(object):
    def __init__(self):
        self.robot_pose = np.zeros(4)

        self.pub_stm_command = rospy.Publisher("/secondary_robot/stm/command", String, queue_size=1)
        rospy.Subscriber("/secondary_robot/stm/response", String, self.callback_response, queue_size=1)
        rospy.Subscriber("/secondary_robot/filtered_coords", PoseWithCovarianceStamped, self.callback_localization, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1. / RATE), self.tcallback_speed)

    def callback_response(self, data):
        rospy.logwarn(data.data)

    def tcallback_speed(self, event):
        data = "0 9"
        self.pub_stm_command.publish(data)

    def callback_localization(self, data):
        q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
             data.pose.pose.orientation.w]
        angle = wrap_angle(euler_from_quaternion(q)[2] % (2 * np.pi))
        self.robot_pose = np.array(
            [data.header.stamp.to_sec(), data.pose.pose.position.x, data.pose.pose.position.y, wrap_angle(angle)])
        rospy.logwarn(self.robot_pose)


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
