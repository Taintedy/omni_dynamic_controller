#!/usr/bin/env python3
import warnings
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

RATE = 100


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class ControllerNode(object):
    def __init__(self):
        self.robot_pose = np.zeros(4)

        self.pub_stm_command = rospy.Publisher("/secondary_robot/stm/command", String, queue_size=1)
        rospy.Subscriber("/secondary_robot/stm/response", String, self.callback_response, queue_size=1)
        rospy.Subscriber("/secondary_robot/filtered_coords", PoseWithCovarianceStamped, self.callback_localization,
                         queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1. / RATE), self.tcallback_speed)

        # Creating trajectory
        # keypoints for the trajectory
        poses = np.array([[1, 0, 0],
                          [1, 0.5, np.pi / 2],
                          [0, 0, np.pi]])
        # creating trajectory [[x], [y], [theta]]
        x = np.array([0])
        y = np.array([0])
        theta = np.array([0])
        num_of_points = 100
        for pose in poses:
            x_lin = np.linspace(x[-1], pose[0], num_of_points)
            y_lin = np.linspace(y[-1], pose[1], num_of_points)
            theta_lin = np.linspace(theta[-1], pose[2], num_of_points)

            x = np.concatenate([x, x_lin])
            y = np.concatenate([y, y_lin])
            theta = np.concatenate([theta, theta_lin])

        self.trajectory = np.vstack([x, y, theta])

        # creating delta array [[dx, dy, dtheta]]
        delta = []
        for i in range(self.trajectory.shape[1] - 1):
            delta.append(np.array(
                [self.trajectory[0, i + 1] - self.trajectory[0, i], self.trajectory[1, i + 1] - self.trajectory[1, i],
                 self.trajectory[2, i + 1] - self.trajectory[2, i]]))
        self.deltas = np.array(delta)

        # Integrals initialization
        self.x_int = 0  # Возможно нужно перевести все в локальную систему координат либо изменить начальные условия на текущую координату
        self.y_int = 0
        self.theta_int = 0

    def callback_response(self, data):
        # rospy.logwarn(data.data)
        pass
        rospy.logwarn(data.data)
        # Subscribe to STM data
        # Kinematics Data Parse
        # self.v =            # Forward Speed V
        # self.vn =           # Lateral Speed Vn
        # self.w =            # Angular Speed W
        # Motor Currents Data Parse
        # self.i_out_0 =
        # self.i_out_1 =
        # self.i_out_2 =

        # Publish data to STM
        # self.i_0 = # Control current for Motor_0
        # self.i_1 = # Control current for Motor_1
        # self.i_2 = # Control current for Motor_2

    def tcallback_speed(self, event):
        data = "0 9"
        self.pub_stm_command.publish(data)

    def LQR(self):
        self.K_fsfb = np.genfromtxt('K_fsf.csv', delimiter=',')
        self.K_int = np.genfromtxt('K_int.csv', delimiter=',')

    def callback_localization(self, data):
        q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
             data.pose.pose.orientation.w]
        self.angle = wrap_angle(euler_from_quaternion(q)[2] % (2 * np.pi))
        self.robot_pose = np.array(
            [data.header.stamp.to_sec(), data.pose.pose.position.x, data.pose.pose.position.y, wrap_angle(angle)])
        # rospy.logwarn(self.robot_pose)

        # Localization Data Parse
        self.x = self.pose.pose.position.x  # Position X
        self.y = self.pose.pose.position.y  # Position Y
        self.theta = self.angle  # Angle Theta

    def integrals(self):
        # Integrals Data
        self.x_int += self.x  # Intergal of Position X
        self.y_int += self.y  # Intergal of Position Y
        self.theta_int += self.theta  # Intergal of Angle Theta

    def robot_state(self):
        self.X = np.array(
            [self.x, self.y, self.theta, self.v, self.vn, self.w, self.i_out_0, self.i_out_1, self.i_out_2, self.x_int,
             self.y_int, self.theta_int])


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
