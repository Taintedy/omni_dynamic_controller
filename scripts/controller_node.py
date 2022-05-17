#!/usr/bin/env python3
import warnings
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

# Define control rate
RATE_get_local_robot_speed = 50
RATE_publish_control = 50


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class ControllerNode(object):
    """
    Controller operates in the local coordinate system assigned with the robot.
    Localization coordinates should be transformed into local robot coordinates.
    Target point should be transformed into local coordinate system.
    """

    def __init__(self):
        self.robot_pose = None
        self.Int_Err = None

        # Creating trajectory
        # keypoints for the trajectory
        #poses = np.array([[1, 0, 0],
        #                  [1, 0.5, np.pi / 2],
        #                  [0, 0, np.pi]])
        # creating trajectory [[x], [y], [theta]]
        #x = np.array([0])
        #y = np.array([0])
        #theta = np.array([0])
        #num_of_points = 100
        # for pose in poses:
        #     x_lin = np.linspace(x[-1], pose[0], num_of_points)
        #     y_lin = np.linspace(y[-1], pose[1], num_of_points)
        #     theta_lin = np.linspace(theta[-1], pose[2], num_of_points)
        #
        #     x = np.concatenate([x, x_lin])
        #     y = np.concatenate([y, y_lin])
        #     theta = np.concatenate([theta, theta_lin])
        #
        # self.trajectory = np.vstack([x, y, theta])

        # creating delta array [[dx, dy, dtheta]]
        # delta = []
        # for i in range(self.trajectory.shape[1] - 1):
        #     delta.append(np.array(
        #         [self.trajectory[0, i + 1] - self.trajectory[0, i], self.trajectory[1, i + 1] - self.trajectory[1, i],
        #          self.trajectory[2, i + 1] - self.trajectory[2, i]]))
        # self.deltas = np.array(delta)

        # Current goal

        # Global system
        self.x_target_global = 1
        self.y_target_global = 0
        self.theta_target_global = 0
        self.x_goal_local = 0
        self.y_goal_local = 0
        self.theta_goal_local = 0

        # Integrals initialization
        self.x_err_int = 0  # Возможно нужно перевести все в локальную систему координат либо изменить начальные условия на текущую координату
        self.y_err_int = 0
        self.theta_err_int = 0

        # Setting initial speed outputs
        self.vx_local = 0
        self.vy_local = 0
        self.w_local = 0

        # Initial poses
        self.x_local = 0  # Position X in the local coordinate system
        self.y_local = 0  # Position Y in the local coordinate system
        self.theta_local = 0  # Position Theta in the local coordinate system

        self.x_global_init = None
        self.y_global_init = None
        self.w_global_init = None

        # Initialize augmented LQR
        self.K_fsfb = np.genfromtxt('/home/rpi2/catkin_ws/src/reset_22/omni_dynamic_controller/scripts/K_fsfb.csv',
                                    delimiter=',')
        self.K_int = np.genfromtxt('/home/rpi2/catkin_ws/src/reset_22/omni_dynamic_controller/scripts/K_int.csv',
                                   delimiter=',')

        # Creating ROS Publishers
        self.pub_stm_command = rospy.Publisher("/secondary_robot/stm/command", String, queue_size=1)
        self.pub_robot_command = rospy.Publisher("/secondary_robot/robot_command", String, queue_size=1)
        self.state_pub = rospy.Publisher("/secondary_robot/robot_state", String, queue_size=1)

        # Creating ROS Subscribers
        rospy.Subscriber("/secondary_robot/stm/response", String, self.callback_response, queue_size=1)
        rospy.Subscriber("/secondary_robot/filtered_coords", PoseWithCovarianceStamped, self.callback_localization, queue_size=1)

        # Initialize poll timers
        self.timer = rospy.Timer(rospy.Duration(1. / RATE_get_local_robot_speed), self.tcallback_speed)
        self.timer_control = rospy.Timer(rospy.Duration(1. / RATE_publish_control), self.tcallback_control, reset=True)

    def tcallback_control(self, event):
        """

        :param event:
        :return:
        """
        # Defines robot state at each time step
        if self.Int_Err is not None:
            self.X = np.array([self.x_local, self.y_local, self.theta_local, self.vx_local, self.vy_local, self.w_local])[np.newaxis].T
            dist = np.sqrt((self.x_local - self.x_goal_local) ** 2 + (self.y_local - self.y_goal_local) ** 2)
            if dist <= 0.1:
                control_string = "0" + " 0x50 " + "0 0 0"
                #rospy.logwarn("control cmd: %s" % control_string)
                self.pub_stm_command.publish(control_string)
                self.Int_Err = np.array([[0],
                                         [0],
                                         [0]])
            else:
                self.state_pub.publish("%s" % (self.X.T[0]))
                # Calculate control vector
                #rospy.logwarn('K_int %s' % self.K_int)
                Control_Int_Part = self.K_int @ self.Int_Err
                Control_FSFB_Part = self.K_fsfb @ self.X
                Control_Vector = Control_Int_Part - Control_FSFB_Part

                rospy.logwarn("control_string: %s" % Control_Vector)
                rospy.logwarn("Control_Int_Part: %s" % Control_Int_Part)
                rospy.logwarn("Control_FSFB_Part: %s" % Control_FSFB_Part)

                for i in range(3):
                    if abs(Control_Vector[i]) > 0.5:
                        Control_Vector[i] = np.sign(Control_Vector[i]) * 0.5

                # Publish data to STM
                control_string = "%f %f %f" % (
                Control_Vector.T[0][0], Control_Vector.T[0][1], Control_Vector.T[0][2])
                rospy.logwarn("control cmd: %s" % control_string)

                self.pub_robot_command.publish(control_string)

    def callback_response(self, data):
        """Function parses data from STM into robot local frame speed data.

        Each time when data is published in topic "/secondary_robot/stm/response",
        this function is called.

        :param data: Data from the STM topic "/secondary_robot/stm/response"
        :return: self.vx_local, self.vy_local, self.w_local
        """
        #rospy.logwarn("speed data: %s" % data.data)
        splitted_data = data.data.split()
        # Subscribe to STM data
        # Kinematics Data Parse

        if len(splitted_data) == 4:
            speed_vals = [float(token) for token in splitted_data]
            self.vx_local = speed_vals[1]  # Forward Speed vx_local
            self.vy_local = speed_vals[2]  # Lateral Speed vy_local
            self.w_local = speed_vals[3]   # Angular Speed w_local
        else:
            pass
            #rospy.logwarn("not speed value")

    def tcallback_speed(self, event):
        """Function requests information about speed from the STM

        Each time the timer ticks with requested rate,
        this function publishes information with the request of the speed from the STM.

        :return: Publish command into the topic "/secondary_robot/stm/command"
        """
        data = "0 9"  # Name of the command to request speed
        self.pub_stm_command.publish(data)  # Publish required command to the topic "/secondary_robot/stm/command"

    def callback_localization(self, data):
        """Function return global localization coordinates of the robot

        Each time information is published in topic "/secondary_robot/filtered_coords",
        this function is called.

        :param data:
        :return:
        """
        # Get global quaternion
        q = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
             data.pose.pose.orientation.w]

        # Get global angle
        angle = wrap_angle(euler_from_quaternion(q)[2] % (2 * np.pi))

        self.robot_pose = np.array(
            [data.header.stamp.to_sec(), data.pose.pose.position.x, data.pose.pose.position.y, wrap_angle(angle)])

        if (self.x_global_init is None):
            self.x_global_init = self.robot_pose[1]
            self.y_global_init = self.robot_pose[2]
            self.w_global_init = self.robot_pose[3]


        # rospy.logwarn("robot pose: %s" % self.robot_pose)

        # Localization Data Parse
        self.x_local = self.robot_pose[1] - self.x_global_init # Position X
        self.y_local = self.robot_pose[2] - self.y_global_init  # Position Y
        self.theta_local = wrap_angle(self.robot_pose[3] - self.w_global_init)  # Angle Theta

        self.integral_errors()  # call integral_errors after receiving new message

    def integral_errors(self):
        # Calculate deviation between goal and current measurement
        x_dev_err = self.x_goal_local - self.x_local
        y_dev_err = self.y_goal_local - self.y_local
        theta_dev_err = self.theta_goal_local - self.theta_local
        # Integrals Data
        self.x_err_int += x_dev_err  # Intergal of Position Error X
        self.y_err_int += y_dev_err  # Intergal of Position Error Y
        self.theta_err_int += theta_dev_err  # Intergal of Angle Error Theta
        # Assign column vector of integral errors  
        self.Int_Err = np.array([[self.x_err_int],
                                 [self.y_err_int],
                                 [self.theta_err_int]])


def shutdown():
    control_string = "0" + " 0x50 " + "0 0 0"
    controller_node.pub_stm_command.publish(control_string)
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
