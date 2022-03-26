#!/usr/bin/env python3
import warnings
import numpy as np
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion

RATE = 100
RATE_control = 100


def wrap_angle(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class ControllerNode(object):
    def __init__(self):
        self.robot_pose = np.zeros(4)

        self.pub_stm_command = rospy.Publisher("/secondary_robot/stm/command", String, queue_size=1)
        rospy.Subscriber("/secondary_robot/stm/response", String, self.callback_response, queue_size=1)
        rospy.Subscriber("/secondary_robot/filtered_coords", PoseWithCovarianceStamped, self.callback_localization, queue_size=1)
        self.timer = rospy.Timer(rospy.Duration(1. / RATE), self.tcallback_speed)
        self.timer_control = rospy.Timer(rospy.Duration(1. / RATE_control), self.tcallback_control)

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

        # Current goal
        self.x_goal = 1
        self.y_goal = 1
        self.theta_goal = 1

        # Integrals initialization
        self.x_err_int = 0  # Возможно нужно перевести все в локальную систему координат либо изменить начальные условия на текущую координату
        self.y_err_int = 0
        self.theta_err_int = 0

        # Initialize augmented LQR
        self.K_fsfb = np.genfromtxt('K_fsf.csv', delimiter=',')
        self.K_int = np.genfromtxt('K_int.csv', delimiter=',')

    def tcallback_control(self, event):
        # Defines robot state at each time step
        self.X = np.array([self.x, self.y, self.theta, self.v, self.vn, self.w, self.i_out_0, self.i_out_1, self.i_out_2]).T

        # Calculate control vector
        Control_Int_Part = self.K_int @ self.Int_Err 
        Control_FSFB_Part = self.K_fsfb @ self.X
        Control_Vector = Control_Int_Part - Control_FSFB_Part

        # Publish data to STM
        control_string = "code 1" + "code 1" + "%s" % (Control_Vector.T[0])
        self.pub_stm_command(control_string)

    def callback_response(self, data):
        rospy.logwarn(data.data)
        # Subscribe to STM data
        # Kinematics Data Parse
        #self.v =            # Forward Speed V
        #self.vn =           # Lateral Speed Vn
        #self.w =            # Angular Speed W
        # Motor Currents Data Parse
        #self.i_out_0 =
        #self.i_out_1 =
        #self.i_out_2 =

        # Publish data to STM

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

        # Localization Data Parse
        self.x = self.pose.pose.position.x  # Position X
        self.y = self.pose.pose.position.y  # Position Y
        self.theta = self.angle             # Angle Theta

        self.integral_errors() # call integral_errors after receiving new message 

    def integral_errors(self):
        # Calculate deviation between goal and current measurement
        x_dev_err = self.x_goal - self.x
        y_dev_err = self.y_goal - self.y
        theta_dev_err = self.theta_goal - self.theta
        # Integrals Data
        self.x_err_int += x_dev_err                # Intergal of Position Error X
        self.y_err_int += y_dev_err                # Intergal of Position Error Y
        self.theta_err_int += theta_dev_err        # Intergal of Angle Error Theta
        # Assign column vector of integral errors  
        self.Int_Err = np.array([[self.x_err_int],      
                                 [self.y_err_int],
                                 [self.theta_err_int]])
        
    
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
