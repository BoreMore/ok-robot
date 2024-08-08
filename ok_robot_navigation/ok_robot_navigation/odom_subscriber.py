import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String
from nav_msgs.msg import Odometry
import numpy as np
import time

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Twist, PoseStamped
import pickle


class MyOdometrySubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')

        # initialize subscriber and publisher
        #self.subscription = self.create_subscription(Odometry, '/odom', self.current_position_callback, 10)
        self.subscription = self.create_subscription(PoseStamped, '/stretch_vicon/hr_base/hr_base', self.current_position_callback, 10)
        self.subscription  # prevent unused variable warning
        self.publisher_move = self.create_publisher(Twist, '/stretch/cmd_vel', 10)
        timer_period_move = 0.1  # seconds
        self.timer = self.create_timer(timer_period_move, self.move_to_target)
        self.state_callback_verbose = False

        # initialize state feedback
        self.set_initial_state = False
        self.initial_state = np.array([0, 0, 0])
        self.use_odom = False
        
        # initialize command
        self.v_cmd = 0.0
        self.omega_cmd = 0.0
        
        # get waypoints
        self.get_waypoints()
        self.current_waypoint_index = 0
        self.navigation_complete = False

    def get_waypoints(self):
        with open('/home/hornylemur/ament_ws/src/ok-robot/ok_robot_navigation/ok_robot_navigation/path_result_transformed_fixedsoccer.pkl', 'rb') as f:  # Python 3: open(..., 'rb')
             self.path, _ = pickle.load(f)
        
        self.waypoint_number = len(self.path)
        print("%d waypoints loaded" % self.waypoint_number)
        for wpt in self.path:
            print(wpt)

    def move_to_target(self):

        if self.set_initial_state and not self.navigation_complete:
            self.target = np.array(self.path[self.current_waypoint_index]).reshape(-1)
            self.position = self.state[:2]
            yaw = self.state[2]

            point_ahead_distance = 0.1
            point_ahead = self.position + point_ahead_distance * np.array([np.cos(yaw), np.sin(yaw)])
            u = 0.5 * (self.target - point_ahead)
            self.v_cmd = np.cos(yaw) * u[0] + np.sin(yaw) * u[1]
            self.omega_cmd = 1 / point_ahead_distance * (-np.sin(yaw) * u[0] + np.cos(yaw) * u[1])
            self.move_around()
            # print("going to waypoint number %d | target (%.4f, %.4f) | actual (%.4f, %.4f)" % (self.current_waypoint_index, self.target[0], self.target[1], point_ahead[0], point_ahead[1]))
            print("going to waypoint number %d | target (%.4f, %.4f) | actual (%.4f, %.4f)" % (self.current_waypoint_index, self.target[0], self.target[1], self.position[0], self.position[1]))
            print("v: %.4f, omega: %.4f" % (self.v_cmd, self.omega_cmd))

            if np.linalg.norm(self.target - self.position) <= 0.15:
                self.current_waypoint_index += 1
                if self.current_waypoint_index == self.waypoint_number:
                    self.navigation_complete = True
                    self.v_cmd = 0.0
                    self.omega_cmd = 0.0
                    print("path completed!")
                    raise SystemExit

    
    def move_around(self):
        command = Twist()
        command.linear.x = self.v_cmd
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = self.omega_cmd
        self.publisher_move.publish(command)

    def current_position_callback(self, msg):
        # forward is +x and left is +y
        # state = [x, y, yaw]^T
        #current_position = np.array([msg.pose.pose.position.y, -msg.pose.pose.position.x])
        current_position = np.array([msg.pose.position.x / 1000.0, msg.pose.position.y / 1000.0])
        #current_q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        current_q = np.array([msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
        current_r = R.from_quat(current_q)
        current_yaw, current_pitch, current_roll = current_r.as_euler("ZYX")
        self.state = np.array([current_position[0], current_position[1], current_yaw]) - self.initial_state

        if not self.set_initial_state:
            if self.use_odom:
                self.initial_state = self.state
            self.set_initial_state = True
        else:
            if self.state_callback_verbose:
                self.get_logger().info("=> BASE x: %.4f, y: %.4f, yaw: %.4f" % (self.state[0], self.state[1], self.state[2]))


def main(args=None):
    rclpy.init(args=args)
    odom_subscriber = MyOdometrySubscriber()
    try:
        rclpy.spin(odom_subscriber)

    except SystemExit:
        rclpy.logging.get_logger('Quit').info('Path completed, exiting script.')
        odom_subscriber.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        rclpy.logging.get_logger('Quit').info('Keyboard interrupt, exiting script.')
        odom_subscriber.destroy_node()
        rclpy.shutdown()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # odom_subscriber.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()