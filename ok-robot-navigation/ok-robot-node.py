import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import math
import hello_helpers.hello_misc as hm
from scipy.spatial.transform import Rotation as R


from std_msgs.msg import Int32, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

#import path_planning

import time

import numpy as np
from PIL import Image

from args import get_args
from global_parameters import *
import os

import pickle

class OKRobotNode(hm.HelloNode):
    def __init__(self):
        hm.HelloNode.__init__(self)
        hm.HelloNode.main(self, 'ok_robot_node', 'ok_robot_node', wait_for_first_pointcloud=False)
        self.current_position = None

        #self.joint_state = None 
        #self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)
        print("INITIALIZE SUBSCRIBER")
        self.create_subscription(Odometry, '/odom', self.current_position_callback, 1)
        time.sleep(0.1)


    # def joint_states_callback(self, msg):
    #     self.joint_state = msg

    def current_position_callback(self, msg):
        self.current_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        self.current_q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.current_r = R.from_quat(self.current_q)
        self.current_yaw, self.current_pitch, self.current_roll = self.current_r.as_euler("ZYX")
        print("===> BASE x: %.4f, y: %.4f, yaw: %.4f" % (msg.pose.pose.position.x, msg.pose.pose.position.y, self.current_yaw))
        # self.current_yaw = np.array([msg.pose.pose.orientation.z])


    def send_waypoints(self):
        path, start_xyz, end_xyz = None, None, None
        #if self.joint_state is not None:
            #path, end_xyz = path_planning.main(['path_planning.py', 'debug=False', 'dataset_path=r3d/2024-08-05--ExperimentRoom_Take2.r3d', 'cache_path=experimentroom.pt', 'pointcloud_path=experimentroom.ply', 'pointcloud_visualization=True', 'min_height=-1.3'])

        with open('path_result_transformed.pkl', 'rb') as f:  # Python 3: open(..., 'rb')
             path, start_xyz, end_xyz = pickle.load(f)

        print('from pkl: path = ', path)
        print('from pkl: start_xyz = ', start_xyz)
        print('from pkl: end_xyz = ', end_xyz)
        #time.sleep(20)
        # start_xyz = np.array(start_xyz) = [0, 0, 0]
        initial_position = self.current_position
        prev_waypoint = initial_position
        self.theta_inc = 0.0
        for waypoint in path:
            waypoint[0] = -1.0 * waypoint[0]
            waypoint = np.array(waypoint) + initial_position
            print('waypoint', waypoint)
            self.navigate(prev_waypoint, waypoint)
            prev_waypoint = waypoint
        print('Done sending all waypoints. Navigation is complete.')
        return path, end_xyz
    
        print("The robot is currently located at " + str(xyt_curr))
#             if np.allclose(xyt_curr[:2], xyt_goal[:2], atol=POS_TOL) and \
#                     (np.allclose(xyt_curr[2], xyt_goal[2], atol=YAW_TOL)\
#                     or np.allclose(xyt_curr[2], xyt_goal[2] + np.pi * 2, atol=YAW_TOL)\
#                     or np.allclose(xyt_curr[2], xyt_goal[2] - np.pi * 2, atol=YAW_TOL)):
#                 print("The robot is finally at " + str(xyt_goal))
#                 break
        

    def navigate(self, xyt_curr, xyt_goal):
        '''
            An closed loop controller to move the robot from current positions to [x, y, theta]
            - xyt_goal: target [x, y, theta] we want the robot to go
        '''
        
        # Closed loop navigation
#         while True:

        # xyt_goal = np.asarray(xyt_goal)
            
        # theta_inc = xyt_goal[2] - xyt_curr[2]
        # from Orrin's old code: theta = math.atan2(xyt_goal[1] - xyt_curr[1], xyt_goal[0] - xyt_curr[0])
        yaw_target = np.arctan((xyt_goal[0] - xyt_curr[0])/(xyt_goal[1] - xyt_curr[1]))
        theta_inc = yaw_target - self.current_yaw
        linear_inc = math.sqrt((xyt_goal[1] - xyt_curr[1])**2 + (xyt_goal[0] - xyt_curr[0])**2)
    
        # robot.nav.navigate_to(xyt_goal, blocking = False)
        point = JointTrajectoryPoint()
        point.time_from_start = Duration(seconds=0).to_msg()

        # Assign trajectory_goal as a FollowJointTrajectoryGoal message prompt_type
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.goal_time_tolerance = Duration(seconds=0).to_msg()

        trajectory_goal.trajectory.joint_names = ['rotate_mobile_base']
        point.positions = [theta_inc]
        trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        #self.get_logger().info('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
        # Make the action call and send goal of the new joint position
        self.trajectory_client.send_goal_async(trajectory_goal)
        time.sleep(5)
        self.get_logger().info('Done sending rotation command.')

        trajectory_goal.trajectory.joint_names = ['translate_mobile_base']
        point.positions = [linear_inc]
        trajectory_goal.trajectory.points = [point]
        trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
        #self.get_logger().info('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
        # Make the action call and send goal of the new joint position
        self.trajectory_client.send_goal_async(trajectory_goal)
        time.sleep(5)
        self.get_logger().info('Done sending linear translation command.')

        print("x: %.4f, y: %.4f, yaw: %.4f" % (self.current_position[0], self.current_position[0], self.current_yaw))

#			 # xyt_curr = robot.nav.get_base_pose()
#             xyt_curr = hm.HelloNode.get_robot_floor_pose_xya()[0]

#             print("The robot is currently located at " + str(xyt_curr))
#             if np.allclose(xyt_curr[:2], xyt_goal[:2], atol=POS_TOL) and \
#              if np.allclose(xyt_curr[:2], xyt_goal[:2], atol=POS_TOL) and \
#                     (np.allclose(xyt_curr[2], xyt_goal[2], atol=YAW_TOL)\
#                     or np.allclose(xyt_curr[2], xyt_goal[2] + np.pi * 2, atol=YAW_TOL)\
#                     or np.allclose(xyt_curr[2], xyt_goal[2] - np.pi * 2, atol=YAW_TOL)):
#                 print("The robot is finally at " + str(xyt_goal))
#                 break
        # self.current_position = xyt_goal
        # self.current_yaw = yaw_target

    
    # Node main
    def main(self, start_xyz):
        while rclpy.ok():
            rclpy.spin_once(self)
            path, end_xyz = self.send_waypoints()
            print(end_xyz)
            print(path)
        

def main():
    try:
        # Initialize and spin ROS2 node
        #rclpy.init(args=args)
        start_xyz = [0.066222, 0.450469, -1.183902] # same as p1 in path planning
        ok_robot_node = OKRobotNode()
        ok_robot_node.main(start_xyz)

    except KeyboardInterrupt:
        ok_robot_node.get_logger().info('Keyboard Interrupt. Shutting Down OKRobotNode...')
        ok_robot_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




# import rclpy
# from rclpy.node import Node
# from rclpy.duration import Duration

# import math
# import hello_helpers.hello_misc as hm


# from std_msgs.msg import Int32, String
# from geometry_msgs.msg import Twist
# from sensor_msgs.msg import JointState
# from trajectory_msgs.msg import JointTrajectoryPoint
# from control_msgs.action import FollowJointTrajectory

# import path_planning

# import time

# import zmq
# import cv2
# import numpy as np
# import PyKDL
# from PIL import Image

# from args import get_args
# from global_parameters import *




# X_OFFSET, Y_OFFSET, THETA_OFFSET, r2n_matrix, n2r_matrix = None, None, None, None, None



# def load_offset(self, x1, y1, x2, y2):

#     '''
#         Take coordinates of two tapes: the robot stands on tape (x1, y1) and look at tape (x2, y2)
#         Compute two rotation matrices r2n_matrix and n2r_matrix that can transform coordinates 
#         from robot hector slam system coordinate system to record3d coordinate system and 
#         from record3d coordinate system to robot hector slam system coordinate system respectively
#     '''

#     global X_OFFSET, Y_OFFSET, THETA_OFFSET, r2n_matrix, n2r_matrix
#     X_OFFSET = x1
#     Y_OFFSET = y1
#     # x1 = X_OFFSET, x2 = another x
#     THETA_OFFSET =  np.arctan2((y2 - y1), (x2 - x1))

#     print(f"offsets - {X_OFFSET}, {Y_OFFSET}, {THETA_OFFSET}")
#     r2n_matrix = \
#         np.array([
#             [1, 0, X_OFFSET],
#             [0, 1, Y_OFFSET],
#             [0, 0, 1]
#         ]) @ \
#         np.array([
#             [np.cos(THETA_OFFSET), -np.sin(THETA_OFFSET), 0],
#             [np.sin(THETA_OFFSET), np.cos(THETA_OFFSET), 0],
#             [0, 0, 1]
#         ])

#     n2r_matrix = \
#         np.array([
#             [np.cos(THETA_OFFSET), np.sin(THETA_OFFSET), 0],
#             [-np.sin(THETA_OFFSET), np.cos(THETA_OFFSET), 0],
#             [0, 0, 1]
#         ]) @ \
#         np.array([
#             [1, 0, -X_OFFSET],
#             [0, 1, -Y_OFFSET],
#             [0, 0, 1]
#         ])


# class OKRobotNode(hm.HelloNode):
#     def __init__(self):
#         hm.HelloNode.__init__(self)
#         hm.HelloNode.main(self, 'ok_robot_node', 'ok_robot_node', wait_for_first_pointcloud=False)
#         self.joint_state = None 
#         self.create_subscription(JointState, '/stretch/joint_states', self.joint_states_callback, 1)


#     def joint_states_callback(self, msg):
#         self.joint_state = msg


#     def send_waypoints(self):
#         if path_planning is not None and self.joint_state is not None:
#             path, end_xyz = path_planning.main(['path_planning.py', 'debug=False', 'dataset_path=r3d/2024-08-05--ExperimentRoom_Take1.r3d', 'cache_path=experimentroom.pt', 'pointcloud_path=experimentroom.ply', 'pointcloud_visualization=True', 'min_height=-1.3'])

#             # Compute start_xy of the robot
#             start_xy = hm.HelloNode.get_robot_floor_pose_xya()[0][:2] # robot.nav.get_base_pose()
#             print(start_xy)
#             transformed_start_xy = r2n_matrix @ np.array([start_xy[0], start_xy[1], 1])
#             start_xy[0], start_xy[1] = transformed_start_xy[0], transformed_start_xy[1]
#             start_xy[2] += THETA_OFFSET
#             print(start_xy)

#             z = end_xyz[2]
#             end_xyz = (n2r_matrix @ np.array([end_xyz[0], end_xyz[1], 1]))
#             end_xyz[2] = z

#             final_path = []
#             for waypoint in path:
#                 transformed_waypoint = n2r_matrix @ np.array([waypoint[0], waypoint[1], 1])
#                 transformed_waypoint[2] = waypoint[2] - THETA_OFFSET
#                 print(transformed_waypoint)
#                 final_path.append(transformed_waypoint)
#                 self.navigate(None, transformed_waypoint)
#             #xyt = robot.nav.get_base_pose()
#             xyt = hm.HelloNode.get_robot_floor_pose_xya()[0]
#             xyt[2] = xyt[2] + np.pi / 2
#             self.navigate(None, xyt)
#         return end_xyz, final_path
        

#     def navigate(self, xyt_goal):

#         '''
#             An closed loop controller to move the robot from current positions to [x, y, theta]
#             - xyt_goal: target [x, y, theta] we want the robot to go
#         '''

#         xyt_goal = np.asarray(xyt_goal)
#         xyt_curr = hm.HelloNode.get_robot_floor_pose_xya()[0]

#         # Make sure theta is within [-pi, pi]
#         while xyt_goal[2] < -np.pi or xyt_goal[2] > np.pi:
#             xyt_goal[2] = xyt_goal[2] + 2 * np.pi if xyt_goal[2] < -np.pi else xyt_goal[2] - 2 * np.pi
            
#         # Closed loop navigation
#         while True:
#             theta_inc = xyt_goal[2] - xyt_curr[2]
#             # from Orrin's old code: theta = math.atan2(xyt_goal[1] - xyt_curr[1], xyt_goal[0] - xyt_curr[0])
#             linear_inc = math.sqrt((xyt_goal[1] - xyt_curr[1])**2 + (xyt_goal[0] - xyt_curr[0])**2)
        
#             # robot.nav.navigate_to(xyt_goal, blocking = False)
#             point = JointTrajectoryPoint()
#             point.time_from_start = Duration(seconds=0).to_msg()

#             # Assign trajectory_goal as a FollowJointTrajectoryGoal message prompt_type
#             trajectory_goal = FollowJointTrajectory.Goal()
#             trajectory_goal.goal_time_tolerance = Duration(seconds=0).to_msg()

#             trajectory_goal.trajectory.joint_names = ['rotate_mobile_base']
#             point.positions = [theta_inc]
#             trajectory_goal.trajectory.points = [point]
#             trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
#             #self.get_logger().info('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
#             # Make the action call and send goal of the new joint position
#             self.trajectory_client.send_goal_async(trajectory_goal)
#             #time.sleep(5)
#             self.get_logger().info('Done sending rotation command.')

#             trajectory_goal.trajectory.joint_names = ['translate_mobile_base']
#             point.positions = [linear_inc]
#             trajectory_goal.trajectory.points = [point]
#             trajectory_goal.trajectory.header.stamp = self.get_clock().now().to_msg()
#             #self.get_logger().info('joint_name = {0}, trajectory_goal = {1}'.format(joint_name, trajectory_goal))
#             # Make the action call and send goal of the new joint position
#             self.trajectory_client.send_goal_async(trajectory_goal)
#             #time.sleep(5)
#             self.get_logger().info('Done sending linear translation command.')


#             # xyt_curr = robot.nav.get_base_pose()
#             xyt_curr = hm.HelloNode.get_robot_floor_pose_xya()[0]

#             print("The robot is currently located at " + str(xyt_curr))
#             if np.allclose(xyt_curr[:2], xyt_goal[:2], atol=POS_TOL) and \
#                     (np.allclose(xyt_curr[2], xyt_goal[2], atol=YAW_TOL)\
#                     or np.allclose(xyt_curr[2], xyt_goal[2] + np.pi * 2, atol=YAW_TOL)\
#                     or np.allclose(xyt_curr[2], xyt_goal[2] - np.pi * 2, atol=YAW_TOL)):
#                 print("The robot is finally at " + str(xyt_goal))
#                 break
    
#     # Node main
#     def main(self):
#         while rclpy.ok():
#             rclpy.spin_once(self)
#             end_xyz, final_path = self.send_waypoints()
#             print(end_xyz)
#             print(final_path)
#             ### insert code for moving
        

# def main():
#     args = get_args()
#     load_offset(args.x1, args.y1, args.x2, args.y2)
#     try:
#         # Initialize and spin ROS2 node
#         #rclpy.init(args=args)
#         ok_robot_node = OKRobotNode()
#         ok_robot_node.main()

#     except KeyboardInterrupt:
#         ok_robot_node.get_logger().info('Keyboard Interrupt. Shutting Down OKRobotNode...')
#         ok_robot_node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()