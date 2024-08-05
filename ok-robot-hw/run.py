import time

import zmq
import cv2
import numpy as np
import PyKDL
from PIL import Image

from robot import HelloRobot
from args import get_args
from camera import RealSenseCamera
from utils.grasper_utils import pickup, move_to_point, capture_and_process_image
# from utils.communication_utils import send_array, recv_array
from global_parameters import *

import hello_helpers.hello_misc as hm

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from rclpy.duration import Duration
import math

X_OFFSET, Y_OFFSET, THETA_OFFSET, r2n_matrix, n2r_matrix = None, None, None, None, None

def load_offset(x1, y1, x2, y2):

    '''
        Take coordinates of two tapes: the robot stands on tape (x1, y1) and look at tape (x2, y2)
        Compute two rotation matrices r2n_matrix and n2r_matrix that can transform coordinates 
        from robot hector slam system coordinate system to record3d coordinate system and 
        from record3d coordinate system to robot hector slam system coordinate system respectively
    '''

    global X_OFFSET, Y_OFFSET, THETA_OFFSET, r2n_matrix, n2r_matrix
    X_OFFSET = x1
    Y_OFFSET = y1
    # x1 = X_OFFSET, x2 = another x
    THETA_OFFSET =  np.arctan2((y2 - y1), (x2 - x1))

    print(f"offsets - {X_OFFSET}, {Y_OFFSET}, {THETA_OFFSET}")
    r2n_matrix = \
        np.array([
            [1, 0, X_OFFSET],
            [0, 1, Y_OFFSET],
            [0, 0, 1]
        ]) @ \
        np.array([
            [np.cos(THETA_OFFSET), -np.sin(THETA_OFFSET), 0],
            [np.sin(THETA_OFFSET), np.cos(THETA_OFFSET), 0],
            [0, 0, 1]
        ])

    n2r_matrix = \
        np.array([
            [np.cos(THETA_OFFSET), np.sin(THETA_OFFSET), 0],
            [-np.sin(THETA_OFFSET), np.cos(THETA_OFFSET), 0],
            [0, 0, 1]
        ]) @ \
        np.array([
            [1, 0, -X_OFFSET],
            [0, 1, -Y_OFFSET],
            [0, 0, 1]
        ])

def navigate(robot, xyt_goal):

    '''
        An closed loop controller to move the robot from current positions to [x, y, theta]
        - robot: StretchClient robot controller
        - xyt_goal: target [x, y, theta] we want the robot to go
    '''

    xyt_goal = np.asarray(xyt_goal)
    xyt_curr = hm.HelloNode.get_robot_floor_pose_xya()[0]

    # Make sure theta is within [-pi, pi]
    while xyt_goal[2] < -np.pi or xyt_goal[2] > np.pi:
        xyt_goal[2] = xyt_goal[2] + 2 * np.pi if xyt_goal[2] < -np.pi else xyt_goal[2] - 2 * np.pi
        
    # Closed loop navigation
    while True:
        theta_inc = xyt_goal[2] - xyt_curr[2]
        # from Orrin's old code: theta = math.atan2(xyt_goal[1] - xyt_curr[1], xyt_goal[0] - xyt_curr[0])
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


        # xyt_curr = robot.nav.get_base_pose()
        xyt_curr = hm.HelloNode.get_robot_floor_pose_xya()[0]

        print("The robot is currently located at " + str(xyt_curr))
        if np.allclose(xyt_curr[:2], xyt_goal[:2], atol=POS_TOL) and \
                (np.allclose(xyt_curr[2], xyt_goal[2], atol=YAW_TOL)\
                 or np.allclose(xyt_curr[2], xyt_goal[2] + np.pi * 2, atol=YAW_TOL)\
                 or np.allclose(xyt_curr[2], xyt_goal[2] - np.pi * 2, atol=YAW_TOL)):
            print("The robot is finally at " + str(xyt_goal))
            break


def run_navigation(robot, socket, A, B):
    '''
        An API for running navigation. By calling this API, human will ask the robot to find objects
        specified by "A (near B)"
        - robot: StretchClient robot controller
        - socket: ZMQ socket, used for asking workstation to compute planned path
        - A: text query specifying target object
        - B: text query specifying an object close to target object that helps localization of A, set to None, if 
                you just want the robot to localize A instead of "A near B"
    '''

    # Compute start_xy of the robot
    start_xy = hm.HelloNode.get_robot_floor_pose_xya()[0][:2] # robot.nav.get_base_pose()
    print(start_xy)
    transformed_start_xy = r2n_matrix @ np.array([start_xy[0], start_xy[1], 1])
    start_xy[0], start_xy[1] = transformed_start_xy[0], transformed_start_xy[1]
    start_xy[2] += THETA_OFFSET
    print(start_xy)

    # Send start_xy, A, B to the workstation and receive planned paths

    z = end_xyz[2]
    end_xyz = (n2r_matrix @ np.array([end_xyz[0], end_xyz[1], 1]))
    end_xyz[2] = z

    if input("Start navigation? Y or N ") == 'N':
        return None
    
    # Let the robot run faster

    #robot.nav.set_velocity(v = 25, w = 20)

    # Transform waypoints into robot hector slam coordinate systems and let robot navigate to those waypoints
    final_paths = []
    for path in paths:
        transformed_path = n2r_matrix @ np.array([path[0], path[1], 1])
        transformed_path[2] = path[2] - THETA_OFFSET
        print(transformed_path)
        final_paths.append(transformed_path)
        navigate(robot, transformed_path)
    #xyt = robot.nav.get_base_pose()
    xyt = hm.HelloNode.get_robot_floor_pose_xya()[0]
    xyt[2] = xyt[2] + np.pi / 2
    navigate(robot, xyt)
    return end_xyz

def run():
    args = get_args()
    load_offset(args.x1, args.y1, args.x2, args.y2)
    
    # base_node = TOP_CAMERA_NODE

    #transform_node = GRIPPER_MID_NODE
    #hello_robot = HelloRobot(end_link = transform_node)

    #context = zmq.Context()
    #nav_socket = context.socket(zmq.REQ)
    #nav_socket.connect("tcp://" + args.ip + ":" + str(args.navigation_port))
    #anygrasp_socket = context.socket(zmq.REQ)
    #anygrasp_socket.connect("tcp://" + args.ip + ":" + str(args.manipulation_port))

    while True:
        A = None
        if input("You want to run navigation? Y or N") != "N":
            A = 'trash bin'
            B = 'floor'

            #hello_robot.robot.switch_to_navigation_mode()
            hm.HelloNode.switch_to_trajectory_mode()
            #hello_robot.robot.move_to_post_nav_posture()
            hm.HelloNode.stow_the_robot()
            #hello_robot.robot.head.look_front()
            end_xyz = run_navigation(hello_robot.robot, nav_socket, A, B)

        print('debug coordinates', hello_robot.robot.nav.get_base_pose())

if __name__ == '__main__':
    run()
