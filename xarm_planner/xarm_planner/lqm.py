#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from xarm_planner.xarm_planning_client import XArmPlanningClient
from geometry_msgs.msg import Pose, PoseArray
from geometry_msgs.msg import Transform
import cv2


import time

def main():

    rclpy.init()
    visualization_node = Node("visualization_node")

    visual_publisher = visualization_node.create_publisher(
        msg_type=PoseArray,
        topic="/xarm_planner/surround_waypoints",
        qos_profile=1,
    )
        
    # gripper_client = URRobotiqClient()
    planning_client = XArmPlanningClient(cartesian_planning=False,
                                         pipeline_id="pilz_industrial_motion_planner",
                                         planner_id="PTP")
    
    obstacles = [{}]
    obstacles[0]['type'] = 'sphere'
    obstacles[0]['id'] = 'object'
    obstacles[0]['position'] = (0.33, 0, 0.3)
    obstacles[0]['radius'] = 0.05

    planning_client.collision_setup(obstacles=obstacles)


    waypoints = planning_client.surround_and_lock(
        center=[0.33, 0, 0.3],
        num_waypoints=30,)
    
    pose_array = PoseArray()
    pose_array.header.frame_id = "link_base"
    pose_array.poses = waypoints

    visual_publisher.publish(pose_array)

    # if I hit esc, stop the program

    # print(waypoints)

    for waypoint in waypoints:
        planning_client.move_to_pose(waypoint)
        # only proceed the for loop if I press enter. Use input() to wait for user input
        a = input()

        

        
    # print(current_pose)
    # target_pose = Pose()

    # target_pose = current_pose
    # target_pose.position.z += 0.1

    # print(target_pose)


    # planning_client.move_to_pose(target_pose)
    
    # time.sleep(2)
    # target_pose.position.z += 0.1
    # planning_client.move_to_pose(target_pose)


    rclpy.shutdown()
    


if __name__ == '__main__':
    main()