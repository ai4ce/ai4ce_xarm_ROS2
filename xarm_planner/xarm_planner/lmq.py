#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from xarm_planner.xarm_planning_client import XArmPlanningClient
from geometry_msgs.msg import PoseArray


def get_obstacles():
    obstacles = [{}]
    obstacles[0]['type'] = 'sphere'
    obstacles[0]['id'] = 'object'
    obstacles[0]['position'] = (0.43887, -0.0834, 0.395)
    obstacles[0]['radius'] = 0.10
    
    obstacles.append({})
    obstacles[1]['type'] = 'box'
    obstacles[1]['id'] = 'table'
    obstacles[1]['position'] = (0.5, 0.5, -0.02)
    obstacles[1]['quat_xyzw'] = (0, 0, 0, 1)
    obstacles[1]['size'] = (1, 1, 0.02)

    obstacles.append({})
    obstacles[2]['type'] = 'box'
    obstacles[2]['id'] = 'left_wall'
    obstacles[2]['position'] = (0, 0.65, 0)
    obstacles[2]['quat_xyzw'] = (0, 0, 0, 1)
    obstacles[2]['size'] = (1, 0.02, 1)
    return obstacles
  

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
    

    obstacles = get_obstacles()
    planning_client.collision_setup(obstacles=obstacles)


    waypoints = planning_client.surround_and_lock(
        center=[0.43887, -0.0834, 0.395],
        num_waypoints=30,)
    
    pose_array = PoseArray()
    pose_array.header.frame_id = "link_base"
    pose_array.poses = waypoints

    visual_publisher.publish(pose_array)


    for waypoint in waypoints:
        traj = planning_client.plan_to_pose(waypoint)
        if traj is not None:
            judge = input()
            if judge == '':
                planning_client.execute_plan(traj)
            else:
                print("Skip this waypoint")
        else:
            print("Failed to plan to waypoint")
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