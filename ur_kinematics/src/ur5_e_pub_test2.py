#!/usr/bin/env python

###referance : https://github.com/ros-planning/moveit_tutorials/blob/melodic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import math

def all_close(goal, actual, tolerance):
    all_equal = True
    if type(goal) is list:
        for index in for range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)
    
    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)
    
    return True



class ur5e_arm_commander(object):
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ur5e_arm_control_python_commander',anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        groupe_name = "move_groupe" # In move_group.launch, the name seems to be set as "move_group", so here I set it as well
        move_groupe = moveit_commander.MoveGroupCommander(groupe_name)

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size = 20)
        
        planning_frame = move_group.get_planning_frame()
        print "============ Planning frame: %s" % planning_frame
        
        eef_link = move_group.get_end_effector_link()
        print "============ End effector link: %s" % eef_link
        
        group_names = robot.get_group_names()
        print "============ Available Planning Groups:", robot.get_group_names()
        
        print "============ Printing robot state"
        print robot.get_current_state()
        
        print ""
        
        # Misc variables
        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def go_to_joint_state(self):
        move_group = self.move_group

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = pi/6
        joint_goal[1] = -pi/6
        joint_goal[2] = pi/6
        joint_goal[3] = -pi/6
        joint_goal[4] = pi/6
        joint_goal[5] = -pi/6
        joint_goal[6] = pi/6

        move_group.go(joint_goal, wait=True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self):
        move_group = self.move_group

        pose_goal = geometry_msg.msg.Pose()
        pose_goal.orientation.w = pi/2
        pose_goal.position.x = 0.5
        pose_goal.position.y = 0.3
        pose_goal.position.z = 0.7

        move_group.set_pose_target(pose_goal)

        plan = move_group.go(wait=True)
        move_group.stop()
        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pos, 0.01)

    def plan_cartesian_path(self, scale=1):
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1 #First move up(z)
        wpose.position.y += scale * 0.2 # and sideways(y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1 # Second move forward (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1 # Third move sideway (y)
        waypoints.append(copy.deepcopy(wpose))

        (plan,fraction) = move_group.compute_cartesian_path(
            waypoints, #waypoints to follow
            0.01, #eef_step
            0.0 #jump_threshold
        )

        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        
        display_trajectory_publisher.publish(display_trajectory)
    
    def excute_plan(self, plan):
        move_group = self.move_group
        move_group.execute(plan, wait=True)

    def main():
        try:
            print ""
            print "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
            raw_input()
            tutorial = MoveGroupPythonIntefaceTutorial()

            print "============ Press `Enter` to execute a movement using a joint state goal ..."
            raw_input()
            tutorial.go_to_joint_state()

            print "============ Press `Enter` to execute a movement using a pose goal ..."
            raw_input()
            tutorial.go_to_pose_goal()

            print "============ Press `Enter` to plan and display a Cartesian path ..."
            raw_input()
            cartesian_plan, fraction = tutorial.plan_cartesian_path()

            print "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
            raw_input()
            tutorial.display_trajectory(cartesian_plan)

            print "============ Press `Enter` to execute a saved path ..."
            raw_input()
            tutorial.execute_plan(cartesian_plan)

            print "============ Python tutorial demo complete!"
            
        except rospy.ROSInterruptException:
            return
        except KeyboardInterrupt:
            return

if __name__ == '__main__':
  main()
