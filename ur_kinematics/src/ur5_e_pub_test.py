#!/usr/bin/env python
# license removed for brevity
import rospy
import math
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def talker():
    rospy.init_node('joint_point_publisher', anonymous = True)
    pub = rospy.Publisher('arm_controller/command', JointTrajectory, queue_size = 10)
    rospy.sleep(1.0)

    msg = JointTrajectory()
    msg.joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    msg.points = [JointTrajectoryPoint() for i in range(7)]
    msg.points[0].positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg.points[0].time_from_start = rospy.Time(1.0)

    msg.points[1].positions = [math.pi/6, 0.0, 0.0, 0.0, 0.0, 0.0]
    msg.points[1].time_from_start = rospy.Time(2.0)

    msg.points[2].positions = [math.pi/6, math.pi/6, 0.0, 0.0, 0.0, 0.0]
    msg.points[2].time_from_start = rospy.Time(3.0)

    msg.points[3].positions = [math.pi/6, math.pi/6, math.pi/6, 0.0, 0.0, 0.0]
    msg.points[3].time_from_start = rospy.Time(4.0)

    msg.points[4].positions = [math.pi/6, math.pi/6, math.pi/6, math.pi/6, 0.0, 0.0]
    msg.points[4].time_from_start = rospy.Time(5.0)

    msg.points[5].positions = [math.pi/6, math.pi/6, math.pi/6, math.pi/6, math.pi/6, 0.0]
    msg.points[5].time_from_start = rospy.Time(6.0)

    msg.points[6].positions = [math.pi/6, math.pi/6, math.pi/6, math.pi/6, math.pi/6, math.pi/6]
    msg.points[6].time_from_start = rospy.Time(7.0)
    
    pub.publish(msg)
    rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass