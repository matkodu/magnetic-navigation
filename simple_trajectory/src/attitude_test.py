#!/usr/bin/env python
# license removed for brevity

import math 
import rospy
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, \
    TwistStamped, Pose, Point, Quaternion, PointStamped

# Global
temp = 0.0

# Instruction
def get_instruction_cb(msg):
    temp = msg

# Attitude
def get_attitude_ref():
    flag = temp

    # Initial value
    attitude_ref = Vector3()

    attitude_ref.x = 0.0
    attitude_ref.y = 0.0
    attitude_ref.z = 0.0

    if flag == 0.0:
        attitude_ref.x = 0.0
    elif flag == 1.0:
        attitude_ref.x = 0.001
    elif flag == -1.0:
        attitude_ref.x = -0.001
    else:
        attitude_ref.x = 0.0

    return(attitude_ref)

#######################################################


def talker():
    rospy.Subscriber('/attitude_instruction', Float64, get_instruction_cb, queue_size=1)
    pub_ref = rospy.Publisher('/uav/euler_ref', Vector3, queue_size=10)

    while not rospy.is_shutdown():
        pub_ref.publish(get_attitude_ref())
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

