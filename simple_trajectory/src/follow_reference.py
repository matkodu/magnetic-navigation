#!/usr/bin/env python
# license removed for brevity

import math 
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, \
    TwistStamped, Pose, Point, Quaternion, PointStamped

# Global variables
pose_ref = Pose()
#msg = PoseStamped()
msg = Pose()

msg.position.x = 0.0   
msg.position.y = 0.0
msg.position.z = 0.0

################### Follow reference #####################

#def uav_pose_cb(msg):

#    uav_pose_cb.p = msg.pose.position

def get_pose_ref():

    # delta_t 
    rate10 = 0.01
    
    # v * delta_t
    v = Pose()
    v.position.x = 1 * rate10
    v.position.y = 1 * rate10
    v.position.z = 1 * rate10
    
    #pose_ref.position.x = uav_pose_cb.p.x + v.position.x
    #pose_ref.position.y = uav_pose_cb.p.y + v.position.y
    #pose_ref.position.z = uav_pose_cb.p.z + v.position.z

    pose_ref.position.x = msg.position.x + v.position.x
    pose_ref.position.y = msg.position.y + v.position.y
    pose_ref.position.z = msg.position.z + v.position.z

    if pose_ref.position.z >= 1:
        pose_ref.position.z = 1

    #pose_ref.orientation.x = 0.0
    #pose_ref.orientation.y = 0.0
    #pose_ref.orientation.z = 0.0
    #pose_ref.orientation.w = 1.0

    msg.position = pose_ref.position


    # smjer pravca -> tocka2 - tocka1
    

    return(pose_ref)

#######################################################


def talker():
    #rospy.Subscriber('/uav/pose', PoseStamped, uav_pose_cb, queue_size=10)
    pub_ref = rospy.Publisher('/uav/pose_ref', Pose, queue_size=10)
    rospy.init_node('trajectory_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        #uav_pose_cb(msg)
        pose_ref = get_pose_ref()
        pub_ref.publish(pose_ref)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

