#!/usr/bin/env python
# license removed for brevity

import math 
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, PoseStamped, \
    TwistStamped, Pose, Point, Quaternion

pose_ref = Pose()
msg = PoseStamped()
msg1 = Vector3()
msg2 = Vector3()
def line_point_1_cb(msg1):
    line_point_1_cb.x = msg1.x
    line_point_1_cb.y = msg1.y
    line_point_1_cb.z = msg1.z

def line_point_2_cb(msg2):
    
    line_point_2_cb.x = msg2.x
    line_point_2_cb.y = msg2.y
    line_point_2_cb.z = msg2.z

def get_trajectory():
    trajectory = JointTrajectory()
    trajectory.header.stamp = rospy.get_rostime();
    trajectory.header.frame_id = "base_link";
    trajectory.joint_names.append("x");
    trajectory.joint_names.append("y");
    trajectory.joint_names.append("z");

    trajectory.joint_names.append("dummy");
    trajectory.joint_names.append("dummy1");
    trajectory.joint_names.append("yaw");

    ############### line parameters ###################

    x_line_1 = line_point_1_cb.x
    y_line_1 = line_point_1_cb.y
    z_line_1 = line_point_1_cb.z

    x_line_2 = line_point_2_cb.x
    y_line_2 = line_point_2_cb.y
    z_line_2 = line_point_2_cb.z

    # point 1
    #x_line_1 = 0
    #y_line_1 = 4
    #z_line_1 = 1.5

    # point 2
    #x_line_2 = 3
    #y_line_2 = 2
    #z_line_2 = 1.5

    # angle
    if y_line_1 - y_line_2 == 0:
        alpha = math.pi/2
    elif x_line_1 - x_line_2 == 0:
        alpha = 0
    else:
        alpha = math.atan((y_line_1 - y_line_2)/(x_line_1 - x_line_2))

    # distance from line
    r = 1

    # x and y segments of distance from line 
    S = r*math.sin(alpha) # y segment
    C = r*math.cos(alpha) # x segment

    ###################################################

    ###### calculate the closest point on the line ######

    # x point
    if x_line_1 > x_line_2:
        start = x_line_2
        end = x_line_1
    else:
        start = x_line_1
        end = x_line_2

    i = start
    smallest_number = math.sqrt(start*start - 0) # distance
    while i >= start and i <= end:
        d = math.sqrt(i*i - 0) # difference between the line point and initial position
        if d <= smallest_number:
            smallest_number = d
            x_ref = i # reference
        else:
            x_ref = smallest_number
        i = i + 0.001

    # y point
    if y_line_1 > y_line_2:
        start = y_line_2
        end = y_line_1
    else:
        start = y_line_1
        end = y_line_2

    i = start
    smallest_number = math.sqrt(start*start - 0) # distance
    while i >= start and i <= start:
        d = math.sqrt(i*i - 0) # difference between the line point and initial position
        if d <= smallest_number:
            smallest_number = d
            y_ref = i # reference
        else:
            y_ref = smallest_number
        i = i + 0.001

    # z point
    if z_line_1 > z_line_2:
        start = z_line_2
        end = z_line_1
    else:
        start = z_line_1
        end = z_line_2

    i = start
    smallest_number = math.sqrt(start*start - 0) # distance
    while i >= start and i <= end:
        d = math.sqrt(i*i - 0) # difference between the line point and initial position
        if d <= smallest_number:
            smallest_number = d
            z_ref = i # reference
        else:
            z_ref = smallest_number
        i = i + 0.001

    #######################################################

    #################### trajectory #######################

    # range in for loop (number of iterations)
    if x_ref >= y_ref:
        ref_points = int(x_ref/0.001)
    else:
        ref_points = int(y_ref/0.001)

    # move UAV in z axis
    i = 1
    z_traj = 1
    while z_traj < z_ref:
        p=JointTrajectoryPoint();
        p.positions.append(0)#0
        p.positions.append(0)#1
        p.positions.append(z_traj)#2
        p.positions.append(0)#3
        p.positions.append(0)#4
        p.positions.append(0)#5
        p.positions.append(0)#6
        p.positions.append(0)#7	
        p.positions.append(0)#8		
        p.positions.append(0)#9
        p.positions.append(0)#10

        p.velocities.append(0)#0

        p.velocities.append(0)#1
        p.velocities.append(0)#2
        p.velocities.append(0)#3
        p.velocities.append(0)#4
        p.velocities.append(0)#5
        p.velocities.append(0)#6
        p.velocities.append(0)#7
        p.velocities.append(0)#8
        p.velocities.append(0)#9
        p.velocities.append(0)#10

        p.accelerations.append(0)#0
        p.accelerations.append(0)#1
        p.accelerations.append(0)#2
        p.accelerations.append(0)#3
        p.accelerations.append(0)#4
        p.accelerations.append(0)#5
        p.accelerations.append(0)#6
        p.accelerations.append(0)#7
        p.accelerations.append(0)#8
        p.accelerations.append(0)#9
        p.accelerations.append(0)#10
        
        trajectory.points.append(p);
        
        z_traj = i*0.001
        i = i + 1

    # move UAV in x axis
    i = 0
    x_traj = 0
    while x_traj < (x_ref - C):
        p=JointTrajectoryPoint();
        p.positions.append(x_traj)#0
        p.positions.append(0)#1
        p.positions.append(z_traj)#2
        p.positions.append(0)#3
        p.positions.append(0)#4
        p.positions.append(0)#5
        p.positions.append(0)#6
        p.positions.append(0)#7	
        p.positions.append(0)#8		
        p.positions.append(0)#9
        p.positions.append(0)#10

        p.velocities.append(0)#0

        p.velocities.append(0)#1
        p.velocities.append(0)#2
        p.velocities.append(0)#3
        p.velocities.append(0)#4
        p.velocities.append(0)#5
        p.velocities.append(0)#6
        p.velocities.append(0)#7
        p.velocities.append(0)#8
        p.velocities.append(0)#9
        p.velocities.append(0)#10

        p.accelerations.append(0)#0
        p.accelerations.append(0)#1
        p.accelerations.append(0)#2
        p.accelerations.append(0)#3
        p.accelerations.append(0)#4
        p.accelerations.append(0)#5
        p.accelerations.append(0)#6
        p.accelerations.append(0)#7
        p.accelerations.append(0)#8
        p.accelerations.append(0)#9
        p.accelerations.append(0)#10
        
        trajectory.points.append(p);
        
        x_traj = i*0.001
        i = i + 1

    # move UAV in y axis
    i = 0
    y_traj = 0
    while y_traj < (y_ref - S):
        p=JointTrajectoryPoint();
        p.positions.append(x_traj)#0
        p.positions.append(y_traj)#1
        p.positions.append(z_traj)#2
        p.positions.append(0)#3
        p.positions.append(0)#4
        p.positions.append(0)#5
        p.positions.append(0)#6
        p.positions.append(0)#7	
        p.positions.append(0)#8		
        p.positions.append(0)#9
        p.positions.append(0)#10

        p.velocities.append(0)#0

        p.velocities.append(0)#1
        p.velocities.append(0)#2
        p.velocities.append(0)#3
        p.velocities.append(0)#4
        p.velocities.append(0)#5
        p.velocities.append(0)#6
        p.velocities.append(0)#7
        p.velocities.append(0)#8
        p.velocities.append(0)#9
        p.velocities.append(0)#10

        p.accelerations.append(0)#0
        p.accelerations.append(0)#1
        p.accelerations.append(0)#2
        p.accelerations.append(0)#3
        p.accelerations.append(0)#4
        p.accelerations.append(0)#5
        p.accelerations.append(0)#6
        p.accelerations.append(0)#7
        p.accelerations.append(0)#8
        p.accelerations.append(0)#9
        p.accelerations.append(0)#10
        
        trajectory.points.append(p);
        
        y_traj = i*0.001
        i = i + 1

    return trajectory	

#######################################################

#def uav_pose_cb(msg):

#    uav_pose_cb.p = msg.pose.position

#def get_pose_ref():
    
#    v = 1
#    rate10 = 10
    
#    pose_ref = uav_pose_cb.p + v * rate10

#    return pose_ref

#######################################################


def talker():
    #rospy.Subscriber('/uav/pose', PoseStamped, uav_pose_cb, queue_size=10)
    rospy.Subscriber('line_point_1', Vector3, line_point_1_cb, queue_size=10)
    rospy.Subscriber('line_point_2', Vector3, line_point_2_cb, queue_size=10)
    #pub_pose_ref = rospy.Publisher('/uav/pose_ref', Pose, queue_size=10)
    pub = rospy.Publisher('/uav/joint_trajectory', JointTrajectory, queue_size=10)
    rospy.init_node('trajectory_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    first = 1
    while not rospy.is_shutdown():
        if first == 1:
            first = 0
            trajectory = get_trajectory()
            pub.publish(trajectory)

        #uav_pose_cb(msg)
        #pose_ref = get_pose_ref()
        #pub.publish(pose_ref)

        line_point_1_cb(msg1)
        line_point_2_cb(msg2)

        rate.sleep()
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

