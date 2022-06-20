#!/usr/bin/env python
# license removed for brevity

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
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
      
    for i in range(999):
        p=JointTrajectoryPoint();
        p.positions.append(0)#0
        p.positions.append(0)#1
        p.positions.append(i*0.001)#2
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
    for i in range(999):
        p=JointTrajectoryPoint();
        p.positions.append(i*0.001)#0
        p.positions.append(0)#1
        p.positions.append(1)#2
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

    for i in range(999):
        p=JointTrajectoryPoint();
        p.positions.append(1)#0
        p.positions.append(i*0.001)#1
        p.positions.append(1)#2
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
    for i in range(999):
        p=JointTrajectoryPoint();
        p.positions.append(1)#0
        p.positions.append(1)#1
        p.positions.append(1)#2
        p.positions.append(0)#3
        p.positions.append(0)#4
        p.positions.append(i*0.001)#5
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


    for i in range(999):
        p=JointTrajectoryPoint();
        p.positions.append(1)#0
        p.positions.append(1)#1
        p.positions.append(1+i*0.001)#2
        p.positions.append(0)#3
        p.positions.append(0)#4
        p.positions.append(1)#5
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

    for i in range(999):
        p=JointTrajectoryPoint();
        p.positions.append(1-i*0.001)#0
        p.positions.append(1-i*0.001)#1
        p.positions.append(2-i*0.001)#2
        p.positions.append(0)#3
        p.positions.append(0)#4
        p.positions.append(1)#5
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

    return trajectory	
  
  # /uav/joint_trajectory multi_dof_trajectory

def talker():
    pub = rospy.Publisher('/uav/joint_trajectory', JointTrajectory, queue_size=10)
    rospy.init_node('trajectory_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    first = 1
    while not rospy.is_shutdown():
        if first == 1:
            first = 0
            trajectory = get_trajectory()
            pub.publish(trajectory)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

