#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>



int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_trajectory");
  
  ros::NodeHandle n;
  ros::NodeHandle nh_ns("~");

  ros::Rate loop_rate(100);


  trajectory_msgs::JointTrajectory trajectory;
  ros::Publisher traj_pub = n.advertise<trajectory_msgs::JointTrajectory>("/uav/joint_trajectory", 1);
  bool first=true;
 	for (int i=0;i<100;i++)
	{	
	    ros::spinOnce();
	
	    loop_rate.sleep();
	}
  while (ros::ok())
  {
	if (first == false)
	{
	    ros::spinOnce();
	
	    loop_rate.sleep();
 continue;
	}
	first=false;
	trajectory.header.stamp=ros::Time::now();
	trajectory.header.frame_id="base_link";
	trajectory.joint_names.push_back("x");
	trajectory.joint_names.push_back("y");
	trajectory.joint_names.push_back("z");

	trajectory.joint_names.push_back("dummy");
	trajectory.joint_names.push_back("dummy1");
	trajectory.joint_names.push_back("yaw");

	trajectory.joint_names.push_back("joint1");
	trajectory.joint_names.push_back("joint2");
	trajectory.joint_names.push_back("joint3");
	trajectory.joint_names.push_back("joint4");
	trajectory.joint_names.push_back("joint5");
	ros::spinOnce();

	for (int i = 0; i <= 1000; i += 1)
	{
		trajectory_msgs::JointTrajectoryPoint p;
		p.positions.push_back(0);//0
		p.positions.push_back(0);//1
		p.positions.push_back(1);//2
		p.positions.push_back(0);//3
		p.positions.push_back(0);//4
		p.positions.push_back(i*0.001);//5
		p.positions.push_back(i*0.001+0.0001);//6
		p.positions.push_back(i*0.001+0.0002);//7	
		p.positions.push_back(i*0.001+0.0003);//8		
		p.positions.push_back(i*0.001+0.0004);//9
		p.positions.push_back(i*0.001+0.0005);//10

		p.velocities.push_back(0);//0
		p.velocities.push_back(0);
		p.velocities.push_back(0);
		p.velocities.push_back(0);
		p.velocities.push_back(0);
		p.velocities.push_back(0);//5
		p.velocities.push_back(0.01);
		p.velocities.push_back(0.01);
		p.velocities.push_back(0.01);
		p.velocities.push_back(0.01);
		p.velocities.push_back(0.01);//10

		p.accelerations.push_back(0);//0
		p.accelerations.push_back(0);
		p.accelerations.push_back(0);
		p.accelerations.push_back(0);
		p.accelerations.push_back(0);
		p.accelerations.push_back(0);//5
		p.accelerations.push_back(0);
		p.accelerations.push_back(0);
		p.accelerations.push_back(0);
		p.accelerations.push_back(0);
		p.accelerations.push_back(0);//10


		trajectory.points.push_back(p);
	}	
	traj_pub.publish(trajectory);
	

  }



}
