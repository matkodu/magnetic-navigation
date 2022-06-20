#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

/*
	Source code for publishing /tf from odometry topics in bag file.
*/

class tf_sub_pub
{

	// Topic subscribers: /firefly/ground_truth/odometry 
	// 		      /red/odometry
	// /hawk1/vrpn_client/estimated_odometry

	public:
		// constructor	
		tf_sub_pub()
		{
		 sub = n.subscribe("/base_link", 1000, &tf_sub_pub::callback, this);
		}


		//	***	Subscriber callback	***
		void callback(const nav_msgs::Odometry::ConstPtr& msg)
		{
		  double roll, pitch, yaw;
		  static tf::TransformBroadcaster br;
		  tf::Transform transform;

		  // Drone position
		  transform.setOrigin(
			tf::Vector3(
				msg->pose.pose.position.x, 
				msg->pose.pose.position.y, 
				msg->pose.pose.position.z));
		  
		  // Orientation quaternion
		  tf::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w);
		  
		  // 3x3 Rotation matrix from quaternion
		    tf::Matrix3x3 m(q);

		  // Roll Pitch and Yaw from rotation matrix
		  m.getRPY(roll, pitch, yaw);
		  
		  // Normalizing values
		  q.normalize();
		  transform.setRotation(q);

		  // Publishing tf
		  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/base_link", "/magnetometer0"));		  
		}

	private:
		  ros::NodeHandle n;
		  tf::TransformBroadcaster br;
		  ros::Subscriber sub;
};


// 	***	Node main function      ***         

int main(int argc, char **argv)
{
 
 ros::init(argc, argv, "rosbag_sub_pub");
 tf_sub_pub my_tf_sub_pub;

 
 ros::spin();

 return 0;
}


