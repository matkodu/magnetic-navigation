
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
//#include <math.h>


//cross product of two vectors
geometry_msgs::Vector3 CrossProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B) {
	geometry_msgs::Vector3 c_P;
   c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
   c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
   c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
   return c_P;
}

//sum of two vectors
geometry_msgs::Vector3 sum_vector(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2) {
	geometry_msgs::Vector3 v3;
	v3.x = v1.x + v2.x;
	v3.y = v1.y + v2.y;
	v3.z = v1.z + v2.z;
   return v3;
}

//normalize vector
geometry_msgs::Vector3 normalize_vector(geometry_msgs::Vector3 v)
{
	double dist=sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	v.x = v.x / dist;
	v.y = v.y / dist;
	v.z = v.z / dist;
	return v;
}

//vector eucledian size
double VectorSize(geometry_msgs::Vector3 vector)
{
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

//dot product of two vectors
double DotProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B)
{
	return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}


double count=0;

// calculate magnetic field of from one wire
//transform  - pose of magnetometer with respect to the wire
// current_amplitude - amplitude of the current flowing through the wire
// current_frequency - frequency of the current flowing through the wire
// sampling_frequency -sampling frequency of the magnetometer
geometry_msgs::Vector3 GetMagnetometerReadings(
		tf::StampedTransform transform,
		double current_amplitude,
		double current_frequency,
		double sampling_frequency)
{

	//get power line position and vector in magnetometer coordinate frame;
	double x0 = (double) transform.getOrigin().x();
	double y0 = (double) transform.getOrigin().y();
	double z0 = (double) transform.getOrigin().z();

	tf::Matrix3x3 basis=transform.getBasis();
	geometry_msgs::Vector3 point;
	geometry_msgs::Vector3 v_line;
	point.x = x0;
	point.y = y0;
	point.z = z0;

	v_line.x = basis.getColumn(0).getX();
	v_line.y = basis.getColumn(0).getY();
	v_line.z = basis.getColumn(0).getZ();


	//calculate powerline point closest to the magnetometer
	double t=-DotProduct(point,v_line);

	geometry_msgs::Vector3 closest;
	closest.x = x0 + t * v_line.x;
	closest.y = y0 + t * v_line.y;
	closest.z = z0 + t * v_line.z;

	//distance to closest point
	double d=sqrt((closest.x) * (closest.x) + (closest.y) * (closest.y) + (closest.z) * (closest.z));

	//direction of the magnetic field vector is cross product
	geometry_msgs::Vector3 vector=CrossProduct(closest, v_line);
	ros::Time time = ros::Time::now();

	double stamp=1. / sampling_frequency * count;
	double b=0;

	//solve case for too close distance
	if (d>0.01)
	{
		b = 2 * current_amplitude * 0.0001 / d * cos(2 *3.14159 * current_frequency * stamp);
	}
	else
	{
		b= 2 * current_amplitude * 0.0001 / 0.01 * cos(2 *3.14159 * current_frequency * stamp);
	}
	vector=normalize_vector(vector);

	vector.x = vector.x * b;
	vector.y = vector.y * b;
	vector.z = vector.z * b;

	return vector;
}



int main (int argc, char** argv){
    ros::init(argc, argv, "power_line_simulation");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    int refresh_rate;
    std::string magnetic_vector0, magnetic_vector1, frame0, frame1, frame2, base_frame , magnetometer_topic;
    double magnetometer_sampling_frequency;

    //read parameters

    //tf frame of magnetometer
    nh_ns.param("magnetometer_frame", frame0, (std::string) "/magnetometer0");
    

    //topic to publish magnetometer readings
    nh_ns.param("magnetometer_topic", magnetometer_topic, (std::string) "/imu_magnetic0");
	

    //sampling frequency of magnetometer
    nh_ns.param("magnetometer_sampling_frequency", magnetometer_sampling_frequency, (double) 200 );

    std::vector<std::string> power_lines_frames_list;
    std::vector<double> power_lines_currents_list;
    std::vector<double> power_lines_frequenties_list;

    // powerline tf pose frame - position defines one point on the line, orientation of vector x presents orientation of the powerline
    nh_ns.getParam ("simulated_power_line_frames", power_lines_frames_list);

    //current amplitude of individual power line
    nh_ns.getParam ("simulated_power_line_currents", power_lines_currents_list);

    //current frequency of individual power line
    nh_ns.getParam ("simulated_power_line_frequencies", power_lines_frequenties_list);


    geometry_msgs::Vector3 magnetometer_vector;
    geometry_msgs::Vector3 power_line_point;
    ros::Rate loop_rate(magnetometer_sampling_frequency);
    tf::TransformListener listener;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Transform transform1;
    int counter=0;
        //old: sensor_msgs::MagneticField (problem different type than in magnetic_field_localization.cpp)
	ros::Publisher magnetic_field_pub = nh.advertise<sensor_msgs::MagneticField>(magnetometer_topic, 1000); 
    while(ros::ok()){

        ros::spinOnce();
        tf::StampedTransform transform;
        ros::Time t = ros::Time(0);
        try{
        	magnetometer_vector.x=0;
        	magnetometer_vector.y=0;
        	magnetometer_vector.z=0;

        	//loop all power lines
        	for (int i=0;i<power_lines_frames_list.size();i++)
        	{
        		listener.lookupTransform(frame0, power_lines_frames_list[i], t, transform);

        		//calculate measurement from power line i
        		geometry_msgs::Vector3 rez=GetMagnetometerReadings(transform,
        				power_lines_currents_list[i],
        				power_lines_frequenties_list[i],
						magnetometer_sampling_frequency);

        		//sum measurement
            	magnetometer_vector.x += rez.x;//*1000;
            	magnetometer_vector.y += rez.y;//*1000;
            	magnetometer_vector.z += rez.z;//*1000;
        	}

        	//publish magnetic field
        	count++;
            sensor_msgs::MagneticField data;
        	data.magnetic_field = magnetometer_vector;
            data.header.stamp = ros::Time::now();
            magnetic_field_pub.publish(data);
        }
        catch (tf::TransformException ex){
         // ROS_ERROR("%s",ex.what());
        }

        loop_rate.sleep();


    }
}


