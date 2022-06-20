
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include "asa047.hpp"

#define MAX_DATA 1000
#define PI 3.14159265
int data_count=0;
bool first = true;
sensor_msgs::MagneticField data[MAX_DATA];
int received=0;
std::string magnetic_vector_topic;


void magnetometer_callback(const sensor_msgs::MagneticField::ConstPtr& msg){
	if (data_count>=MAX_DATA)
	{
		first =false;
		data_count=0;
	}
	received ++;
	data[data_count]=*msg;
	data_count++;
}


geometry_msgs::Vector3 CrossProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B) {
	geometry_msgs::Vector3 c_P;
   c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
   c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
   c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
   return c_P;
}


double DotProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B)
{
	return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}


geometry_msgs::Vector3 normalize_vector(geometry_msgs::Vector3 v)
{
	double dist=sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	v.x = v.x / dist;
	v.y = v.y / dist;
	v.z = v.z / dist;
	return v;
}

double measurements[MAX_DATA];
int measurements_count = 0;
double frequency = 50;
double magnetometer_sampling_time=0.01282;
double maxa, mina;
double Objective(double x[3])
{
	double a = x[0] * 1000000;
	double fi = x[1];
	double dc = x[2] * 1000000;

	double y = 0;
	for (int i = 0; i < measurements_count; i++)
	{
		double t = i * magnetometer_sampling_time; //
		double value = dc + a * sin(2 * 3.14159265 * frequency * t + fi);

		y = y + (value - measurements[i] * 1000000) * (value - measurements[i]*1000000);
	}
	return y;
}

double Optimize(int n, int f, double *data, double *phase)
{
	int icount, numres, ifault;
	double initial_xd[3] = {0.00001, 1, 0.000001};
	double optim_krit;
	double optim_x[3];
	double optim_x_final[3];

	double step[3] = {0.000001, 0.1, 0.0000001};

	measurements_count = n;
	double minvalue=0;
	double maxvalue=0;
	double sum=0;
	for (int i = 0; i < n; i++)
	{
		measurements[i] = data[i];
		if (data[i] < minvalue || i==0)
		{
			minvalue = data[i];
		}
		if (data[i] > maxvalue || i==0)
		{
			maxvalue = data[i];
		}
		sum = sum + data[i];
	}
	double mean = sum / n;
	double min = 1000000000000;
	std::cout<<std::endl;
	maxa = (maxvalue - minvalue) / 2 * 1.2 * 1000000;
	mina = (maxvalue - minvalue) / 2 * 0.8 * 1000000;
	initial_xd[0] = (maxvalue - minvalue) / 2;
	initial_xd[1] = 0.5;;
	initial_xd[2] = mean;
	nelmin(Objective, 3, initial_xd, optim_x, &optim_krit, 1.0e-8, step, 10, 500, &icount, &numres, &ifault );

	if (fabs(optim_x[0])<(maxvalue - minvalue) / 2 /5)
	{
		initial_xd[0] = (maxvalue - minvalue) / 2;
		initial_xd[1] = 0.5+3;;
		initial_xd[2] = mean;
		nelmin(Objective, 3, initial_xd, optim_x, &optim_krit, 1.0e-8, step, 10, 500, &icount, &numres, &ifault );

	}
	std::cout<<"max min vs optim "<<(maxvalue - minvalue) / 2<<" "<<optim_x[0]<<std::endl;
	if (optim_krit < min)
	{
		min=optim_krit;
		optim_x_final[0] = optim_x[0];
		optim_x_final[1] = optim_x[1];
		optim_x_final[2] = optim_x[2];
	}
	*phase = optim_x_final[1];
	return optim_x_final[0];
}


void ispisi_vektor(geometry_msgs::Vector3 vektor, std::string str)
{
	std::cout<<str<<" "<<vektor.x<<" "<<vektor.y<< " "<<vektor.z<<std::endl;
}

double GetDFTAngle(int selected_axis, int point_count, int start_element)
{
	double real=0, img=0;
	double current_data;
	int k = 50;
	for (int i=0;i<point_count; i++)
	{
		int current_element = start_element + i - point_count;
		if (current_element < 0) current_element = current_element + MAX_DATA;

		if (selected_axis == 0)
		{
			current_data = data[current_element].magnetic_field.x;
		}
		if (selected_axis == 1)
		{
			current_data = data[current_element].magnetic_field.y;
		}
		if (selected_axis == 2)
		{
			current_data = data[current_element].magnetic_field.z;
		}
		real = real + current_data * cos((2 * i * k * 3.14159265) / point_count );
		img = img - current_data * sin((2 * i * k * 3.14159265) / point_count );
	}
	std::cout<<"dft real img "<<real<<" "<<img<<std::endl;
	return atan2(img,real);
}

int last1=0;
int last2=0;
geometry_msgs::Vector3 GetVectorUsingOptimization(int n, int f, ros::Time *stamp)
{
	geometry_msgs::Vector3 result;
	double x[MAX_DATA];
	double y[MAX_DATA];
	double z[MAX_DATA];
	double time[MAX_DATA];
	int countx=0, county = 0, countz=0 ;
	int data_count_current = data_count;
	double poc=0,kr=0;
//	int current_element_old;
	ros::Time begin_stamp_x;
	ros::Time begin_stamp_y;
	ros::Time begin_stamp_z;
	ros::Time begin_stamp;

	bool first_x=true;
	bool first_y=true;
	bool first_z=true;

	double start_element_x=0;
	double start_element_y=0;
	double start_element_z=0;

	for (int i=0;i<n;i++)
	{
		int current_element = data_count_current + i - n;
		if (current_element<0) current_element = current_element + MAX_DATA;
		x[countx] = data[current_element].magnetic_field.x;
		y[county] = data[current_element].magnetic_field.y;
		z[countz] = data[current_element].magnetic_field.z;


		if (i>0)
		{
			if (x[countx] != x[countx-1])
			{
				countx++;
				if (first_x)
				{
					start_element_x = current_element;
					begin_stamp_x = data[current_element].header.stamp;
					first_x = false;
				}


			}
			if (y[county] != y[county-1])
			{
				county++;
				if (first_y)
				{
					start_element_y = current_element;
					begin_stamp_y = data[current_element].header.stamp;
					first_y = false;
				}
			}
			if (z[countz] != z[countz-1])
			{
				if (first_z)
				{
					start_element_z = current_element;
					begin_stamp_z = data[current_element].header.stamp;
					first_z = false;
				}
				countz++;
			}


		}
		else
		{
			countx++;
			county++;
			countz++;
		}

	}
	double anglex,angley,anglez;
	std::cout<<magnetic_vector_topic<< " x ";
	result.x = Optimize(countx, f, x, &anglex);
	std::cout<<magnetic_vector_topic<<" y ";
	result.y = Optimize(county, f, y, &angley);
	std::cout<<magnetic_vector_topic<<" z ";
	result.z = Optimize(countz, f, z, &anglez);
	bool add_pi[3]={false,false,false};
	if (result.x < 0){ anglex = anglex + 3.14159; result.x = fabs(result.x); add_pi[0]= true;}
	if (result.y < 0){ angley = angley + 3.14159; result.y = fabs(result.y); add_pi[1]=true;}
	if (result.z < 0){ anglez = anglez + 3.14159; result.z = fabs(result.z); add_pi[2]=true;}
	std::cout<<" angle init "<<anglex<< " "<< angley<<" "<<anglez<<std::endl;

	anglex = anglex - ((int)(anglex / (2 * 3.14159)))*2*3.14159;
	angley = angley - ((int)(angley / (2 * 3.14159)))*2*3.14159;
	anglez = anglez - ((int)(anglez / (2 * 3.14159)))*2*3.14159;

	double maxx = result.x, angle_max = anglex; begin_stamp = begin_stamp_x;
	int selected_angle=0;
	if (result.y > maxx) { maxx = result.y; angle_max=angley; begin_stamp = begin_stamp_y; selected_angle = 1;}
	if (result.z > maxx) { maxx = result.z; angle_max=anglez; begin_stamp = begin_stamp_z; selected_angle = 2;}

	if (fabs(anglex - angle_max) > 3.14159/2 && fabs(fabs(anglex - angle_max) - 3.14159 * 2) > 3.14159/2)
	{
		result.x = -result.x;
	}
	if (fabs(angley - angle_max) > 3.14159/2 && fabs(fabs(angley - angle_max) - 3.14159 * 2) > 3.14159/2)
	{
		result.y = -result.y;
	}
	if (fabs(anglez - angle_max) > 3.14159/2 && fabs(fabs(anglez - angle_max) - 3.14159 * 2) > 3.14159/2)
	{
		result.z = -result.z;
	}
	if (angle_max < 0) angle_max += 2 * 3.14159265;
	ispisi_vektor(result," vektor ");
	std::cout<<" angle "<<anglex<< " "<< angley<<" "<<anglez<<"     angle max"<< angle_max<<std::endl;

	double fft_angle=0;
	if (selected_angle == 0)
	{
		fft_angle = GetDFTAngle(selected_angle, 200, start_element_x);
	}
	if (selected_angle == 1)
	{
		fft_angle = GetDFTAngle(selected_angle, 200, start_element_y);
	}
	if (selected_angle == 2)
	{
		fft_angle = GetDFTAngle(selected_angle, 200, start_element_z);
	}

	std::cout<<"fft angle "<<fft_angle<<std::endl;


	ros::Duration difference(0, 0.02 * (2 * 3.14159 - fft_angle) / 2 / 3.14159265 * 1000000000);
	std::cout<<"time difference "<< 0.02 * (2 * 3.14159 - fft_angle) / 2 / 3.14159265*1000<<std::endl;

	ros::Time new_time(begin_stamp+difference);
	std::cout<<"time "<<new_time.nsec/1000000<<std::endl;

	int present= new_time.nsec/1000000;


	int diff1 = present - last1;
	int diff2 = present - last2;

	if (diff1 < 0) diff1 = diff1 + 1000;
	if (diff2 < 0) diff2 = diff2 + 1000;

	diff1=diff1 % 20;
	diff2=diff2 % 20;
	std::cout<< " time difference since last "<<diff1<<" "<<diff2<<std::endl;
/*
	if (diff1 <= 5 && diff2 <= 5)
	{
		if (diff1<diff2)
		{
			ros::Duration correction(0, diff1 * 1000000);
			new_time = new_time - correction;
			std::cout<<"correction "<<diff1<<std::endl;
		}
		else
		{
			ros::Duration correction(0, diff2 * 1000000);
			new_time = new_time - correction;
			std::cout<<"correction "<<diff2<<std::endl;

		}
	}
	if (diff1 >= 15 & diff2 >= 15)
	{
		if (diff1<diff2)
		{
			ros::Duration correction(0, diff2 * 1000000);
			new_time = new_time - correction;
			std::cout<<"correction "<<diff2<<std::endl;

		}
		else
		{
			ros::Duration correction(0, diff1 * 1000000);
			new_time = new_time - correction;
			std::cout<<"correction "<<diff1<<std::endl;

		}
	}
	if (diff1>5  && diff1 <15 && diff2 >5 && diff2<15)
	{
		ros::Duration correction(0, diff2 * 1000000);
		new_time = new_time - correction;
		std::cout<<"correction "<<diff2<<std::endl;


	}*/
	last2 = last1;
	last1 = present;


	std::cout<<"stamp "<<new_time.sec<<" "<<new_time.nsec<<std::endl;

	*stamp = new_time;
	return result;
}



double Median (double v1, double v2, double v3)
{
	if (v1 >= v2 && v1 <= v3) return v1;
	if (v1 <= v2 && v1 >= v3) return v1;
	if (v2 >= v1 && v2 <= v3) return v2;
	if (v2 <= v1 && v2 >= v3) return v2;
	return v3;
}
geometry_msgs::Vector3 Median(geometry_msgs::Vector3 v1,
		geometry_msgs::Vector3 v2,
		geometry_msgs::Vector3 v3
		)
{
	geometry_msgs::Vector3 rez;
	rez.x= Median(v1.x, v2.x, v3.x);
	rez.y= Median(v1.y, v2.y, v3.y);
	rez.z= Median(v1.z, v2.z, v3.z);
	return rez;

}

int main (int argc, char** argv){
    ros::init(argc, argv, "magnetic_field_vector_fast");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    int magnetic_field_frequency, cycles_for_analysis;
    std::string magnetometer_topic;
    std::string magnetometer_frame="/magnetometer1";

    int number_of_points_for_analysis;
    nh_ns.param("magnetometer_topic", magnetometer_topic, (std::string) "/imu_magnetic0");
    nh_ns.param("vector_topic", magnetic_vector_topic, (std::string) "/magnetic_vector0");
    nh_ns.param("magnetometer_frame", magnetometer_frame, (std::string) "/magnetometer0");
    nh_ns.param("magnetic_field_frequency", magnetic_field_frequency, 50);
    nh_ns.param("number_of_points_for_analysis", number_of_points_for_analysis, 200);
    nh_ns.param("number_of_cycles_for_analysis", cycles_for_analysis, 5);
    nh_ns.param("magnetometer_sampling_time", magnetometer_sampling_time,0.01282);


    ros::Subscriber magnetometer_subscriber = nh.subscribe(magnetometer_topic, 1000, magnetometer_callback);

    ros::Publisher magnetic_vector_publisher = nh.advertise<sensor_msgs::MagneticField>(magnetic_vector_topic, 1000);

    ros::Rate loop_rate(1000);

    geometry_msgs::Vector3 vector;
    geometry_msgs::Vector3 vector_final;
    geometry_msgs::Vector3 vector_final_old;
    geometry_msgs::Vector3 vector_final_old_old;

    
    int counter=0;
    int cycle_count = 0;
    double x_size = 0;
    double y_size = 0;
    double z_size = 0;
    double vector_size = 0;
    vector_final.x = 0;
    vector_final.y = 0;
    vector_final.z = 0;
    vector_final_old.x = 0;
    vector_final_old.y = 0;
    vector_final_old.z = 0;
    vector_final_old_old.x = 0;
    vector_final_old_old.y = 0;
    vector_final_old_old.z = 0;


    tf::Transform transform1;
    tf::TransformBroadcaster br;



    while(ros::ok()){

        ros::spinOnce();
        if (first == false && received > number_of_points_for_analysis)
        {
        	std::cout<<"received "<<received<<std::endl;
        	double pom1=0,pom2=0,pom3=0;

        	ros::Time stamp;
        	vector=GetVectorUsingOptimization(number_of_points_for_analysis, magnetic_field_frequency , &stamp);
        	received=0;

    		if (vector_final.x != vector_final.x) vector_final.x = 0;
    		if (vector_final.y != vector_final.y) vector_final.y = 0;
    		if (vector_final.z != vector_final.z) vector_final.z = 0;
    		geometry_msgs::Vector3 pomocni_vektor = vector;


    		vector = Median(vector, vector_final_old, vector_final_old_old);



//        	ispisi_vektor(vector_final,"final ");

    		vector_final.x = /*vector_final.x / 1000000* 0.9 + */vector.x;// * 0.1;
          	vector_final.y = /*vector_final.y / 1000000 * 0.9 +*/ vector.y;// * 0.1;
        	vector_final.z = /*vector_final.z / 1000000 * 0.9 + */vector.z;// * 0.1;
        	vector_final.x *= 1000000;
        	vector_final.y *= 1000000;
        	vector_final.z *= 1000000;

    		vector_final_old_old = vector_final_old;
    		vector_final_old = pomocni_vektor;




            geometry_msgs::Vector3 pomocni;
            pomocni.x=1;
            pomocni.y=0;
            pomocni.z=0;
            geometry_msgs::Vector3 kros=normalize_vector(CrossProduct(pomocni,normalize_vector(vector_final)));

            double w=1+DotProduct(pomocni,normalize_vector(vector_final));

            transform1.setOrigin(tf::Vector3(0,0,0));
            transform1.setRotation(tf::Quaternion(kros.x, kros.y, kros.z, w));
            br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), magnetometer_frame, magnetic_vector_topic));

            sensor_msgs::MagneticField publish_field;
            publish_field.magnetic_field = vector_final;
            publish_field.header.stamp = stamp;

        	magnetic_vector_publisher.publish(publish_field);
            x_size=0;
            y_size=0;
            z_size=0;
            vector_size=0;
        	cycle_count = 0;
        	std::cout<<magnetometer_frame<<std::endl;


        }
        loop_rate.sleep();


    }
}


