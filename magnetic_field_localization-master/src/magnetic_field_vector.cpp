
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#define MAX_DATA 1000
#define PI 3.14159265
int data_count=0;
sensor_msgs::MagneticField data[MAX_DATA];
void magnetometer_callback(const sensor_msgs::MagneticField::ConstPtr& msg){
	if (data_count>=MAX_DATA)
	{
		data_count=0;
	}

	data[data_count]=*msg;
	data_count++;
}
class DFT_Coeff {
public:
	double real, img;
	DFT_Coeff() {
		real = 0.0;
		img = 0.0;
	}
};
double DFT(double* time, double* data, int n,double f,double *angle)
{
	double cosine[MAX_DATA];
	double sine[MAX_DATA];
	double sumax=0;
	double sumay=0;
	double min=100000000;
	int k;
	int krez=0;
	for (k = 0; k < n / 2; k++)
	{
		if (fabs((time[n - 1] - time[0]) / k - 1 / f) < min)
		{
			min=fabs((time[n - 1] - time[0]) / k - 1 / f);
			krez=k;
		}
	}
	k=krez;
//	ROS_INFO_STREAM("element "<<krez<<" "<<(time[n - 1] - time[0]) / k );
	for (int i = 0; i < n; i++) {
		cosine[i] = cos((2 * i * k * PI) / n);
		sine[i] = sin((2 * i * k * PI) / n);
	}
	DFT_Coeff dft_value;
	for (int i = 0; i < n; i++) {
		dft_value.real += data[i] * cosine[i];
		dft_value.img += data[i] * sine[i];
	}
//	ROS_INFO_STREAM(k<<" "<< (time[n - 1] - time[0]) / k<<" "<< sqrt(dft_value.real * dft_value.real + dft_value.img * dft_value.img) / n * 2);
	if (fabs((time[n - 1] - time[0]) / k - 1 / f) < 1 / f / 20)
	{
		sumax=sumax + dft_value.real / n * 2;
		sumay=sumay + dft_value.img / n * 2;
	}
	*angle=atan2(sumay,sumax);
	return sqrt(sumax*sumax+sumay*sumay);


}
double Median(double* data, int n)
{
	for (int i = 0; i < n - 1; i++)
	{
		for (int j = i + 1; j < n; j++)
		{
			if (data[i] > data[j])
			{
				double t = data[i];
				data[i] = data[j];
				data[j] = t;
			}
		}
	}
	return data[n / 2];
}
void GetSumOfDifferences(double* data1, double* data2, int n,double *result1,double *result2)
{
	double suma1 = 0;
	double suma2 = 0;
	double pod1;
	double pod2;
	for (int i = 1; i < n; i++)
	{
		pod1=0;
		pod2=0;
		if (data1[i] - data1[i - 1] > 0)  pod1 = 1;
		if (data1[i] - data1[i - 1] < 0)  pod1 = -1;
		if (data2[i] - data2[i - 1] > 0)  pod2 = 1;
		if (data2[i] - data2[i - 1] < 0)  pod2 = -1;

		suma1 = suma1 + fabs(pod1);
		suma2 = suma2 + fabs(pod1 - pod2);

	}
	*result1 = suma1;
	*result2 = suma2;

}

double GetPeak(double* time, double* data, int n, double f)
{
	int c1 = 0;
	int c2 = 0;
	double* p = new double[n];
	double* b = new double[n];
	double dif1, dif2;
	for (int i = 1; i < n - 1; i++)
	{
		dif1 = data[i] - data[i - 1];
		dif2 = data[i] - data[i + 1];
		if (dif1 == 0 && i > 2) {
			dif1 = data[i] - data[i - 2];
		}
		if (dif2 == 0 && i < n - 2) {
			dif2 = data[i] - data[i + 2];
		}


		if (dif1 > 0) {
			if (dif2 > 0) {
				p[c1] = data[i];
				c1 = c1 + 1;
			}
		}
		if (dif1 < 0) {
			if (dif2 < 0) {
				b[c2] = data[i];
				c2 = c2 + 1;
			}
		}
	}
	double medianp = Median(p, c1);
	double medianb = Median(b, c2);


	double amp = medianp - medianb;
	if (c1 == 0 || c2 == 0)
	{
		amp = 0;
	}
	return amp;
}
geometry_msgs::Vector3 GetVectorUsingPeak(int n, int f, double *suma1,double *suma2, double *suma3)
{
	geometry_msgs::Vector3 result;
	double x[MAX_DATA];
	double y[MAX_DATA];
	double z[MAX_DATA];
	double time[MAX_DATA];

	for (int i=0;i<n;i++)
	{
		x[i] = data[i].magnetic_field.x;
		y[i] = data[i].magnetic_field.y;
		z[i] = data[i].magnetic_field.z;
	}
	double anglex,angley,anglez;
	result.x = GetPeak(time, x, n, f);
	result.y = GetPeak(time, y, n, f);
	result.z = GetPeak(time, z, n, f);

	GetSumOfDifferences(x, y, n,suma1,suma2);
	GetSumOfDifferences(x, z, n,suma1,suma3);


	return result;
}


geometry_msgs::Vector3 GetVector(int n, int f)
{
	geometry_msgs::Vector3 result;
	double x[MAX_DATA];
	double y[MAX_DATA];
	double z[MAX_DATA];
	double time[MAX_DATA];

	for (int i=0;i<n;i++)
	{
		x[i] = data[i].magnetic_field.x;
		y[i] = data[i].magnetic_field.y;
		z[i] = data[i].magnetic_field.z;
		time[i] = data[i].header.stamp.sec + (double) data[i].header.stamp.nsec / 1000000000.;
	}
	double anglex,angley,anglez;
	result.x = DFT(time, x, n, f, &anglex);
	result.y = DFT(time, y, n, f, &angley);
	result.z = DFT(time, z, n, f, &anglez);

	if (angley-anglex>3.14159)
	{
		angley=angley-2*3.14159;
	}
	if (angley-anglex<-3.14159)
	{
		angley=angley+2*3.14159;
	}
	if (anglez-anglex>3.14159)
	{
		anglez=anglez-2*3.14159;
	}
	if (anglez-anglex<-3.14159)
	{
		anglez=anglez+2*3.14159;
	}
	if (fabs(result.x) >= fabs(result.y) && fabs(result.x) >= fabs(result.z))
	{
		if (fabs(anglex-angley)>3.14159/2) result.y=-result.y;
		if (fabs(anglex-anglez)>3.14159/2) result.z=-result.z;
	} else if (fabs(result.y) >= fabs(result.x) && fabs(result.y) >= fabs(result.z))
	{
		if (fabs(anglex-angley)>3.14159/2) result.x=-result.x;
		if (fabs(angley-anglez)>3.14159/2) result.z=-result.z;
	}
	else
	{
		if (fabs(anglez-anglex)>3.14159/2) result.x=-result.x;
		if (fabs(angley-anglez)>3.14159/2) result.y=-result.y;
	}
	return result;
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

int main (int argc, char** argv){
    ros::init(argc, argv, "magnetic_field_vector");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    int magnetic_field_frequency, cycles_for_analysis;
    std::string magnetometer_topic;
    std::string magnetic_vector_topic;
    std::string magnetometer_frame="/magnetometer1";

    int number_of_points_for_analysis;
    nh_ns.param("magnetometer_topic", magnetometer_topic, (std::string) "/imu_magnetic0");
    nh_ns.param("vector_topic", magnetic_vector_topic, (std::string) "/magnetic_vector0");
    nh_ns.param("magnetometer_frame", magnetometer_frame, (std::string) "/magnetometer0");
    nh_ns.param("magnetic_field_frequency", magnetic_field_frequency, 50);
    nh_ns.param("number_of_points_for_analysis", number_of_points_for_analysis, 200);
    nh_ns.param("number_of_cycles_for_analysis", cycles_for_analysis, 5);


/*    if (magnetometer_topic=="/imu0/mag")///imu_magnetic0")
    {
    	magnetometer_frame="/magnetometer0";
    }*/

    ros::Subscriber magnetometer_subscriber = nh.subscribe(magnetometer_topic, 1000, magnetometer_callback);

    ros::Publisher magnetic_vector_publisher = nh.advertise<geometry_msgs::Vector3>(magnetic_vector_topic, 1000);

    ros::Rate loop_rate(1000);

    geometry_msgs::Vector3 vector;
    geometry_msgs::Vector3 vector_final;
    
    int counter=0;
    int cycle_count=0;
    double x_size=0;
    double y_size=0;
    double z_size=0;
    double vector_size=0;
    double suma1=0;
    double suma2=0;
    double suma3=0;
    tf::Transform transform1;
    tf::TransformBroadcaster br;
    while(ros::ok()){

        ros::spinOnce();
        if (data_count >= number_of_points_for_analysis)
        {
        	double pom1=0,pom2=0,pom3=0;
    		vector=GetVectorUsingPeak(number_of_points_for_analysis, magnetic_field_frequency,&pom1,&pom2,&pom3);
    		vector=GetVector(number_of_points_for_analysis, magnetic_field_frequency);
        	data_count=0;

/*    		suma1=suma1+pom1;
    		suma2=suma2+pom2;
    		suma3=suma3+pom3;
    		if (fabs(vector.x) > fabs(x_size))
    		{
    			x_size=vector.x;
    		}
    		if (fabs(vector.y) > fabs(y_size))
    		{
    			y_size=vector.y;
    		}
    		if (fabs(vector.z) > fabs(z_size))
    		{
    			z_size=vector.z;
    		}*/
        	cycle_count++;
        	if (cycle_count==cycles_for_analysis)
        	{
        		vector_final=vector;
        		vector_final.x *= 1000000;
        		vector_final.y *= 1000000;
        		vector_final.z *= 1000000;

        		/*        		vector_final.x=x_size;
        		vector_final.y=y_size;
        		vector_final.z=z_size;
        		if (suma2>suma1/2)
        		{
        			vector_final.y=-y_size;
        		}
        		if (suma3>suma1/2)
        		{
        			vector_final.z=-z_size;
        		}*/
        		ROS_INFO_STREAM("publishing " << magnetometer_frame << " "<< magnetometer_topic << " vector"<<vector_final);
//        		ROS_INFO_STREAM("direction sum1 sum2 sum3 "<<suma1<<" "<<suma2<<" "<< suma3);


            	geometry_msgs::Vector3 pomocni;
            	pomocni.x=1;
            	pomocni.y=0;
            	pomocni.z=0;
            	geometry_msgs::Vector3 kros=normalize_vector(CrossProduct(pomocni,normalize_vector(vector_final)));


            	double w=1+DotProduct(pomocni,normalize_vector(vector_final));
//            	transform1.setRotation(tf::Quaternion(power_line_vector.x, power_line_vector.y, power_line_vector.z,0));
            	transform1.setRotation(tf::Quaternion(kros.x, kros.y, kros.z, w));
            	br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), magnetometer_frame, magnetic_vector_topic));



        		magnetic_vector_publisher.publish(vector_final);
                x_size=0;
                y_size=0;
                z_size=0;
                suma1=0;
                suma2=0;
                suma3=0;
                vector_size=0;
        		cycle_count = 0;
        	}
        }
        loop_rate.sleep();


    }
}


