
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>


#include "asa047.hpp"


//using namespace mrpt;
//using namespace mrpt::math;
//using namespace mrpt::system;
using namespace std;

sensor_msgs::MagneticField  received_vector0;
sensor_msgs::MagneticField received_vector1;
sensor_msgs::MagneticField  received_vector2;

void magnetic_vector0_callback(const sensor_msgs::MagneticField::ConstPtr& msg){

	received_vector0 = *msg;

}

void magnetic_vector1_callback(const sensor_msgs::MagneticField::ConstPtr& msg){
	received_vector1 = *msg;
}

void magnetic_vector2_callback(const sensor_msgs::MagneticField::ConstPtr& msg){
	received_vector2 = *msg;
}

inline geometry_msgs::Vector3 CrossProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B) {
	geometry_msgs::Vector3 c_P;
   c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
   c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
   c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
   return c_P;
}

inline geometry_msgs::Vector3 sum_vector(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2) {
	geometry_msgs::Vector3 v3;
	v3.x = v1.x + v2.x;
	v3.y = v1.y + v2.y;
	v3.z = v1.z + v2.z;
   return v3;
}

inline geometry_msgs::Vector3 normalize_vector(geometry_msgs::Vector3 v)
{
	double dist=sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	v.x = v.x / dist;
	v.y = v.y / dist;
	v.z = v.z / dist;
	return v;
}

inline double VectorSize(geometry_msgs::Vector3 vector)
{
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

inline double DotProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B)
{
	return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}
tf::StampedTransform  base_link;

inline geometry_msgs::Vector3 transform_vector(geometry_msgs::Vector3 vector, tf::Matrix3x3 rotation)
{
	geometry_msgs::Vector3 result;

	result.x = rotation.getRow(0).getX() * vector.x +  rotation.getRow(0).getY() * vector.y + rotation.getRow(0).getZ() * vector.z;
	result.y = rotation.getRow(1).getX() * vector.x +  rotation.getRow(1).getY() * vector.y + rotation.getRow(1).getZ() * vector.z;
	result.z = rotation.getRow(2).getX() * vector.x +  rotation.getRow(2).getY() * vector.y + rotation.getRow(2).getZ() * vector.z;
	return result;
}

void ispisi_vektor(geometry_msgs::Vector3 vektor, std::string str)
{
	cout<<str<<" "<<vektor.x<<" "<<vektor.y<< " "<<vektor.z<<endl;
}

bool ispis=false;

inline geometry_msgs::Vector3 getClosestPointOnLine(geometry_msgs::Vector3 line_point, geometry_msgs::Vector3 line_vector)
{
	double x1 = line_point.x;
	double y1 = line_point.y;
	double z1 = line_point.z;
	double vx = line_vector.x;
	double vy = line_vector.y;
	double vz = line_vector.z;
	geometry_msgs::Vector3 p1, vector, result;
	p1.x = x1;
	p1.y = y1;
	p1.z = z1;
	vector.x = vx;
	vector.y = vy;
	vector.z = vz;
//	if (ispis) ispisi_vektor(line_point, " closest point line point ");
//	if (ispis) ispisi_vektor(line_vector, " closest point line vektor ");

	double t=-DotProduct(p1, vector) / VectorSize(vector)/*/VectorSize(p1)*/;
	result.x = x1 + t * vx;
	result.y = y1 + t * vy;
	result.z = z1 + t * vz;
//	if (ispis) ispisi_vektor(result, " closest point result ");

	return result;
}

void ispisi_vektor(tf::Vector3 vektor, std::string str)
{
	cout<<str<<" "<<vektor.getX()<<" "<<vektor.getY()<< " "<<vektor.getZ()<<endl;
}

inline geometry_msgs::Vector3 GetFieldVectorOneWire(geometry_msgs::Vector3 line_point, geometry_msgs::Vector3 line_vector, double current, tf::StampedTransform magnetometer_location)
{
	line_point.x = line_point.x - magnetometer_location.getOrigin().getX();
	line_point.y = line_point.y - magnetometer_location.getOrigin().getY();
	line_point.z = line_point.z - magnetometer_location.getOrigin().getZ();

	geometry_msgs::Vector3 closest_point = getClosestPointOnLine(line_point, line_vector);

//    if (ispis) ispisi_vektor(line_point,"line point");
//    if (ispis) ispisi_vektor(closest_point,"closest point");
//    if (ispis)    cout<<" vector size "<<VectorSize(closest_point)<<endl;

    double distance = VectorSize(closest_point) ;

	geometry_msgs::Vector3 magnetic_vector = CrossProduct(closest_point, line_vector);
	magnetic_vector=normalize_vector(magnetic_vector);

	magnetic_vector.x = magnetic_vector.x / distance * current * 0.2;
	magnetic_vector.y = magnetic_vector.y / distance * current * 0.2;
	magnetic_vector.z = magnetic_vector.z / distance * current * 0.2;

	return magnetic_vector;
}
inline bool GetPowerLineLocation(geometry_msgs::Vector3 m_vector1,
					geometry_msgs::Vector3 m_vector2,
					tf::StampedTransform transform,
					geometry_msgs::Vector3 power_line_vector,
					geometry_msgs::Vector3 *power_line_point,
					tf::StampedTransform calib1,
					tf::StampedTransform calib2)
{
	//adjust vectors according to calibration
	tf::Matrix3x3 basis1 = calib1.getBasis();
	tf::Matrix3x3 basis2 = calib2.getBasis();

//	m_vector1 = transform_vector(m_vector1, basis1);
//	m_vector2 = transform_vector(m_vector2, basis2);

	//to do rotacija m_vector2 to m_vector1 frame  - za paralelne senzore nije potrebna

	//line between sensors does not point towards power line
	geometry_msgs::Vector3 towards_center1=CrossProduct(m_vector1, power_line_vector);
	geometry_msgs::Vector3 towards_center2=CrossProduct(m_vector2, power_line_vector);

	geometry_msgs::Vector3 a0;
	a0.x = 0; a0.y = 0; a0.z = 0;
	geometry_msgs::Vector3 b0;

	b0.x = (double) transform.getOrigin().x();
	b0.y = (double) transform.getOrigin().y();
	b0.z = (double) transform.getOrigin().z();

	geometry_msgs::Vector3 a = normalize_vector(towards_center1);
	geometry_msgs::Vector3 b = normalize_vector(towards_center2);
	geometry_msgs::Vector3 cn = normalize_vector(CrossProduct(b, a));

	cn=CrossProduct(b, a);
	geometry_msgs::Vector3 n2 = CrossProduct(b, cn);
	double dot1 = DotProduct(b0, n2)/DotProduct(a,n2);
	geometry_msgs::Vector3 a2 = a;
	a2.x *= dot1;
	a2.y *= dot1;
	a2.z *= dot1;
	geometry_msgs::Vector3 closest_approach = a2;

	*power_line_point = closest_approach;

	return true;
}


inline geometry_msgs::Vector3 operator-(geometry_msgs::Vector3 o1, tf::Vector3 o2)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x-o2.getX();
	rez.y=o1.y-o2.getY();
	rez.z=o1.z-o2.getZ();
	return rez;
}

inline geometry_msgs::Vector3 operator*(geometry_msgs::Vector3 o1, double o2)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x * o2;
	rez.y=o1.y * o2;
	rez.z=o1.z * o2;
	return rez;
}

inline geometry_msgs::Vector3 operator*(double o2, geometry_msgs::Vector3 o1)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x * o2;
	rez.y=o1.y * o2;
	rez.z=o1.z * o2;
	return rez;
}


inline geometry_msgs::Vector3 operator/(geometry_msgs::Vector3 o1,  double o2)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x / o2;
	rez.y=o1.y / o2;
	rez.z=o1.z / o2;
	return rez;
}


inline geometry_msgs::Vector3 operator+(geometry_msgs::Vector3 o1, tf::Vector3 o2)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x+o2.getX();
	rez.y=o1.y+o2.getY();
	rez.z=o1.z+o2.getZ();
	return rez;
}


inline geometry_msgs::Vector3 operator-(geometry_msgs::Vector3 o1, geometry_msgs::Vector3 o2)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x-o2.x;
	rez.y=o1.y-o2.y;
	rez.z=o1.z-o2.z;
	return rez;
}

double GetAngleBetweenVectors(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2)
{
	return acos( DotProduct(v1,v2)/VectorSize(v1)/VectorSize(v2));
}
inline geometry_msgs::Vector3 operator+(geometry_msgs::Vector3 o1, geometry_msgs::Vector3 o2)
{
	geometry_msgs::Vector3 rez;
	rez.x=o1.x+o2.x;
	rez.y=o1.y+o2.y;
	rez.z=o1.z+o2.z;

	return rez;
}


tf::StampedTransform m1_loc;
tf::StampedTransform m2_loc;
geometry_msgs::Vector3 m_0;
geometry_msgs::Vector3 m_1;
geometry_msgs::Vector3 m_2;
geometry_msgs::Vector3 line_vector;

geometry_msgs::Vector3 output_pose;
double skip=1;



int objective_count=0;

//objective function for optimization
//receives one point on the conductor and current going through the conductor
//assumes that two conductors have the same current and are on known distance from each other
void Objective(
	double *x, double *out_f)
{

	double space_between_wires=0.4;
	if (fabs(x[0])>2 || fabs(x[1])>2 || fabs(x[2])>2 || fabs(x[3])>40)
	{
		*out_f=1000000000;
		return;
	}


	*out_f=0;
	geometry_msgs::Vector3 point;
	point.x = x[0];
	point.y = x[1];
	point.z = x[2];


	tf::Matrix3x3 rot=base_link.getBasis();

	geometry_msgs::Vector3 line_vector_global = transform_vector(line_vector,rot);

	geometry_msgs::Vector3 z_vector;
	z_vector.x=0;
	z_vector.y=0;
	z_vector.z=-1;
	geometry_msgs::Vector3 pomocni3 = normalize_vector(CrossProduct(line_vector_global,z_vector));

	geometry_msgs::Vector3 line_vector_z = transform_vector(pomocni3,rot.inverse());



	double i1 = x[3];
	objective_count++;
	tf::StampedTransform m0_loc;
	tf::StampedTransform m12_loc;
	tf::StampedTransform calib0;

	//get transformation from magnetometer 1 to magnetometer 2
	tf::Transform t1= m1_loc.inverse() * m2_loc;
	m12_loc.setBasis(m2_loc.getBasis());
	m12_loc.setOrigin(m2_loc.getOrigin()-m1_loc.getOrigin());

	//calib0 is used for pose calibration, currently identity matrix
	calib0.setOrigin(tf::Vector3(0,0,0));
	calib0.setRotation(tf::Quaternion(0, 0, 0, 1));
	m0_loc.setOrigin(tf::Vector3(0,0,0));


	//calculate magnetic field  of magnetometers based only on one conductor
	geometry_msgs::Vector3 m00 = GetFieldVectorOneWire(point, line_vector, i1, m0_loc);
	geometry_msgs::Vector3 m01 = GetFieldVectorOneWire(point, line_vector, i1, m1_loc);
	geometry_msgs::Vector3 m02 = GetFieldVectorOneWire(point, line_vector, i1, m2_loc);
	geometry_msgs::Vector3 m10, m11, m12;

	//get magnetic field  of magnetometers of the second conductor
	m10 = m_0 - m00;
	m11 = m_1 - m01;
	m12 = m_2 - m02;
	if (ispis) ispisi_vektor(m00," m00 ");
	if (ispis) ispisi_vektor(m01," m01 ");
	if (ispis) ispisi_vektor(m02," m02 ");

	double min=1;
	geometry_msgs::Vector3 pomocni;
	if (fabs(line_vector.x)<min)
	{
		min=line_vector.x;
		pomocni.x=1;
		pomocni.y=0;
		pomocni.z=0;
	}
	if (fabs(line_vector.y)<min)
	{
		min=line_vector.y;
		pomocni.x=0;
		pomocni.y=1;
		pomocni.z=0;
	}
	if (fabs(line_vector.z)<min)
	{
		min=line_vector.z;
		pomocni.x=0;
		pomocni.y=0;
		pomocni.z=1;
	}


	geometry_msgs::Vector3 plv0, plv1, plv2,plp0, plp1, plp2;

	if (ispis) ispisi_vektor(line_vector_z," line vektor z ");

	plp0=point + line_vector_z * space_between_wires;
	if (ispis) ispisi_vektor(point," point ");
	if (ispis) ispisi_vektor(plp0," plp0 ");

	geometry_msgs::Vector3 m10n = GetFieldVectorOneWire(plp0, line_vector, i1, m0_loc);
	geometry_msgs::Vector3 m11n = GetFieldVectorOneWire(plp0, line_vector, i1, m1_loc);
	geometry_msgs::Vector3 m12n = GetFieldVectorOneWire(plp0, line_vector, i1, m2_loc);

	if (ispis) ispisi_vektor(m10n," m10n ");
	if (ispis) ispisi_vektor(m11n," m12n ");
	if (ispis) ispisi_vektor(m12n," m12n ");

	double dist = VectorSize(m10-m10n) +VectorSize(m11-m11n) + VectorSize(m12-m12n);
	double min_dist=dist;

	if (ispis) ispisi_vektor(m10-m10n," diff1 ");
	if (ispis) ispisi_vektor(m11-m11n," diff2 ");
	if (ispis) ispisi_vektor(m12-m12n," diff3 ");

	plp1=plp0;

	plp0=point + line_vector_z * space_between_wires;

/*	m10n = GetFieldVectorOneWire(plp0, line_vector_z, i1, m0_loc);
	m11n = GetFieldVectorOneWire(plp0, line_vector_z, i1, m1_loc);
	m12n = GetFieldVectorOneWire(plp0, line_vector_z, i1, m2_loc);
*/
	dist = VectorSize(m10-m10n) +VectorSize(m11-m11n) + VectorSize(m12-m12n);
	if (dist<min_dist){
		min_dist=dist;
		plp1=plp0;
	}
	plp0=point + line_vector_z * space_between_wires;


	dist = VectorSize(m_0+m00+m10n) +VectorSize(m_1+m01+m11n) + VectorSize(m_2+m02+m12n);

	if (ispis) ispisi_vektor(m_0+m00+m10n," diff1 neg");
	if (ispis) ispisi_vektor(m_1+m01+m11n," diff2 neg");
	if (ispis) ispisi_vektor(m_2+m02+m12n," diff3 neg");
	if (dist<min_dist){
		min_dist=dist;
		plp1=point + line_vector_z * space_between_wires;;
	}
	plp0=plp1;
	output_pose=plp0;

	geometry_msgs::Vector3 plp0_global = transform_vector(point,rot.inverse());
	geometry_msgs::Vector3 plp1_global = transform_vector(plp0,rot.inverse());
	geometry_msgs::Vector3 m1_loc_vector;
	m1_loc_vector.x=m1_loc.getOrigin().getX();
	m1_loc_vector.y=m1_loc.getOrigin().getY();
	m1_loc_vector.z=m1_loc.getOrigin().getZ();
	m1_loc_vector=transform_vector(m1_loc_vector,rot.inverse());
	if (ispis)
	{
		 ispisi_vektor(plp0_global," plp0 global ");
	}
	if (plp0_global.z<0)
	{
		geometry_msgs::Vector3 v1=m1_loc_vector-plp0_global;
		geometry_msgs::Vector3 v2=m1_loc_vector-plp1_global;
		geometry_msgs::Vector3 v3=plp0_global-plp1_global;
		geometry_msgs::Vector3 v4=plp1_global-plp0_global;
		if (ispis) ispisi_vektor(v1," v1");
		if (ispis) ispisi_vektor(v2," v2");
		if (ispis) ispisi_vektor(v3," v3");
		if (ispis) ispisi_vektor(v4," v4");

		double angle1=GetAngleBetweenVectors(v1,v4);
		double angle2=GetAngleBetweenVectors(v2,v3);
		if (ispis)
		{
			std::cout<<"angle "<<angle1<<" "<<angle2<<std::endl;
		}

		if (fabs(angle1)<3.14159/2 && fabs(angle2)<3.14159/2)
		{
			out_f[0]=100000;
			return;
		}

	}


//	std::cout<<min_dist<<" ";
	out_f[0] = min_dist;
}

double Objective1(double x[4])
{



	double initial_x[4];
	double f;
	initial_x[0] = x[0];  // x
	initial_x[1] = x[1];  // y
	initial_x[2] = x[2];  // y
	initial_x[3] = x[3];  // y
	Objective(initial_x,&f);
	return f;


}

double levmarq_final_error = 1e10;

double previousx=0,previousy=0,previousz=0,previousi=0;;
bool first=true;
void FindBestLocation(double *final,double *crit)
{
	double optim_x[4];
	double step[4]={0.01,0.01,0.01,1};
	double pose2[4];

	double initial_xd[4];
	double optimal_x[4];

	double xi, yi;
	double min=10000000000000;
	double y, optim_krit;
	int count_less_than_1000=0;
	skip=5;
	int icount, numres, ifault;

	double min1=2;
	//find vector not parallel with power line vector
	geometry_msgs::Vector3 pomocni;
	if (fabs(line_vector.x)<min1)
	{
		min1=fabs(line_vector.x);
		pomocni.x=1;
		pomocni.y=0;
		pomocni.z=0;
	}
	if (fabs(line_vector.y)<min1)
	{
		min1=fabs(line_vector.y);
		pomocni.x=0;
		pomocni.y=1;
		pomocni.z=0;
	}
	if (fabs(line_vector.z)<min1)
	{
		min1=fabs(line_vector.z);
		pomocni.x=0;
		pomocni.y=0;
		pomocni.z=1;
	}
	//find rotation matrix with z axis in line vector
	geometry_msgs::Vector3 pomocni2 = normalize_vector(CrossProduct(line_vector,pomocni));
	geometry_msgs::Vector3 pomocni1 = normalize_vector(CrossProduct(pomocni2,line_vector));
	geometry_msgs::Vector3 pomak;


	double finalxi=0;
	double finalyi=0;
	double finalii=0;
	for (double xi = -2; xi < 2 ; xi = xi + 0.1)
	{
//		std::cout<<xi<<" ";
		for (double yi = -2; yi < 2 ; yi = yi + 0.1)
		{

			double poc=1;
			double kr=20;
			pomak = pomocni1 * xi;
			pomak = pomak + pomocni2 * yi;
			for (double ii=10;ii<40;ii+=2)
			{
				pomak = pomocni1 * xi;
				pomak = pomak + pomocni2 * yi;
//double ii=27;


				initial_xd[0] = pomak.x;// 0;  // x
				initial_xd[1] = pomak.y;//yi;  // y
				initial_xd[2] = pomak.z;//xi;//zi;  // y
				initial_xd[3] = ii;//ii;  // y




				optim_krit = Objective1(initial_xd);
				double diff=fabs(pomak.x-previousx)/4+fabs(pomak.y-previousy)/4+fabs(pomak.z-previousz)/4+fabs(ii-previousi)/40;
				optim_krit=optim_krit*(1+diff);

//				std::cout<< optim_krit<<" ";
		//		if (optim_krit<500)
				optimal_x[0]=initial_xd[0];
				optimal_x[1]=initial_xd[1];
				optimal_x[2]=initial_xd[2];
				optimal_x[3]=initial_xd[3];
				levmarq_final_error=optim_krit;
//				std::cout<<optim_krit<<" ";
				if (levmarq_final_error<min)
				{
//					std::cout<<" best ";

					y = Objective1(optimal_x);
					ispis=false;
					min=levmarq_final_error;
					final[0] = optimal_x[0];
					final[1] = optimal_x[1];
					final[2] = optimal_x[2];
					final[3] = optimal_x[3];
					pose2[0]=output_pose.x;
					pose2[1]=output_pose.y;
					pose2[2]=output_pose.z;
					finalxi=xi;
					finalyi=yi;
					finalii=ii;

				}
			}
		}
//		std::cout<<std::endl;
	}

	for (double xi = finalxi-0.1; xi < finalxi+0.1 ; xi = xi + 0.01)
	{
		for (double yi = finalyi-0.1; yi < finalyi+0.1 ; yi = yi + 0.01)
		{

			double poc=1;
			double kr=20;

			pomak = pomocni1 * xi;
			pomak = pomak + pomocni2 * yi;
//			double ii=27;
			for (double ii=finalii-1;ii<finalii+1;ii+=0.2)
			{
				pomak = pomocni1 * xi;
				pomak = pomak + pomocni2 * yi;



				initial_xd[0] = pomak.x;// 0;  // x
				initial_xd[1] = pomak.y;//yi;  // y
				initial_xd[2] = pomak.z;//xi;//zi;  // y
				initial_xd[3] = ii;//ii;  // y




				optim_krit = Objective1(initial_xd);
				double diff=fabs(pomak.x-previousx)/4+fabs(pomak.y-previousy)/4+fabs(pomak.z-previousz)/4+fabs(ii-previousi)/40;
				optim_krit=optim_krit*(1+diff);
//				std::cout<< optim_krit<<" ";
//				if (optim_krit<500)
				optimal_x[0]=initial_xd[0];
				optimal_x[1]=initial_xd[1];
				optimal_x[2]=initial_xd[2];
				optimal_x[3]=initial_xd[3];
				levmarq_final_error=optim_krit;
				//std::cout<<optim_krit<<" ";

				if (levmarq_final_error<min)
				{
					y = Objective1(optimal_x);
					ispis=false;
					min=levmarq_final_error;
					final[0] = optimal_x[0];
					final[1] = optimal_x[1];
					final[2] = optimal_x[2];
					final[3] = optimal_x[3];
					pose2[0]=output_pose.x;
					pose2[1]=output_pose.y;
					pose2[2]=output_pose.z;


				}
			}
		}
	}

	previousx=final[0];
	previousy=final[1];
	previousz=final[2];
	previousi=final[3];
	first=false;
	initial_xd[0] = 0;// 0;  // x
	initial_xd[1] = -0.05;//yi;  // y
	initial_xd[2] = -0.190;//xi;//zi;  // y
	initial_xd[3] = 27;//ii;  // y
	ispis=false;
	double real_krit = Objective1(initial_xd);
//	std::cout<<"real krit "<<real_krit<<std::endl;
	ispis=false;
	/*
	initial_xd[0] = final[0];  // x
	initial_xd[1] = final[1];  // y
	initial_xd[2] = final[2];//z
	initial_xd[3] = final[3];//ii;

	nelmin(Objective1, 4, initial_xd, optim_x, &optim_krit, 1.0e-8, step, 10, 300, &icount, &numres, &ifault );
	final[0]=optim_x[0];
	final[1]=optim_x[1];
	final[2]=optim_x[2];
	final[3]=optim_x[3];
	min=optim_krit;


	initial_xd[0] = pose2[0];  // x
	initial_xd[1] = pose2[1];  // y
	initial_xd[2] = pose2[2];//zi;  // y
	initial_xd[3] = final[3];//ii;
	nelmin(Objective1, 4, initial_xd, optim_x, &optim_krit, 1.0e-8, step, 10, 300, &icount, &numres, &ifault );
	if (optim_krit<min)
	{
		final[0]=optim_x[0];
		final[1]=optim_x[1];
		final[2]=optim_x[2];
		final[3]=optim_x[3];
		min=optim_krit;
	}*/

	*crit=min;

}


bool GetPowerLinesLocation(sensor_msgs::MagneticField  m_vector0,
					sensor_msgs::MagneticField  m_vector1,
					sensor_msgs::MagneticField  m_vector2,
					tf::StampedTransform transform0,
					tf::StampedTransform transform1,
					geometry_msgs::Vector3 *power_line_vector,
					geometry_msgs::Vector3 *power_line_point0,
					geometry_msgs::Vector3 *power_line_point1,
					tf::StampedTransform calib0,
					tf::StampedTransform calib1,
					tf::StampedTransform calib2)
{
	tf::Matrix3x3 basis0 = calib0.getBasis();
	tf::Matrix3x3 basis1 = calib1.getBasis();
	tf::Matrix3x3 basis2 = calib2.getBasis();
	m1_loc=transform0;
	m2_loc=transform1;
	ros::Duration duration1;
	ros::Duration duration2;

	if (m_vector0.header.stamp> m_vector1.header.stamp)
		duration1= m_vector0.header.stamp-m_vector1.header.stamp;
	else
		duration1= m_vector1.header.stamp-m_vector0.header.stamp;

	if (m_vector0.header.stamp> m_vector2.header.stamp)
		duration2= m_vector0.header.stamp-m_vector2.header.stamp;
	else
		duration2= m_vector2.header.stamp-m_vector0.header.stamp;



	int d1=duration1.nsec/1000000;
	int d2=duration2.nsec/1000000;
//	std::cout<<" duration 1 "<<d1<<std::endl;
//	std::cout<<" duration 2 "<<d2<<std::endl;
		if (d1%20>5 && d1%20<15)
		{
			m_vector1.magnetic_field.x=-m_vector1.magnetic_field.x;
			m_vector1.magnetic_field.y=-m_vector1.magnetic_field.y;
			m_vector1.magnetic_field.z=-m_vector1.magnetic_field.z;


		}
		if (d2%20>5 && d2%20<15)
		{
			m_vector2.magnetic_field.x=-m_vector2.magnetic_field.x;
			m_vector2.magnetic_field.y=-m_vector2.magnetic_field.y;
			m_vector2.magnetic_field.z=-m_vector2.magnetic_field.z;

		}


	tf::Matrix3x3 base_link_rot=base_link.getBasis();

	geometry_msgs::Vector3 z_vector;
	z_vector.x=0;
	z_vector.y=0;
	z_vector.z=1;
//	ispisi_vektor(m_vector0.magnetic_field,"m0 ");
//	ispisi_vektor(m_vector1.magnetic_field,"m1 ");
//	ispisi_vektor(m_vector2.magnetic_field,"m2 ");

	m_0 = transform_vector(m_vector0.magnetic_field, base_link_rot);
	m_1 = transform_vector(m_vector1.magnetic_field, base_link_rot);
	m_2 = transform_vector(m_vector2.magnetic_field, base_link_rot);


	geometry_msgs::Vector3 pom_vector1 = (CrossProduct(m_0, z_vector));
	geometry_msgs::Vector3 pom_vector2 = (CrossProduct(m_1, z_vector));
	geometry_msgs::Vector3 pom_vector3 = (CrossProduct(m_2, z_vector));
	pom_vector1=transform_vector(pom_vector1, base_link_rot.inverse());
	pom_vector2=transform_vector(pom_vector2, base_link_rot.inverse());
	pom_vector3=transform_vector(pom_vector3, base_link_rot.inverse());

	m_0 = /*transform_vector(*/m_vector0.magnetic_field/*, basis0)*/;
	m_1 =/* transform_vector(*/m_vector1.magnetic_field/*, basis1)*/;
	m_2 =/* transform_vector(*/m_vector2.magnetic_field/*, basis2)*/;


	power_line_vector->x = (pom_vector1.x + pom_vector2.x + pom_vector3.x);
	power_line_vector->y = (pom_vector1.y + pom_vector2.y + pom_vector3.y);
	power_line_vector->z = (pom_vector1.z + pom_vector2.z + pom_vector3.z);


	*power_line_vector=normalize_vector(*power_line_vector);

	line_vector = *power_line_vector;



	line_vector=normalize_vector(line_vector);


	double optimal_x[4];
	double initial_x[4];



	initial_x[0] = 1.4;  // x
	initial_x[1] = 2.5;  // y
	initial_x[2] = 2.5;  // y
	initial_x[3] = 2.5;  // y


	double increments_x[4];
	increments_x[0]=0.001;;
	increments_x[2]=0.001;;
	increments_x[3]=0.001;;
	increments_x[3]=0.1;


	double min=100000000;
	double initial_xd[4];
	double optim_x[4];
	double step[4]={0.01,0.01,0.01,1};
	double optim_krit=0;
	double final[4]={0,0,0,0};
	double final1[4]={0,0,0,0};

	ispis=false;


	FindBestLocation(final,&min);
//	ispis=true;
	Objective1(final);
	ispis=false;
	cout<<"final "<<min<<"   x y z i "<<final[0]<<" "<<final[1]<<" "<<final[2]<<" "<<final[3]<<endl;
	power_line_point0->x = final[0];
	power_line_point0->y = final[1];
	power_line_point0->z = final[2];

	final1[0] = output_pose.x;
	final1[1] = output_pose.y;
	final1[2] = output_pose.z;
	power_line_point1->x = final1[0];
	power_line_point1->y = final1[1];
	power_line_point1->z = final1[2];



}

geometry_msgs::Vector3 getClosestPointOnLine(geometry_msgs::Vector3 line_point,
		geometry_msgs::Vector3 line_vector,
		geometry_msgs::Vector3 point)
{
	double x1 = line_point.x-point.x;
	double y1 = line_point.y-point.y;
	double z1 = line_point.z-point.z;
	double vx = line_vector.x;
	double vy = line_vector.x;
	double vz = line_vector.x;
	geometry_msgs::Vector3 p1, vector, result;
	p1.x = x1;
	p1.y = y1;
	p1.z = z1;
//	std::cout<<"transform base power_line "<<p1;
	vector.x = vx;
	vector.y = vy;
	vector.z = vz;
//	std::cout<<"line vector  "<<vector;
	double t=-DotProduct(p1,vector)/VectorSize(vector)/VectorSize(p1);

	result.x = x1 + t * vx;
	result.y = y1 + t * vy;
	result.z = z1 + t * vz;
	return result;

}


int main (int argc, char** argv){
    ros::init(argc, argv, "magnetic_field_localization");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    int refresh_rate;
    std::string magnetic_vector0, magnetic_vector1, magnetic_vector2, frame0, frame1, frame2, power_line_frame0, power_line_frame1;
    std::string vector0_calibration, vector1_calibration, vector2_calibration;
    nh_ns.param("magnetic_vector0", magnetic_vector0, (std::string) "/magnetic_vector0");
    nh_ns.param("magnetic_vector1", magnetic_vector1, (std::string)"/magnetic_vector1");
    nh_ns.param("magnetic_vector2", magnetic_vector2, (std::string)"/magnetic_vector2");
    nh_ns.param("vector0_calibration", vector0_calibration, (std::string) "/magn0_cal");
    nh_ns.param("vector1_calibration", vector1_calibration, (std::string)"/magn1_cal");
    nh_ns.param("vector2_calibration", vector2_calibration, (std::string)"/magn2_cal");

    nh_ns.param("vector0_frame", frame0, (std::string) "/magnetometer0"); //imu1
    nh_ns.param("vector1_frame", frame1, (std::string) "/magnetometer1"); //imu2
    nh_ns.param("vector2_frame", frame2, (std::string) "/magnetometer2");  //imu3
    nh_ns.param("power_line_frame0", power_line_frame0, (std::string) "/power_line0");
    nh_ns.param("power_line_frame1", power_line_frame1, (std::string) "/power_line1");


    ros::Subscriber magnetic_vector_subscriber1 = nh.subscribe(magnetic_vector0, 1, magnetic_vector0_callback);
    ros::Subscriber magnetic_vector_subscriber2 = nh.subscribe(magnetic_vector1, 1, magnetic_vector1_callback);
    ros::Subscriber magnetic_vector_subscriber3 = nh.subscribe(magnetic_vector2, 1, magnetic_vector2_callback);

    geometry_msgs::Vector3 power_line_vector;
    geometry_msgs::Vector3 power_line_point;

    geometry_msgs::Vector3 power_line_vector0;
    geometry_msgs::Vector3 power_line_point0;
    geometry_msgs::Vector3 power_line_vector1;
    geometry_msgs::Vector3 power_line_point1;

    geometry_msgs::Vector3 point0;
    geometry_msgs::Vector3 point1;
    point0.x=0;
    point0.y=0;
    point0.z=0;
    point1.x=0;
    point1.y=0;
    point1.z=0;


    ros::Rate loop_rate(10);
    tf::TransformListener listener;
    tf::TransformBroadcaster br;
    tf::StampedTransform  transform0;
    tf::StampedTransform  transform1;
    int counter=0;
    tf::StampedTransform cal1;
    cal1.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::StampedTransform cal2;
    cal2.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::StampedTransform cal3;
    cal3.setRotation(tf::Quaternion(0, 0, 0, 1));
    ros::Time t = ros::Time(0);


    try{
    	listener.lookupTransform("/magnetometer0", "/power_line_gound_truth0", t, cal1);
    	listener.lookupTransform("/magnetometer0", "/power_line_gound_truth1", t, cal2);
    	cout<<"gorund truth"<<endl;
    	ispisi_vektor(cal1.getOrigin(),"power line 0_n");
    	ispisi_vektor(cal2.getOrigin(),"power line 1_n");
    	cout<<endl<<endl;
    }
    catch (tf::TransformException ex){
    	ROS_ERROR("%s",ex.what());
    }



    while(ros::ok()){

        ros::spinOnce();
        tf::StampedTransform transform;

        t = ros::Time(0);



        try {
        	listener.lookupTransform(frame0, vector0_calibration, t, cal1);
        	listener.lookupTransform(frame1, vector1_calibration, t, cal2);
        	listener.lookupTransform(frame2, vector2_calibration, t, cal3);

        }
        catch (tf::TransformException ex){
        }

        try{


        	listener.lookupTransform(frame0, frame1, t, transform0);
        	listener.lookupTransform(frame0, frame2, t, transform1);
        	listener.lookupTransform("/base_link", frame0, t, base_link);

        	point0.x=transform0.getOrigin().getX();
            point0.y=transform0.getOrigin().getY();
            point0.z=transform0.getOrigin().getZ();

        	point1.x=transform1.getOrigin().getX();
            point1.y=transform1.getOrigin().getY();
            point1.z=transform1.getOrigin().getZ();


            bool rez=GetPowerLinesLocation(received_vector0,
            		received_vector1,
            		received_vector2,
					transform0,
					transform1,
					&power_line_vector,
					&power_line_point0,
					&power_line_point1,
					cal1,
					cal2,
					cal3);
            geometry_msgs::Vector3 p1 = getClosestPointOnLine(power_line_point0, power_line_vector, point0);
            geometry_msgs::Vector3 p2 = getClosestPointOnLine(power_line_point1, power_line_vector, point0);


            transform0.setOrigin(tf::Vector3(power_line_point0.x, power_line_point0.y, power_line_point0.z));
            transform1.setOrigin(tf::Vector3(power_line_point1.x, power_line_point1.y, power_line_point1.z));

            tf::Quaternion q;
            if (power_line_vector.x == power_line_vector.x &&
            		power_line_point0.x == power_line_point0.x &&
					power_line_point1.x == power_line_point1.x)
            {
            	geometry_msgs::Vector3 pomocni;
            	pomocni.x=1;
            	pomocni.y=0;
            	pomocni.z=0;
            	geometry_msgs::Vector3 kros=CrossProduct(pomocni,power_line_vector);
            	double w=1+DotProduct(pomocni,power_line_vector);

            	transform0.setRotation(tf::Quaternion(kros.x, kros.y, kros.z, w));

            	transform1.setRotation(tf::Quaternion(kros.x, kros.y, kros.z, w));


            	br.sendTransform(tf::StampedTransform(transform0, ros::Time::now(), frame0, power_line_frame0));
            	br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), frame0, power_line_frame1));
            }
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
        loop_rate.sleep();


    }
}


