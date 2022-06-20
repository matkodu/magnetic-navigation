
#include <ros/ros.h>

#include <serial/serial.h>

#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

//old: geometry_msgs::Vector3
geometry_msgs::Vector3 received_vector0; 
geometry_msgs::Vector3 received_vector1;

void magnetic_vector0_callback(const geometry_msgs::Vector3::ConstPtr& msg){

	received_vector0 = *msg;
}
void magnetic_vector1_callback(const geometry_msgs::Vector3::ConstPtr& msg){
	received_vector1 = *msg;
}
geometry_msgs::Vector3 CrossProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B) {
	geometry_msgs::Vector3 c_P;
   c_P.x = v_A.y * v_B.z - v_A.z * v_B.y;
   c_P.y = -(v_A.x * v_B.z - v_A.z * v_B.x);
   c_P.z = v_A.x * v_B.y - v_A.y * v_B.x;
   return c_P;
}
geometry_msgs::Vector3 sum_vector(geometry_msgs::Vector3 v1, geometry_msgs::Vector3 v2) {
	geometry_msgs::Vector3 v3;
	v3.x = v1.x + v2.x;
	v3.y = v1.y + v2.y;
	v3.z = v1.z + v2.z;
   return v3;
}
geometry_msgs::Vector3 normalize_vector(geometry_msgs::Vector3 v)
{
	double dist=sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	v.x = v.x / dist;
	v.y = v.y / dist;
	v.z = v.z / dist;
	return v;
}
double VectorSize(geometry_msgs::Vector3 vector)
{
	return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}
double DotProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B)
{
	return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}
geometry_msgs::Vector3 transform_vector(geometry_msgs::Vector3 vector, tf::Matrix3x3 rotation)
{
	geometry_msgs::Vector3 result;

	result.x = rotation.getRow(0).getX() * vector.x +  rotation.getRow(0).getY() * vector.y + rotation.getRow(0).getZ() * vector.z;
	result.y = rotation.getRow(1).getX() * vector.x +  rotation.getRow(1).getY() * vector.y + rotation.getRow(1).getZ() * vector.z;
	result.z = rotation.getRow(2).getX() * vector.x +  rotation.getRow(2).getY() * vector.y + rotation.getRow(2).getZ() * vector.z;
	return result;
}
bool GetPowerLineLocation(geometry_msgs::Vector3 m_vector1,
					geometry_msgs::Vector3 m_vector2,
					tf::StampedTransform transform,
					geometry_msgs::Vector3 *power_line_vector,
					geometry_msgs::Vector3 *power_line_point,
					tf::StampedTransform calib1,
					tf::StampedTransform calib2)
{

//	ROS_INFO_STREAM("vector 1 "<<m_vector1.x<<" "<<m_vector1.y<<" "<<m_vector1.z);
//	ROS_INFO_STREAM("vector 2 "<<m_vector2.x<<" "<<m_vector2.y<<" "<<m_vector2.z);

	//adjust vectors according to calibration
	tf::Matrix3x3 basis1 = calib1.getBasis();
	tf::Matrix3x3 basis2 = calib2.getBasis();

	m_vector1 = transform_vector(m_vector1, basis1);
	m_vector2 = transform_vector(m_vector2, basis2);
	ROS_INFO_STREAM("vector 1 tran"<<m_vector1.x<<" "<<m_vector1.y<<" "<<m_vector1.z);
	ROS_INFO_STREAM("vector 2 tran"<<m_vector2.x<<" "<<m_vector2.y<<" "<<m_vector2.z);


	//to do rotacija m_vector2 to m_vector1 frame  - za paralelne senzore nije potrebna
	*power_line_vector=CrossProduct(m_vector1, m_vector2);

	ROS_INFO_STREAM("power_line "<<(*power_line_vector).x<<" "<<(*power_line_vector).y<<" "<<(*power_line_vector).z);

	if (VectorSize(m_vector1) * 0.1 > VectorSize(*power_line_vector)
			&& VectorSize(m_vector2) * 0.1 > VectorSize(*power_line_vector) && 0==1)
	{
		// line between sensors points towards powerline
		if (VectorSize(m_vector1) > 0.8 * VectorSize(m_vector2)
		&& VectorSize(m_vector1) < 1.2 * VectorSize(m_vector2))
		{
			//paralel to the powerline
			return false;
		}
		else
		{

			double max_axis=m_vector1.x, max_axis2=m_vector2.x;
			if (fabs(m_vector1.y) > fabs(max_axis)) {max_axis=m_vector1.y; max_axis2 = m_vector2.y;}
			if (fabs(m_vector1.z) > fabs(max_axis)) {max_axis=m_vector1.z; max_axis2 = m_vector2.z;}
			if (max_axis * max_axis2>0) //same direction
			{
				double dist=sqrt((double)(transform.getOrigin().x()*transform.getOrigin().x()+
						transform.getOrigin().y()*transform.getOrigin().y()+
						transform.getOrigin().z()*transform.getOrigin().z()));
				double a=VectorSize(m_vector1);
				double b=VectorSize(m_vector2);
				if (a < b)
				{
					double t = a;
					a = b;
					b = t;
				}
				double r = (sqrt(dist * dist + 4 * a / b) - dist) / 2;
				if (VectorSize(m_vector1) > VectorSize(m_vector2))
				{
					double directionx = m_vector1.x - m_vector2.x;
					double directiony = m_vector1.y - m_vector2.y;
					double directionz = m_vector1.z - m_vector2.z;
					power_line_point->x = m_vector1.x + directionx * r;
					power_line_point->y = m_vector1.y + directiony * r;
					power_line_point->z = m_vector1.z + directionz * r;
				}
				else
				{
					double directionx = m_vector2.x - m_vector1.x;
					double directiony = m_vector2.y - m_vector1.y;
					double directionz = m_vector2.z - m_vector1.z;
					power_line_point->x = m_vector2.x + directionx * r;
					power_line_point->y = m_vector2.y + directiony * r;
					power_line_point->z = m_vector2.z + directionz * r;
				}
			}
			else
			{
				//opposite direction todo

			}
		}
	}
	else
	{
		//line between sensors does not point towards power line

		geometry_msgs::Vector3 towards_center1=CrossProduct(m_vector1, *power_line_vector);
		geometry_msgs::Vector3 towards_center2=CrossProduct(m_vector2, *power_line_vector);
//		ROS_INFO_STREAM("mag vector1 "<< m_vector1 <<" mag_vector2 "<< m_vector2);

		ROS_INFO_STREAM("towards center1 "<< towards_center1 <<" towards_center_2 "<< towards_center2);


		//closest point between two lines in 3d space;
/*		geometry_msgs::Vector3 w0;
		w0.x=(double) transform.getOrigin().x();
		w0.y=(double) transform.getOrigin().y();
		w0.z=(double) transform.getOrigin().z();

		double a = DotProduct(towards_center1, towards_center1);
		double b = DotProduct(towards_center1, towards_center2);
		double c = DotProduct(towards_center2, towards_center2);
		double d = DotProduct(towards_center1, w0);
		double e = DotProduct(towards_center2, w0);
		double sc = (b * e - c * d) / (a * c - b * b);
		double tc = (a * e - b * d) / (a * c - b * b);
		power_line_point->x = towards_center1.x * sc;
		power_line_point->y = towards_center1.y * sc;
		power_line_point->z = towards_center1.z * sc;*/
		geometry_msgs::Vector3 a0;
		a0.x = 0; a0.y=0; a0.z=0;
		geometry_msgs::Vector3 b0;
		b0.x=(double) transform.getOrigin().x();
		b0.y=(double) transform.getOrigin().y();
		b0.z=(double) transform.getOrigin().z();

		geometry_msgs::Vector3 a=normalize_vector(towards_center1);
		geometry_msgs::Vector3 b=normalize_vector(towards_center2);
		geometry_msgs::Vector3 cn=normalize_vector(CrossProduct(b, a));

		std::cout<<"a "<<a.x<< " "<<a.y<< " "<<a.z<<std::endl;
		std::cout<<"b "<<b.x<< " "<<b.y<< " "<<b.z<<std::endl;
		std::cout<<"b0 "<<b0.x<< " "<<b0.y<< " "<<b0.z<<std::endl;

		std::cout<<"cn "<<cn.x<< " "<<cn.y<< " "<<cn.z<<std::endl;


		cn=CrossProduct(b, a);
		geometry_msgs::Vector3 n2=CrossProduct(b, cn);
		std::cout<<"n2 "<<n2.x<< " "<<n2.y<< " "<<n2.z<<std::endl;
		double dot1=DotProduct(b0, n2)/DotProduct(a,n2);
		std::cout<<"dot "<<dot1<<std::endl;
		geometry_msgs::Vector3 a2=a;
		a2.x *= dot1;
		a2.y *= dot1;
		a2.z *= dot1;
		geometry_msgs::Vector3 closest_approach=a2;



		/*geometry_msgs::Vector3 sum1 = b0;  //b0-a0;
		double dot1=DotProduct(sum1, a);
		geometry_msgs::Vector3 projection_ = a;
		projection_.x*=dot1;
		projection_.y*=dot1;
		projection_.z*=dot1;

		double dot2=DotProduct(sum1, cn);
		geometry_msgs::Vector3 sum2 = projection_;
		sum2.x = -sum2.x;
		sum2.y = -sum2.y;
		sum2.z = -sum2.z;
		geometry_msgs::Vector3 sum3 = cn;

		sum3.x *= -dot2;
		sum3.y *= -dot2;
		sum3.z *= -dot2;

		geometry_msgs::Vector3 rejection = sum_vector(sum_vector(sum1, sum2), sum3);

		double faktor=VectorSize(rejection)/DotProduct(b, normalize_vector(rejection));
		geometry_msgs::Vector3 sum4 = b;
		sum4.x *= -faktor;
		sum4.y *= -faktor;
		sum4.z *= -faktor;
		geometry_msgs::Vector3 closest_approach=sum_vector(b0, sum4);*/

		*power_line_point=closest_approach;

		ROS_INFO_STREAM("power_line "<<(*power_line_point).x<<" "<<(*power_line_point).y<<" "<<(*power_line_point).z);
		*power_line_vector = normalize_vector(*power_line_vector);

                

	}
	return true;
}

geometry_msgs::Vector3 getClosestPointOnLine(geometry_msgs::Vector3 line_point,geometry_msgs::Vector3 line_vector, geometry_msgs::Vector3 point)
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
	std::cout<<"t "<<t<<std::endl;
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
    std::string magnetic_vector0, magnetic_vector1, frame0, frame1, power_line_frame;
    std::string vector0_calibration,vector1_calibration;
    nh_ns.param("magnetic_vector0", magnetic_vector0, (std::string) "/magnetic_vector0"); //old: magnetic_vector0
    nh_ns.param("magnetic_vector1", magnetic_vector1, (std::string)"/magnetic_vector1");  //old: magnetic_vector1
    nh_ns.param("vector0_calibration", vector0_calibration, (std::string) "/magn1_cal");
    nh_ns.param("vector1_calibration", vector1_calibration, (std::string)"/magn2_cal");

    nh_ns.param("vector0_frame", frame0, (std::string) "/magnetometer0"); //old: vector0_frame
    nh_ns.param("vector1_frame", frame1, (std::string) "/magnetometer1"); //old: vector1_frame
    nh_ns.param("power_line_frame", power_line_frame, (std::string) "/power_line"); // power_line_frame


    // ros::Subscriber magnetic_field_sub1 = n.subscribe<sensor_msgs::MagneticField>("/imu_magnetic0", 1000, chatterCallback1);


    ros::Subscriber magnetic_vector_subscriber1 = nh.subscribe("/magnetic_vector0", 10, magnetic_vector0_callback); //magnetic_vector0
    ros::Subscriber magnetic_vector_subscriber2 = nh.subscribe("/magnetic_vector1", 10, magnetic_vector1_callback);  //magnetic_vector1


    // Publishers for powerline points
    ros::Publisher test_pub_1 = nh.advertise<geometry_msgs::Vector3>("line_point_1", 20);
    ros::Publisher test_pub_2 = nh.advertise<geometry_msgs::Vector3>("line_point_2", 20);

    geometry_msgs::Vector3 power_line_vector;
    geometry_msgs::Vector3 power_line_point;
    geometry_msgs::Vector3 power_line_point_new;

    geometry_msgs::Vector3 power_line_vector1;
    geometry_msgs::Vector3 power_line_point1;
    geometry_msgs::Vector3 power_line_vector2;
    geometry_msgs::Vector3 power_line_point2;
    geometry_msgs::Vector3 power_line_vector3;
    geometry_msgs::Vector3 power_line_point3;

    geometry_msgs::Vector3 point0;
    geometry_msgs::Vector3 point1;
    point0.x=0;
    point0.y=0;
    point0.z=0;
    point1.x=0;
    point1.y=0;
    point1.z=0;


    ros::Rate loop_rate(1);
    tf::TransformListener listener;
    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Transform transform1;
    int counter=0;
    tf::StampedTransform cal1;
    cal1.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf::StampedTransform cal2;
    cal2.setRotation(tf::Quaternion(0, 0, 0, 1));
    ros::Time t = ros::Time(0);
    while(ros::ok()){

        ros::spinOnce();
        tf::StampedTransform transform;

        t = ros::Time(0);
        try {
        	listener.lookupTransform(frame0, vector0_calibration, t, cal1);
        	listener.lookupTransform(frame1, vector1_calibration, t, cal2);


        }
        catch (tf::TransformException ex){
        	ROS_ERROR("%s",ex.what());
        }
//    	ROS_INFO_STREAM("cal1 r0 " << cal1.getBasis().getRow(0).getX() << " " <<cal1.getBasis().getRow(0).getY() << " "<<cal1.getBasis().getRow(0).getZ()<<std::endl);
 //   	ROS_INFO_STREAM("cal1 r1 " << cal1.getBasis().getRow(1).getX() << " " <<cal1.getBasis().getRow(1).getY() << " "<<cal1.getBasis().getRow(1).getZ()<<std::endl);
 //   	ROS_INFO_STREAM("cal1 r2 " << cal1.getBasis().getRow(2).getX() << " " <<cal1.getBasis().getRow(2).getY() <<" " <<cal1.getBasis().getRow(2).getZ()<<std::endl);

 //   	ROS_INFO_STREAM("cal2 r0 " << cal2.getBasis().getRow(0).getX() << " " <<cal2.getBasis().getRow(0).getY()<<" " << cal2.getBasis().getRow(0).getZ()<<std::endl);
 //   	ROS_INFO_STREAM("cal2 r1 " << cal2.getBasis().getRow(1).getX() << " " <<cal2.getBasis().getRow(1).getY()<<" " << cal2.getBasis().getRow(1).getZ()<<std::endl);
 //   	ROS_INFO_STREAM("cal2 r2 " << cal2.getBasis().getRow(2).getX() << " " <<cal2.getBasis().getRow(2).getY()<<" " << cal2.getBasis().getRow(2).getZ()<<std::endl);

        try{


        	listener.lookupTransform(frame0, frame1, t, transform);
            point1.x=transform.getOrigin().getX();
            point1.y=transform.getOrigin().getY();
            point1.z=transform.getOrigin().getZ();

//        	ROS_INFO_STREAM("transform" <<transform.getOrigin().getX()<<" "<<transform.getOrigin().getY()<< " "<<transform.getOrigin().getZ());

            bool rez=GetPowerLineLocation(received_vector0, received_vector1, transform, &power_line_vector, &power_line_point, cal1, cal2);
            geometry_msgs::Vector3 p1 = getClosestPointOnLine(power_line_point,power_line_vector,point0);
            geometry_msgs::Vector3 p2 = getClosestPointOnLine(power_line_point,power_line_vector,point1);
            std::cout<<"0 dist1 dist2 magn1 magn2  d1/d2 m2/m1 "<<VectorSize(p1)<<" "<<VectorSize(p2)<<" "<<VectorSize(received_vector0)<<" "<<VectorSize(received_vector1)<<" "
            	<<" "<<VectorSize(p1)/VectorSize(p2)<<" "<<VectorSize(received_vector1)/VectorSize(received_vector0)<<std::endl;

            
            

            /*if (fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0))>0.25)
            {
            	double d1=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                received_vector0.y = -received_vector0.y;
                bool rez1=GetPowerLineLocation(received_vector0, received_vector1, transform, &power_line_vector1, &power_line_point1, cal1, cal2);

                p1 = getClosestPointOnLine(power_line_point1,power_line_vector,point0);
                p2 = getClosestPointOnLine(power_line_point1,power_line_vector,point1);
                std::cout<<"1 dist1 dist2 magn1 magn2  d1/d2 m2/m1 "<<VectorSize(p1)<<" "<<VectorSize(p2)<<" "<<VectorSize(received_vector0)<<" "<<VectorSize(received_vector1)<<" "
                	<<" "<<VectorSize(p1)/VectorSize(p2)<<" "<<VectorSize(received_vector1)/VectorSize(received_vector0)<<std::endl;
                double d2=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                if (d2 < d1 && d2<0.5)
                {
                	d1=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                	std::cout<<"d1 = "<<d1<<std::endl;
                	power_line_vector=power_line_vector1;
                	power_line_point=power_line_point1;
                }

                received_vector1.y = -received_vector1.y;
                bool rez2=GetPowerLineLocation(received_vector0, received_vector1, transform, &power_line_vector2, &power_line_point2, cal1, cal2);
                p1 = getClosestPointOnLine(power_line_point2,power_line_vector,point0);
                p2 = getClosestPointOnLine(power_line_point2,power_line_vector,point1);
                std::cout<<"2 dist1 dist2 magn1 magn2  d1/d2 m2/m1 "<<VectorSize(p1)<<" "<<VectorSize(p2)<<" "<<VectorSize(received_vector0)<<" "<<VectorSize(received_vector1)<<" "
                	<<" "<<VectorSize(p1)/VectorSize(p2)<<" "<<VectorSize(received_vector1)/VectorSize(received_vector0)<<std::endl;
                d2=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                if (d2 < d1 && d2<0.5)
                {
                	d1=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                	std::cout<<"d1 = "<<d1<<std::endl;
                	power_line_vector=power_line_vector2;
                	power_line_point=power_line_point2;
                }


                received_vector0.y = -received_vector0.y;
                bool rez3=GetPowerLineLocation(received_vector0, received_vector1, transform, &power_line_vector3, &power_line_point3, cal1, cal2);
                p1 = getClosestPointOnLine(power_line_point3,power_line_vector,point0);
                p2 = getClosestPointOnLine(power_line_point3,power_line_vector,point1);
                received_vector1.y = -received_vector1.y;
                std::cout<<"3 dist1 dist2 magn1 magn2  d1/d2 m2/m1 "<<VectorSize(p1)<<" "<<VectorSize(p2)<<" "<<VectorSize(received_vector0)<<" "<<VectorSize(received_vector1)<<" "
                	<<" "<<VectorSize(p1)/VectorSize(p2)<<" "<<VectorSize(received_vector1)/VectorSize(received_vector0)<<std::endl;
                d2=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                if (d2 < d1 && d2<0.5)
                {
                	d1=fabs(VectorSize(p1)/VectorSize(p2)-VectorSize(received_vector1)/VectorSize(received_vector0));
                	std::cout<<"d1 = "<<d1<<std::endl;
                	power_line_vector=power_line_vector3;
                	power_line_point=power_line_point3;
                }

            }*/

            // 2nd powerline point (1st point + vector)
            power_line_point_new.x = power_line_point.x + power_line_vector.x;
            power_line_point_new.y = power_line_point.y + power_line_vector.y;
            power_line_point_new.z = power_line_point.z + power_line_vector.z;
//TODO

            
            power_line_point.x = float(power_line_point.x);        
            power_line_point.y = float(power_line_point.y); 
            power_line_point.z = float(power_line_point.z);

            power_line_point_new.x = float(power_line_point_new.x);        
            power_line_point_new.y = float(power_line_point_new.y); 
            power_line_point_new.z = float(power_line_point_new.z); 


            test_pub_1.publish(power_line_point);
            test_pub_2.publish(power_line_point_new);

                        ROS_INFO_STREAM(rez<< "TEST vector "<< power_line_vector<<" TEST  point1 "<<power_line_point << "TEST point2"<<power_line_point_new);
            transform1.setOrigin(tf::Vector3(power_line_point.x, power_line_point.y, power_line_point.z));
            tf::Quaternion q;
            if (power_line_vector.x==power_line_vector.x && power_line_point.x==power_line_point.x)
            {
//            	q.setsetRotation(tf::Vector3(power_line_vector.x, power_line_vector.y, power_line_vector.z),0.1);
            	geometry_msgs::Vector3 pomocni;
            	pomocni.x=1;
            	pomocni.y=0;
            	pomocni.z=0;
            	geometry_msgs::Vector3 kros=CrossProduct(pomocni,power_line_vector);
            	double w=1+DotProduct(pomocni,power_line_vector);

//            	geometry_msgs::Vector3 kros=CrossProduct(power_line_vector,pomocni);
//            	double w=1+DotProduct(power_line_vector,pomocni);
            	//            	transform1.setRotation(tf::Quaternion(power_line_vector.x, power_line_vector.y, power_line_vector.z,0));
            	transform1.setRotation(tf::Quaternion(kros.x, kros.y, kros.z, w));

            	br.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), frame0, power_line_frame));
            }
        }
        catch (tf::TransformException ex){
          ROS_ERROR("%s",ex.what());
        }
        loop_rate.sleep();


    }
}


