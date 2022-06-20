#include <ros/ros.h>
#include "std_msgs/String.h"
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <algorithm>    // std::sort

#define PI 3.14159265358979323846 
#define MEDIAN_ON 	1
#define POS_CALC	1
#define POLYG_DETECT	1
#define MEDIAN_WND_SZ	9
#define MEDIAN_IDX	4	//for window size of 9

#define NUM_OF_SENS	4 // software works for values 3 and 4
#define NUM_OF_POLYG 	((unsigned int)pow(2,NUM_OF_SENS))
static geometry_msgs::Vector3 magnetometer1, magnetometer2, magnetometer3, magnetometer4;	// vectors containing the sensor data 

static geometry_msgs::Vector3* const ptrSensData[NUM_OF_SENS] = {&magnetometer1, &magnetometer2, &magnetometer3, &magnetometer4 };	// pointer array to sensor data.
static const unsigned int sensCombs[6][2] = {	{1,2},		/* This array contains all possilbe sensor combinations for yaw and pitch calculations. First three entries are combs for 3 sensors only. */
						{1,3},
						{2,3},
						{1,4},
						{2,4},
						{3,4} };

static double rollG, pitchG, yawG, yG, zG;		// local vars for angle calculation
#if (MEDIAN_ON == 1)
static double yawFiltWnd[] = {0,0,0,0,0,0,0,0,0};	//for moving median filtering, 9 values
static double pitchFiltWnd[] = {0,0,0,0,0,0,0,0,0};
static double rollFiltWnd[] = {0,0,0,0,0,0,0,0,0};
static const int sortWndSz = sizeof(yawFiltWnd) / sizeof(yawFiltWnd[0]);
#endif

#if (POS_CALC == 1)
static const double Y0 = 0.2;	// TODO: read it from parameter
static const double I = (31);	// TODO: read it from parameter
static const double C = (double)(4e-7*I/2);
// sensor positions in yz-plane clockwise with increasing array index
static const double pi0x[] = {0.02,0.027,0.08,0.11}; 
static const double pi0y[] = {-0.08,0.073,0.25,-0.25};
static const double pi0z[] = {0.235,0.235,0,0};
#endif

//	***	Subscriber callbacks	***
void chatterCallback1(sensor_msgs::MagneticField data)
{
	double x = (double) data.magnetic_field.y;  // original: data.magnetic_field.x
	double y = (double) -data.magnetic_field.x;  // original: +data.magnetic_field.y
	double z = (double) data.magnetic_field.z;
	magnetometer1.x = x;
	magnetometer1.y = y;
	magnetometer1.z = z;
}

void chatterCallback2(sensor_msgs::MagneticField data)
{
	double x = (double) data.magnetic_field.y; // original: data.magnetic_field.x
	double y = (double)  -data.magnetic_field.x; // original: +data.magnetic_field.y
	double z = (double) data.magnetic_field.z;
	magnetometer2.x = x;
	magnetometer2.y = y;
	magnetometer2.z = z;
}

void chatterCallback3(sensor_msgs::MagneticField data)
{
	double x = (double) data.magnetic_field.y;  // original: data.magnetic_field.x
	double y = (double) -data.magnetic_field.x;  // original: +data.magnetic_field.y
	double z = (double) data.magnetic_field.z;
	magnetometer3.x = x;
	magnetometer3.y = y;
	magnetometer3.z = z;
}

void chatterCallback4(sensor_msgs::MagneticField data)
{
	double x = (double) data.magnetic_field.y; // original: data.magnetic_field.x
	double y = (double) -data.magnetic_field.x; // original: +data.magnetic_field.y
	double z = (double) data.magnetic_field.z;
	magnetometer4.x = x;
	magnetometer4.y = y;
	magnetometer4.z = z;
}



static void GetOrientPos()
{
	double yawA = 0;
	double pitchA = 0;
	double rollA = 0;
	int i, k;
	unsigned int v, m;
	static unsigned int cnt = 0;

	++cnt;
	
	for( i = 0 ; i < NUM_OF_SENS; i++ )
	{
		// get sensor combination
		v = sensCombs[i][0]-1;
		m = sensCombs[i][1]-1;
		// calc intermediate yaw and pitch 
		yawA = yawA + (double) atan((ptrSensData[v]->z*ptrSensData[m]->x - ptrSensData[v]->x*ptrSensData[m]->z) / (ptrSensData[v]->y*ptrSensData[m]->z-	ptrSensData[v]->z*ptrSensData[m]->y));
		pitchA = pitchA + (double) atan((ptrSensData[m]->x + ptrSensData[m]->y*tan(yawA)) / (ptrSensData[m]->z*sqrt(tan(yawA)*tan(yawA) + 1)));
	}
	yawA = (double) (yawA / (double)NUM_OF_SENS);
	pitchA = (double) (pitchA / (double)NUM_OF_SENS);
	rollA = 0; // TODO: provided by Gazebo

	// Back transformation for Gazebo convention	
	pitchG = -asin((sin(rollA)*sin(yawA) + cos(rollA)*cos(yawA)*sin(pitchA)));
	rollG = asin((cos(rollA)*sin(pitchA)*sin(yawA) - cos(yawA)*sin(rollA))/cos(pitchG));
	yawG = asin((cos(yawA)*sin(rollA)*sin(pitchA) - cos(rollA)*sin(yawA))/cos(pitchG)); 

#if (MEDIAN_ON == 1)
	double yawArrLc[sortWndSz];
	double pitchArrLc[sortWndSz];
	double rollArrLc[sortWndSz];
	// Apply median filter to the angles
	// shift FIFO and store the new value in FIFO
	for( i = 1; i < MEDIAN_WND_SZ; i++)
	{
		yawFiltWnd[i-1] = yawFiltWnd[i];
		pitchFiltWnd[i-1] = pitchFiltWnd[i];
		rollFiltWnd[i-1] = rollFiltWnd[i];
		// fill temporary array
		yawArrLc[i-1] = yawFiltWnd[i];
		pitchArrLc[i-1] = pitchFiltWnd[i];
		rollArrLc[i-1] = rollFiltWnd[i];
	}
	// store new value in FIFO
	yawFiltWnd[MEDIAN_WND_SZ-1] = yawG;
	pitchFiltWnd[MEDIAN_WND_SZ-1] = pitchG;
	rollFiltWnd[MEDIAN_WND_SZ-1] = rollG;
	// fill temporary array
	yawArrLc[MEDIAN_WND_SZ-1] = yawG;
	pitchArrLc[MEDIAN_WND_SZ-1] = pitchG;
	rollArrLc[MEDIAN_WND_SZ-1] = rollG;
	// sort it in the temporary array
	std::sort((double*)yawArrLc, yawArrLc + sortWndSz);
	std::sort((double*)pitchArrLc, pitchArrLc + sortWndSz);
	std::sort((double*)rollArrLc, rollArrLc + sortWndSz);
	// read the final value
	yawG = yawArrLc[MEDIAN_IDX];
	pitchG = pitchArrLc[MEDIAN_IDX];
	rollG = rollArrLc[MEDIAN_IDX];
#endif

#if (POS_CALC == 1)
	// vars
	double wz = yawA;
        double wy = pitchA;
        double wx = rollA;
	double yLoc[2];
	double zLoc[2];
	double zArr[2];
	long double P,vx,vy,vz,Vy,Vz,a,b,r,fi,yn,yp,g,h,AoN,AoP,VzN,VzP,U0;
	int hlpV1,hlpV2,hlpV3;
	double ys[NUM_OF_SENS][2];
	double zs[NUM_OF_SENS][2];
	double ppiy[NUM_OF_SENS];
	double ppiz[NUM_OF_SENS];
	double ppy[NUM_OF_SENS];
	double ppz[NUM_OF_SENS];
	// for testing only, remove later
	//double yDD[NUM_OF_SENS];
	//double zDD[NUM_OF_SENS];
	
	// get the possible positions of each sensor and determine structure of the original polygon
	for( i = NUM_OF_SENS; --i >= 0; )
	{
		/***************** 1 calculat the possible positions of each sensor *****************/
		// get the magnetic field vector
		vx = ptrSensData[i]->x;
		vy = ptrSensData[i]->y;
		vz = ptrSensData[i]->z;
		// rotate back.....V = Rs.v
		Vy = (cos(wz)*sin(wx)*sin(wy)-cos(wx)*sin(wz))*vx + (cos(wx)*cos(wz)+sin(wx)*sin(wy)*sin(wz))*vy + cos(wy)*sin(wx)*vz;
		Vz = (sin(wx)*sin(wz)+cos(wx)*cos(wz)*sin(wy))*vx + (cos(wx)*sin(wy)*sin(wz)-cos(wz)*sin(wx))*vy + cos(wx)*cos(wy)*vz;
		// signal power
		P = sqrt( vx*vx + vy*vy + vz*vz );
		// Assumtion that Vy is never zero. If Vz is zero, set it to 1e-9 --> in
                // this way always case I) from paper can be used for position calculation.
		hlpV1 = (int)(Vz*1e9);
		if (hlpV1 == 0)	Vz = 1e-9; //1nT
		// calc y-coordinates
		//a = P*P*P*P*Y0*Y0+(2*Vz*Vz-P*P)*C*C;		// P^4*y_0^2+(2*Vz^2-P^2)*C^2;
             	//b = 2*C*C*Vz*sqrt(P*P-Vz*Vz);			// 2*C^2*Vz*sqrt(P^2-Vz^2);
		a = pow(P,4)*pow(Y0,2)+(2*pow(Vz,2)-pow(P,2))*pow(C,2);		// P^4*y_0^2+(2*Vz^2-P^2)*C^2;
             	b = 2*pow(C,2)*Vz*sqrt(pow(P,2)-pow(Vz,2));			// 2*C^2*Vz*sqrt(P^2-Vz^2);
              	r = sqrt(pow(a,2)+pow(b,2));
              	fi = atan2(b,a);
              	yn = (-sqrt(r)*cos(fi/2)+C*Vz)/pow(P,2);
              	yp = (sqrt(r)*cos(fi/2)+C*Vz)/pow(P,2);
		yLoc[0] = yn;
		yLoc[1] = yp;

		// FOR TESTING
		//yDD[i] = yn; // nope
		//yDD[i] = Vy; // nice

		// calc z-coordinate for each y-coordinate
		for( k = 2; --k >= 0; )
		{
			// select the correct inner sign by component matching with Vz
		        g = 2*sqrt((yLoc[k]*yLoc[k]-C*C/P/P)*Y0*Y0+C*C*C*C/P/P/P/P);
		        h = -Y0*Y0-yLoc[k]*yLoc[k]+2*C*C/P/P;
		        AoN = -g+h; AoP = g+h;  // zk^2
			if (AoN > 0 && AoP > 0) // check Bz(yk, zk^2) == Vz
			{
				VzN = C*( (yLoc[k]+Y0)/(AoN+pow(yLoc[k]+Y0,2)) + (yLoc[k]-Y0)/(AoN+pow(yLoc[k]-Y0,2)) );
				VzP = C*( (yLoc[k]+Y0)/(AoP+pow(yLoc[k]+Y0,2)) + (yLoc[k]-Y0)/(AoP+pow(yLoc[k]-Y0,2)) );
				zArr[0] = -sqrt(AoN); // assuming UAV location in z<0
				zArr[1] = -sqrt(AoP); // assuming UAV location in z<0
				if (Vy < 0) //TODO: Cmp error???....UAV is in z>0, flip sign according to the navigation constraints in chapter 3.3 of the paper
				{
					zArr[0] = -zArr[0];
					zArr[1] = -zArr[1];
				}
				hlpV1 = (int)((VzN-Vz)*1e9);
				hlpV3 = abs(hlpV1);
				hlpV1 = hlpV3;

				hlpV2 = (int)((VzP-Vz)*1e9);
				hlpV3 = abs(hlpV2);
				hlpV2 = hlpV3;

				if (hlpV1 < hlpV2)	zLoc[k] = zArr[0];
				else			zLoc[k] = zArr[1];
			}
			else if (AoN < 0 && AoP > 0)	// sqrt(AoN) is complex
			{
				zLoc[k] = -sqrt(AoP);
				if (Vy < 0) //UAV is in z>0 //TODO: Cmp error???
					zLoc[k] = -zLoc[k];
			}
			else if (AoN > 0 && AoP < 0)	// sqrt(AoP) is complex
			{
				zLoc[k] = -sqrt(AoN);
				if (Vy < 0) //UAV is in z>0 //TODO: Cmp error???
					zLoc[k] = -zLoc[k];
			}
			else if (AoN < 0 && AoP < 0)	// this case should never happen respectively is impossible.
			{
				// For safety put positive large (impossible) value
				zLoc[k] = 100;
			}

			// Store the position data of the current sensor i. Each sensor has two possible positions.
			ys[i][k] = yLoc[k];
			zs[i][k] = zLoc[k];

			// FOR TESTING
			//if (k == 0) // use the z corresponding to yn
			//{
			//	zDD[i] = zLoc[k];
			//}

		} //for loop k=2

		// prepare perimeter calculation of rotated original polygon (ROP) for step 2
		// calculate rotated current node [ppiy, ppiz] of the original polygon
		vx = pi0x[i]; vy = pi0y[i]; vz = pi0z[i];
		ppiy[i] = (cos(wz)*sin(wx)*sin(wy)-cos(wx)*sin(wz))*vx + (cos(wx)*cos(wz)+sin(wx)*sin(wy)*sin(wz))*vy + cos(wy)*sin(wx)*vz;
		ppiz[i] = (sin(wx)*sin(wz)+cos(wx)*cos(wz)*sin(wy))*vx + (cos(wx)*sin(wy)*sin(wz)-cos(wz)*sin(wx))*vy + cos(wx)*cos(wy)*vz;
		
	}// for loop i=NUM_OF_SENS

# if (POLYG_DETECT == 1)
	// shift upper left corner of the ROP into the centre of the global frame
	for( i = NUM_OF_SENS; --i >= 0; )
	{
		ppy[i] = ppiy[i] - ppiy[0];
		ppz[i] = ppiz[i] - ppiz[0];
	}


	/***************** 2 calculate perimeter of ROP (this will be the reference) *****************/
	U0 = 0;
	for( i = NUM_OF_SENS; --i > 0; )
	{
		U0 = U0 + sqrt( pow(ppy[i-1] - ppy[i],2) + pow(ppz[i-1] - ppz[i],2) );
	}
	U0 = U0 + sqrt( pow(ppy[0] - ppy[NUM_OF_SENS-1],2) + pow(ppz[0] - ppz[NUM_OF_SENS-1],2) );


	/***************** 3 compare all polygons with the original one and select the most similar *****************/
	unsigned int n = 0;	// count variable for numbering the particular polygons
	unsigned int binSeq[NUM_OF_POLYG][4];	// this variable will hold the node combination of the found polygon
	double ppLcy[NUM_OF_SENS];
	double ppLcz[NUM_OF_SENS];
	int sk[NUM_OF_SENS];
	double uks, pks;
	double Uk[NUM_OF_POLYG];
	unsigned int Pk[NUM_OF_POLYG];

	// iterate over all possible 2^N poylgons set up by N sensors. Basis 2 because for every sensor there exist
	// 2 possible locations in the yz-plane it can be.
	for( sk[0] = 1; sk[0] >= 0; sk[0]--)
	{
		for( sk[1] = 1; sk[1] >= 0; sk[1]--)
		{
			for( sk[2] = 1; sk[2] >= 0; sk[2]--)
			{
#if (NUM_OF_SENS == 4)
				for( sk[3] = 1; sk[3] >= 0; sk[3]--)
#else
				for( sk[3] = 0; sk[3] >= 0; sk[3]--)
#endif
				{
					uks = 0;
					pks = 0;

					for( i = NUM_OF_SENS; --i >= 0; )
					{
						// shift upper left corner of the current polygon into the centre of the global frame
						ppLcy[i] = ys[i][ sk[i] ] - ys[0][ sk[0] ];
						ppLcz[i] = zs[i][ sk[i] ] - zs[0][ sk[0] ];

						// calculate square error of the current node (deviation from original node)
						uks = uks + pow( ppy[i] - ppLcy[i],2 ) + pow( ppz[i] - ppLcz[i],2 );

						// store node combination of the current polygon with number n. It is a unique fingerprint among all polygons
						binSeq[n][i] = sk[i];
					}

					// store sum of squared errors for the current polygon
					Uk[n] = uks;
			
					// calculate perimeter of the current polygon and store it
					for( i = NUM_OF_SENS; --i > 0; )
					{
						pks = pks + sqrt( pow(ppLcy[i-1] - ppLcy[i],2) + pow(ppLcz[i-1] - ppLcz[i],2) );
					}
					pks = pks + sqrt( pow(ppLcy[0] - ppLcy[NUM_OF_SENS-1],2) + pow(ppLcz[0] - ppLcz[NUM_OF_SENS-1],2) );
					Pk[n] = (unsigned int)(abs( (double)(pks - U0) )*1000);

					// increment the polygon number for the next polygon
					n = n + 1;
				}
			}
		}
	} // for loop sk[0] = 1

	// find the two smallest Uk
	unsigned int il[2];
	unsigned int prm[2];
	unsigned int hlpV, currUk;

	for( i = 2; --i >= 0; )
	{
		// assume first element having smallest Uk
		hlpV = (unsigned int)(Uk[NUM_OF_POLYG-1]*1e5);
		// store index and perimeter of first element
		il[i] = NUM_OF_POLYG-1;
		prm[i] = Pk[NUM_OF_POLYG-1];

		for( k = NUM_OF_POLYG-1; --k >= 0; )
		{
			// get next element 
			currUk = (unsigned int)(Uk[k]*1e5);

			// compare with last found smallest element
			if ( currUk < hlpV )
			{
				// save new found smallest element
				hlpV = currUk;
				// store its index and perimeter
				il[i] = k;
				prm[i] = Pk[k];
			}
		}

		// mark found element in the Uk array so that it is not selected again in the second turn
		Uk[ il[i] ] = 1e3;
	} // for loop n=2

	// now in the last step select from the two found elements the one with the smaller perimeter and store its index.
	// The index simultanously represents the polygon number, by which its nodes can be identified using variables binSeq, ys and zs.
	unsigned int finalPolygNum;
	if ( prm[0] < prm[1] )
		finalPolygNum = il[0];
	else
		finalPolygNum = il[1];


	/***************** 4 calculate the UAV position *****************/
	// iterate over all nodes of the found most similar polygon and calculate from each node the UAV position.
	// In the final step take the average of the coordinates as final UAV position
	double ny,nz,tmpCoord;
	double yDLoc = 0;
	double zDLoc = 0;
	unsigned int polyIdx;
	for( i = NUM_OF_SENS; --i >= 0; ) // the number of sensors corresponds to the number of nodes per polygon
	{
		// get node 
		k = binSeq[finalPolygNum][i]; // get node index. (Remember: for each sensor there exist two nodes respectively positions)
		ny = ys[i][k];
		nz = zs[i][k];
		// calculate UAV position
		// y-coordinate
		tmpCoord = ny - ((cos(wz)*sin(wx)*sin(wy)-cos(wx)*sin(wz))*pi0x[i] + (cos(wx)*cos(wz)+sin(wx)*sin(wy)*sin(wz))*pi0y[i] + cos(wy)*sin(wx)*pi0z[i]);
		yDLoc = yDLoc + tmpCoord;
		// z-coordinate
		tmpCoord = nz - ((sin(wx)*sin(wz)+cos(wx)*cos(wz)*sin(wy))*pi0x[i] + (cos(wx)*sin(wy)*sin(wz)-cos(wz)*sin(wx))*pi0y[i] + cos(wx)*cos(wy)*pi0z[i]);
		zDLoc = zDLoc + tmpCoord;
	}
	// average the coordinates
	yG = (double)(yDLoc/(double)NUM_OF_SENS);
	zG = (double)(zDLoc/(double)NUM_OF_SENS);
# endif // if(POLYG_DETECT == 1)
#endif
}




//	***	Function for calculating orientation of UAV	***
static void GetOrientation()
{
	double rollA, pitchA, yawA;

	// Equations for calculating input of back transformation

	double alpha = (double) atan((magnetometer1.z*magnetometer2.x - magnetometer1.x*magnetometer2.z) / 
			(magnetometer1.y*magnetometer2.z-magnetometer1.z*magnetometer2.y));
	
	 double beta = (double) atan((magnetometer2.x + magnetometer2.y*tan(alpha)) / 
			(magnetometer2.z*sqrt(tan(alpha)*tan(alpha) + 1)));

	double gamma = (double) atan((magnetometer3.y*sin(beta)*sin(alpha) + magnetometer3.x*sin(beta)*cos(alpha) + magnetometer3.z*cos(beta)) / 
			(magnetometer3.y*cos(alpha) - magnetometer3.x*sin(alpha)));

	rollA = gamma;
	pitchA = beta;
	yawA = alpha;

	// Back transformation for Gazebo convention	
	pitchG    = -asin((sin(rollA)*sin(yawA) + cos(rollA)*cos(yawA)*sin(pitchA)));

	rollG = asin((cos(rollA)*sin(pitchA)*sin(yawA) - cos(yawA)*sin(rollA))/cos(pitchG));

	yawG = asin((cos(yawA)*sin(rollA)*sin(pitchA) - cos(rollA)*sin(yawA))/cos(pitchG)); 
	

	/*
	ROS_INFO("-----Orientation------\n"); //Radians
	ROS_INFO("alfa: %.10lf \n", alfa);
	ROS_INFO("beta: %.10lf \n", beta);
	ROS_INFO("gama: %.10lf \n", gama);
	ROS_INFO("-----------------------\n");
	*/
	
}

// 	*** 	Node main function	***
int main (int argc, char** argv){
	// Node initialization
	ros::init(argc, argv, "orientation");
    	ros::NodeHandle n;
    	ros::NodeHandle n_ns("~");

	
	// Topic subscribers
	ros::Subscriber magnetic_field_sub1 = n.subscribe<sensor_msgs::MagneticField>("/imu_magnetic0", 1000, chatterCallback1); //imu_magnetic
	ros::Subscriber magnetic_field_sub2 = n.subscribe<sensor_msgs::MagneticField>("/imu_magnetic1", 1000, chatterCallback2);
	ros::Subscriber magnetic_field_sub3 = n.subscribe<sensor_msgs::MagneticField>("/imu_magnetic2", 1000, chatterCallback3);
	ros::Subscriber magnetic_field_sub4 = n.subscribe<sensor_msgs::MagneticField>("/imu_magnetic3", 1000, chatterCallback4);

	// TODO: ros::Subscriber magnetic_field_sub3 = n.subscribe("/firefly/ground_truth/odometry", 1000, &tf_sub_pub::callback, this);
	
	// Topic publishers
	ros::Publisher chatter_pub1 = n.advertise<std_msgs::Float64>("rollD", 20);
	ros::Publisher chatter_pub2 = n.advertise<std_msgs::Float64>("pitchD", 20);
	ros::Publisher chatter_pub3 = n.advertise<std_msgs::Float64>("yawD", 20);
	ros::Publisher chatter_pub4 = n.advertise<std_msgs::Float64>("yD", 20);
	ros::Publisher chatter_pub5 = n.advertise<std_msgs::Float64>("zD", 20);
	
	ros::Rate loop_rate (20);
	while (ros::ok ())
  	{
		ros::spinOnce ();
		try 
		{
	    		//GetOrientation();
			GetOrientPos();
			
			std_msgs::Float64 roll, pitch, yaw, y, z;
			roll.data = rollG;
			pitch.data = pitchG;
			yaw.data = yawG;
			y.data = yG;
			z.data = zG;
			chatter_pub1.publish(roll);
			chatter_pub2.publish(pitch);
			chatter_pub3.publish(yaw);
			chatter_pub4.publish(y);
			chatter_pub5.publish(z);
		}
        	catch (tf::TransformException ex){
          		ROS_ERROR("%s",ex.what());
        	}
	
    		
    		loop_rate.sleep ();
  	}

	return 0;


}


