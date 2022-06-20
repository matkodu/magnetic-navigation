/***
 * Modbus master for communicating over serial port
 */
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/MagneticField.h>
#define DEFAULT_SERIAL_PORT "/dev/ttyACM0"
#define DEFAULT_BAUD_RATE 460800


serial::Serial ser;
uint8_t data1[500];
double received_data[13];
double GetDoubleFromString(std::string data, int start, int end)
{
	double value=0;
	for (int i=start;i<=end;i++)
	{	
		value = value*10 + data[i]-48;
	}
	if (value>32767)
	{
		value=-(65536-value);
	}
	return value;
}
double Parse(std::string data,int count)
{
	int start=0;
	int data_count=0;
	for (int i=0;i<count;i++)
	{
		if (data[i]==' ')
		{
			received_data[data_count]=GetDoubleFromString(data,start,i-1);
			start=i+1;
			data_count=data_count+1;
			if (data_count>=13) break;
		}
//		printf("%d ",data[i] );
	}
	if (data_count<13)
	{
		received_data[data_count]=GetDoubleFromString(data,start,count-1);
		data_count=data_count+1;
	}
}

int main (int argc, char** argv){
    ros::init(argc, argv, "magnetic_sensors_arduino");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    std::string port, topic;
    int baudrate,offset;
    nh_ns.param("port", port, (std::string) DEFAULT_SERIAL_PORT); 
    nh_ns.param("baudrate", baudrate, DEFAULT_BAUD_RATE);
    nh_ns.param("offset", offset, 0);
    nh_ns.param("topic", topic , (std::string) "/magnetic");

    ros::Publisher data_pub = nh.advertise<std_msgs::Float32>(topic, 1000);

   //open port
    try
    {
        ser.setPort(port);
        ser.setBaudrate(baudrate);
        serial::Timeout to = serial::Timeout::simpleTimeout(30);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        return -1;
    }

    ros::Rate loop_rate(1000);
    bool first = true;
    int start_stamp=0;
	ros::Publisher magnetic_field_pub1 = nh.advertise<sensor_msgs::MagneticField>(topic+"1", 1000);
	ros::Publisher magnetic_field_pub2 = nh.advertise<sensor_msgs::MagneticField>(topic+"2", 1000);
	ros::Publisher magnetic_field_pub3 = nh.advertise<sensor_msgs::MagneticField>(topic+"3", 1000);
	ros::Publisher magnetic_field_pub4 = nh.advertise<sensor_msgs::MagneticField>(topic+"4", 1000);
	ros::Time start_time;
    while(ros::ok()){

        ros::spinOnce();
        std::string data2;
        int count = ser.readline(data2,500,(std::string)"\n");
        if (count>5)
        {
        	double number=Parse(data2,count);
        	if (first==true)
        	{
        		start_time = ros::Time::now();
        		start_stamp=(int) received_data[0];
        		first=false;
        	}
        	std_msgs::Float32 pub_data;
        	pub_data.data = number - offset;
        	sensor_msgs::MagneticField pub_data1;
        	sensor_msgs::MagneticField pub_data2;
        	sensor_msgs::MagneticField pub_data3;
        	sensor_msgs::MagneticField pub_data4;

        	pub_data1.header.stamp = start_time + ros::Duration((received_data[0]-start_stamp)/1000);
        	pub_data2.header.stamp = start_time + ros::Duration((received_data[0]-start_stamp)/1000);
        	pub_data3.header.stamp = start_time + ros::Duration((received_data[0]-start_stamp)/1000);
        	pub_data4.header.stamp = start_time + ros::Duration((received_data[0]-start_stamp)/1000);

        	pub_data1.magnetic_field.x=received_data[1];
        	pub_data1.magnetic_field.y=received_data[2];
        	pub_data1.magnetic_field.z=received_data[3];

        	pub_data2.magnetic_field.x=received_data[4];
        	pub_data2.magnetic_field.y=received_data[5];
        	pub_data2.magnetic_field.z=received_data[6];

        	pub_data3.magnetic_field.x=received_data[7];
        	pub_data3.magnetic_field.y=received_data[8];
        	pub_data3.magnetic_field.z=received_data[9];

        	pub_data4.magnetic_field.x=received_data[10];
        	pub_data4.magnetic_field.y=received_data[11];
        	pub_data4.magnetic_field.z=received_data[12];


        	data_pub.publish(pub_data);
        	magnetic_field_pub1.publish(pub_data1);
        	magnetic_field_pub2.publish(pub_data2);
        	magnetic_field_pub3.publish(pub_data3);
        	magnetic_field_pub4.publish(pub_data4);

        }
        loop_rate.sleep();

    }
}

