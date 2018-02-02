#include <ros/ros.h>
#include <vector>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <stdio.h>
#include <geometry_msgs/Point.h>

using namespace ros;
using namespace std;

geometry_msgs::Point dummy_current;

void send_current()
{
	double x = rand() % 8;
	double y = rand() % 8;
	dummy_current.x = x;
	dummy_current.y = y;
	cout << "sending : " << x << " " << y << endl;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "dummyXY");
	ros::NodeHandle nh;
	Rate loop_rate(10);

	ros::Publisher dummyXY_pub = nh.advertise<geometry_msgs::Point>("dummyXY",10);
	ros::Publisher dummy_refLong_pub = nh.advertise<std_msgs::Float64>("dummy_lat",10);
	ros::Publisher dummy_refLat_pub = nh.advertise<std_msgs::Float64>("dummy_long",10);

	std_msgs::Float64 dummy_long;
	std_msgs::Float64 dummy_lat;

	dummy_long.data = 0;
	dummy_lat.data = 0;

	while(ros::ok())
	{
		send_current();
		dummy_refLat_pub.publish(dummy_lat);
		dummy_refLong_pub.publish(dummy_long);
		dummyXY_pub.publish(dummy_current);
	}

}