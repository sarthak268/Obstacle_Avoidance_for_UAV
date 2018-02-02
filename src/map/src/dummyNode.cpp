#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <stdio.h>

using namespace ros;
using namespace std;

sensor_msgs::NavSatFix msg;

void create()
{
	msg.latitude = rand() % 8;
	msg.longitude = rand() % 8;
	ros::Duration(2).sleep();
	//cout << "sending msg" << endl;
}

int main(int argc, char** argv)
{
	ros::init (argc, argv, "dummy");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	ros::Publisher pub = nh.advertise<sensor_msgs::NavSatFix>("navsatfix",10);

	while (ros::ok())
	{
		create();
		cout << msg << endl;
		pub.publish(msg);
	}
}