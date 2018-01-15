#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <stdio.h>

using namespace ros;
using namespace std;

geometry_msgs::Point current;
double pi = 3.14;
bool first_time;
double reference_long_rad;
double reference_lat_rad;

double toRad(double deg)
{
	return (deg*pi)/180;
}

double toDeg(double rad)
{
	return (rad*180)/pi;
}

geometry_msgs::Point toXY(double longitude, double latitude)
{
	double x = (longitude - reference_long_rad) * (40075000.0 / (2.0 * pi)) * cos(reference_lat_rad);
	double y = (latitude - reference_lat_rad) * (40007000.0 / (2.0 * pi));
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	return p;
}

void global_position_callback(const sensor_msgs::NavSatFix::ConstPtr& globalpos) // altitude //
{
	cout << "receiving mavros data" << endl;
	double lat = globalpos->latitude;
	double longi = globalpos->longitude;
	if (first_time)
	{
		reference_lat_rad = toRad(lat);
		reference_long_rad = toRad(longi);
		first_time = false;
	}
	else
	{
		double long_rad = toRad(longi);
		double lat_rad = toRad(lat);
		current = toXY(long_rad,lat_rad);
 	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "currentXY");
	ros::NodeHandle nh;
	Rate loop_rate(10);
	
	ros::Subscriber global_position_sub = nh.subscribe("/mavros/global_position/global",10,global_position_callback); 
	ros::Publisher currentXY_pub = nh.advertise<geometry_msgs::Point>("currentXY",10);
	ros::Publisher referenceLong_pub = nh.advertise<std_msgs::Float64>("referenceLong",10);
	ros::Publisher referenceLat_pub = nh.advertise<std_msgs::Float64>("referenceLat",10);

	std_msgs::Float64 reference_lat_ra;
	reference_lat_ra.data = reference_lat_rad; 
	referenceLat_pub.publish(reference_lat_ra);

	std_msgs::Float64 reference_long_ra;
	reference_long_ra.data = reference_long_rad; 
	referenceLong_pub.publish(reference_long_ra);

	while (ros::ok())
	{
		currentXY_pub.publish(current);
		spin();
	}

}