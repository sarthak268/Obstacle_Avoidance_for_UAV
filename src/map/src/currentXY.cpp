#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <vector>

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

void global_position_callback(const sensor_msgs::NavSatFix::ConstPtr globalpos)
{
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

	while (ros::ok())
	{
		spin();
	}

}