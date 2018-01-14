/*
0 - unknown(white)
1 - current(blue)
2 - waypoints(green)
3 - obstacles(red)s
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <math.h>
#include <visualization_msgs/Marker.h>

using namespace ros;
using namespace std;

double referenceLat;
double referenceLong;
double pi = 3.14;
visualization_msgs::Marker points;

int my_map[101][101] = {0};

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
	double x = (longitude - referenceLong) * (40075000.0 / (2.0 * pi)) * cos(referenceLat);
	double y = (latitude - referenceLat) * (40007000.0 / (2.0 * pi));
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	return p;
}

void readWaypointsFile()
{

}

void readObstacleFile()
{

}

void markCurrent(const geometry_msgs::Point::ConstPtr& pt)
{
	double x = pt->x;
	double y = pt->y;
	int roundx = int(x);
	int roundy = int(y);
	my_map[51+roundx][51+roundy] = 1;

	points.header.frame_id = "/currentXY";
	points.header.stamp = ros::Time::now();
	points.ns = "currentXY points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.5;
	points.scale.y = 0.5;
	points.color.b = 1;
	points.color.a = 1;
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	points.points.push_back(p);
}

void referenceLong_callback(const std_msgs::Float64::ConstPtr& longref)
{
	referenceLong = longref->data;
}

void referenceLat_callback(const std_msgs::Float64::ConstPtr& latref)
{
	referenceLat = latref->data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "markPoints");
	NodeHandle nh;
	Rate loop_rate(10);
	
	ros::Subscriber currentXY_sub = nh.subscribe("currentXY",10,markCurrent);
	ros::Subscriber referenceLong_Sub = nh.subscribe("referenceLong",10,referenceLong_callback);
	ros::Subscriber referenceLat_Sub = nh.subscribe("referenceLat",10,referenceLat_callback);
	ros::Publisher currentXY_vis = nh.advertise<visualization_msgs::Marker>("currentXY_vis",10);

	while (ros::ok())
	{	
		spin();
	}
}