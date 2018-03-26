#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include <string>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int8.h>

using namespace ros;
using namespace std;

double pi = 3.14;

vector<double> waypoint_x;
vector<double> waypoint_y;
vector<bool> waypoint_passed;

static double referenceLat, referenceLong;

geometry_msgs::Point toXY(double longitude, double latitude)
{
   double x = (longitude - referenceLong) * (40075000.0 / (2.0 * pi)) * cos(referenceLat);
   double y = (latitude - referenceLat) * (40007000.0 / (2.0 * pi));
   geometry_msgs::Point p;
   p.x = x;
   p.y = y;
   return p;
}

void referenceLong_callback(const std_msgs::Float64::ConstPtr& longref)
{
   referenceLong = longref->data;
}

void referenceLat_callback(const std_msgs::Float64::ConstPtr& latref)
{
   referenceLat = latref->data;
}

double distance(double x1, double y1, double x2, double y2)
{
   return sqrt( pow((x1 - x2), 2) + pow((y1 - y2), 2) );
}

void readWaypointsFile()
{
	std::ifstream infile("/home/sarthak/Desktop/Aurora/waypoints.txt");
	double lati, longi;
	while (infile >> longi >> lati)
	{
		geometry_msgs::Point pt = toXY(longi,lati);
		double x = pt.x;
		double y = pt.y;
		waypoint_x.push_back(x);
		waypoint_y.push_back(y);
	}
}

void main_callback(const std_msgs::Int8::ConstPtr& bol)
{
	if (bol->data == 0)
	{

	}
	else if(bol->data == 1)
	{
		
	}
}

int main(int argc, char** argv)
{
	init(argc, argv, "getPath");
	NodeHandle nh;
	Rate loop_rate(10);

	ros::Subscriber referenceLong_Sub = nh.subscribe("referenceLong",10,referenceLong_callback);
	ros::Subscriber referenceLat_Sub = nh.subscribe("referenceLat",10,referenceLat_callback);
	ros::Subscriber reached_sub = nh.subscribe("reached",10,main_callback);

	readWaypointsFile();

	int index = 0;
	while(ros::ok())
	{

	}
}