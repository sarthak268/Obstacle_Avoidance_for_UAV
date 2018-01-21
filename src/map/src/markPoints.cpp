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
#include <stdio.h>
#include <math.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace ros;
using namespace std;

double referenceLat;
double referenceLong;
double pi = 3.14;
visualization_msgs::Marker points;
visualization_msgs::Marker obstacles;
visualization_msgs::Marker waypoints;

double toRad(double deg)
{
	return (deg*pi)/180;
}

double toDeg(double rad)
{
	return (rad*180)/pi;
}

double ft2m(double ft)
{
	return (0.3084*ft);
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
	std::ifstream infile("waypoints.txt");
	double x, y;
	while (infile >> x >> y)
	{
		waypoints.header.frame_id = "/waypoints";
		waypoints.header.stamp = ros::Time::now();
		waypoints.ns = "waypoints";
		waypoints.action = visualization_msgs::Marker::ADD;
		waypoints.pose.orientation.w = 1;
		waypoints.id = 1;
		waypoints.type = visualization_msgs::Marker::POINTS;
		waypoints.scale.x = 0.1;
		waypoints.scale.y = 0.1;
		waypoints.color.g = 1;
		waypoints.color.a = 1;
		geometry_msgs::Point wp;
		wp.x = x;
		wp.y = y;
		waypoints.points.push_back(wp);
	}
}

void readObstacleFile()
{	
	std::ifstream infile("obstacle.txt");
	double x, y, r;
	while (infile >> x >> y >> r)
	{
		obstacles.header.frame_id = "/obstacles";
		obstacles.header.stamp = ros::Time::now();
		obstacles.ns = "obstacle points";
		obstacles.action = visualization_msgs::Marker::ADD;
		obstacles.id = 2;
		obstacles.type = visualization_msgs::Marker::CYLINDER;
		obstacles.pose.position.x = x;
		obstacles.pose.position.y = y;
		obstacles.pose.position.z = 0;
		obstacles.pose.orientation.x = 0;
		obstacles.pose.orientation.y = 0;
		obstacles.pose.orientation.z = 0;
		obstacles.pose.orientation.w = 1;
		obstacles.scale.x = 2*r;
		obstacles.scale.y = 2*r;
		obstacles.scale.z = 10;
		obstacles.color.r = 1;
		obstacles.color.a = 1;
	}
}

void markCurrent(const geometry_msgs::Point::ConstPtr& pt)
{
	cout << "receiving current" << endl;
	double x = pt->x;
	double y = pt->y;

	points.header.frame_id = "/currentXY";
	points.header.stamp = ros::Time::now();
	points.ns = "currentXY points";
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 0;
	points.id = 0;
	points.type = visualization_msgs::Marker::POINTS;
	points.scale.x = 0.1;
	points.scale.y = 0.1;
	points.color.b = 1;
	points.color.a = 1;
	geometry_msgs::Point p;
	p.x = x;
	p.y = y;
	points.points.push_back(p);
}

void referenceLong_callback(const std_msgs::Float64::ConstPtr& longref)
{
	cout << "receiving reference Long" << endl;
	referenceLong = longref->data;
}

void referenceLat_callback(const std_msgs::Float64::ConstPtr& latref)
{
	cout << "receiving reference Lat" << endl;
	referenceLat = latref->data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "markPoints");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	
	ros::Subscriber currentXY_sub = nh.subscribe("currentXY",10,markCurrent);
	ros::Subscriber referenceLong_Sub = nh.subscribe("referenceLong",10,referenceLong_callback);
	ros::Subscriber referenceLat_Sub = nh.subscribe("referenceLat",10,referenceLat_callback);
	
	ros::Publisher currentXY_vis = nh.advertise<visualization_msgs::Marker>("currentXY_vis",10);
	ros::Publisher obstacle_vis = nh.advertise<visualization_msgs::Marker>("obstacle_vis",10);
	ros::Publisher waypoints_vis = nh.advertise<visualization_msgs::Marker>("waypoints_vis",10);

	while (ros::ok())
	{
		currentXY_vis.publish(points);
		obstacle_vis.publish(obstacles);
		waypoints_vis.publish(waypoints);
		spinOnce();
	}
}