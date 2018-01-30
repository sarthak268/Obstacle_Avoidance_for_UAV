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

double current_x = 0;
double current_y = 0;

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

double distance(double x1, double y1, double x2, double y2)
{
	return (pow( pow((x1 - x2),2) + pow((y1 - y2),2) ,0.5));
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
	std::ifstream infile("/home/sarthak/Desktop/Aurora/waypoints.txt");
	cout << "opened waypoints" << endl;
	double x, y;
	while (infile >> x >> y)
	{
		waypoints.header.frame_id = "/my_frame";
		waypoints.header.stamp = ros::Time::now();
		waypoints.ns = "rviz_plot";
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
		cout << "waypoints : \n" <<  wp << endl;
	}
}

void readObstacleFile()
{	
	std::ifstream infile("/home/sarthak/Desktop/Aurora/obstacles.txt");
	cout << "opened obstacles" << endl;
	double x, y, r;
	double min_dis = 0;
	double x_min = 0;
	double y_min = 0;
	double r_min = 0;
	bool first = true;
	while (infile >> x >> y >> r)
	{
		if (first == true)
		{
			min_dis = distance(current_x, current_y, x, y);
			x_min = x;
			y_min = y;
			r_min = r;
			first = false;
		}	
		else
		{
			double dist = distance(current_x, current_y, x, y);
			if (dist < min_dis)
			{
				min_dis = dist;
				x_min = x;
				y_min = y;
				r_min = r;
			}
		}
	}
	obstacles.header.frame_id = "/my_frame";
	obstacles.header.stamp = ros::Time::now();
	obstacles.ns = "rviz_plot";
	obstacles.action = visualization_msgs::Marker::ADD;
	obstacles.id = 2;
	obstacles.type = visualization_msgs::Marker::CYLINDER;
	obstacles.pose.position.x = x_min;
	obstacles.pose.position.y = y_min;
	obstacles.pose.position.z = 0;
	obstacles.pose.orientation.x = 0;
	obstacles.pose.orientation.y = 0;
	obstacles.pose.orientation.z = 0;
	obstacles.pose.orientation.w = 1;
	obstacles.scale.x = 2*r_min;
	obstacles.scale.y = 2*r_min;
	obstacles.scale.z = 2;
	obstacles.color.r = 1;
	obstacles.color.a = 1;
	cout << "obstacles : \n" << x_min << " " << y_min << " "<< r_min << endl; 
}

void markCurrent(const geometry_msgs::Point::ConstPtr& pt)
{
	//cout << "receiving current" << endl;
	double x = pt->x;
	double y = pt->y;

	current_x = x;
	current_y = y;

	points.header.frame_id = "/map";
	points.header.stamp = ros::Time::now();
	points.ns = "rviz_plot";
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
	//cout << "added" << p << endl;
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

	//readObstacleFile();
	readWaypointsFile();
	//cout << waypoints <<  endl;
	//cout << obstacles << endl;

	while (ros::ok())
	{
		readObstacleFile();
		waypoints_vis.publish(waypoints);
		obstacle_vis.publish(obstacles);

		currentXY_vis.publish(points);
		spinOnce();
	}
}