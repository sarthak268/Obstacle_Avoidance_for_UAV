#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace ros;
using namespace std;

visualization_msgs::Marker waypoints;
double referenceLat;
double referenceLong;

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

int main (int argc, char** argv)
{
	ros::init (argc, argv, "waypoints");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	ros::Subscriber referenceLong_Sub = nh.subscribe("referenceLong",10,referenceLong_callback);
	ros::Subscriber referenceLat_Sub = nh.subscribe("referenceLat",10,referenceLat_callback);

	ros::Publisher wp_pub = nh.advertise<visualization_msgs::Marker>("waypoints",10);
	readWaypointsFile();

	while (ros::ok())
	{
		wp_pub.publish(waypoints);
		spinOnce();
	}

}