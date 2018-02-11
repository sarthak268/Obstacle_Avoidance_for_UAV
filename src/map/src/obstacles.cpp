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

visualization_msgs::Marker obstacles;
double referenceLat;
double referenceLong;


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
	ros::init (argc, argv, "obstacles");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);

	ros::Subscriber referenceLong_Sub = nh.subscribe("referenceLong",10,referenceLong_callback);
	ros::Subscriber referenceLat_Sub = nh.subscribe("referenceLat",10,referenceLat_callback);

	ros::Publisher obs_pub = nh.advertise<visualization_msgs::Marker>("obstacles",10);

	while (ros::ok())
	{
		readObstacleFile();
		obs_pub.publish(obstacles);
		spinOnce();
	}

}