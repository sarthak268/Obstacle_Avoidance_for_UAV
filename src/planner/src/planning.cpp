#include <math.h>
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point.h>

using namespace ros;
using namespace std;

static double current_x;
static double current_y;

static double referenceLat;
static double referenceLong;

void getCurrent(const geometry_msgs::Point::ConstPtr& p)
{
   current_x = p->x;
   current_y = p->y;
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

int main(int argc, char ** argv)
{
   ros::init(argc, argv, "planner");
   NodeHandle nh;
   Rate loop_rate(10);

   ros::Subscriber currentXY_sub = nh.subscibe("currentXY",10,getCurrent);
   ros::Subscriber referenceLong_Sub = nh.subscribe("referenceLong",10,referenceLong_callback);
   ros::Subscriber referenceLat_Sub = nh.subscribe("referenceLat",10,referenceLat_callback);
   
}