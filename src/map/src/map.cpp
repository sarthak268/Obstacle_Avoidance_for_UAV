#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>

using namespace ros;
using namespace std;

geometry_msgs::Point current;
geometry_msgs::Point previous;
double pi = 3.14;
bool first_time;
geometry_msgs::Point first_waypoint;
double pre_long;
double pre_lat;

double toRad(double deg)
{
	return (deg*pi)/180;
}

double toDeg(double rad)
{
	return (rad*180)/pi;
}

geometry_msgs::Point haversine(double latitude1, double longitude1, double latitude2, double longitude2, geometry_msgs::Point previous)
{
	double r = 6372.8;
	double previous_x = previous.x;
	double previous_y = previous.y;
	double dlat = latitude2 - latitude1;
	double dlong = longitude2 - longitude1;
	double a = sin(dlat/2)*(sin(dlat/2) + cos(latitude1)*cos(latitude2) + sin(dlong/2)*sin(dlong/2) ); 
	double c = 2 * atan2(sqrt(a),sqrt(1-a));
	double d = r * c * 1000;
	double y = sin(dlong) * cos(latitude2);
	double x = (-1)*(sin(latitude1)*cos(latitude2)*cos(dlong)-cos(latitude1)*sin(latitude2));
	double brng = fmod(atan2(y,x) + 2*pi , 2*pi);
	double previous_x1 = previous_x + (d*cos(brng));
	double previous_y1 = previous_y + (d*cos(brng));
	
	geometry_msgs::Point previous1;
	previous1.x = previous_y1;
	previous1.y = previous_y1;
	return previous1;
}

geometry_msgs::Point getXY(double latitude,double longitude,double latitude_pre,double longitude_pre, geometry_msgs::Point previous)
{
	double previous_x = previous.x;
	double previous_y = previous.y;

	double latitude_rad = toRad(latitude);
	double longitude_rad = toRad(longitude);
	double latitude_pre_rad = toRad(latitude_pre);
	double longitude_pre_rad = toRad(longitude_pre);

	geometry_msgs::Point p;

	if (latitude == latitude_pre)
	{
		p.x = 0;
		p.y = 0;
	}
	else 
	{
		p = haversine(latitude_pre,longitude_pre,latitude_rad,longitude_rad,previous);
	}
	return p;
}

void global_position_callback(const sensor_msgs::NavSatFix::ConstPtr& globalpos)
{
	double lat = globalpos->latitude;
	double longi = globalpos->longitude;

	if (first_time)
	{
		cout << "Origin" << longi << " , " << lat << endl;
		current.x = 0;
		current.y = 0;
		first_time = false;
	}
	else
	{
		previous = current;
		current = getXY(lat,longi,pre_lat,pre_long,current);
	}
	pre_lat = lat;
	pre_long = longi;

}

// void imu_callback(const sensor_msgs::Imu::ConstPtr& imudata)
// {
// 	geometry_msgs::Quaternion orientation = imudata->orientation;
// 	tf::Matrix3x3 m(orientation);

// 	double roll, pitch, yaw;
// 	m.getRPY(roll,pitch,yaw);
// }

int main(int argc, char** argv)
{
	ros::init(argc, argv, "map");
	ros::NodeHandle nh;
	Rate loop_rate(10);
	
	ros::Subscriber global_position_sub = nh.subscribe("/mavros/global_position/global",10,global_position_callback); 
	//ros::Subscriber pitch_sub = nh.subscribe("/mavros/imu/data",10,imu_callback);

	while (ros::ok())
	{
		spin();
	}

}