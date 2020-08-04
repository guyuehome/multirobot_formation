

//Master node should guide the master robot autonomously without bumping into
//objects and the slave should subscribe for master position
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <sstream>

#define PI 3.14


double intensities[27];
double mul =1;

ros::Publisher velocity_publisher;

//Function declerations of move and rotate
void move(double speed, double distance, bool isForward);
void rotate (double angular_speed, double relative_angle, bool clockwise);

//Call back decleration for the laser messages
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg);

//Function declearations for equilidian distance and degrees to radians conversion
double getDistance(double x1, double y1, double x2, double y2);
double degrees2radians(double angle_in_degrees);

//Wanader without bumping into obstacles 
void wander(void);

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "master");
  ros::NodeHandle masterNode;
 
  velocity_publisher = masterNode.advertise<geometry_msgs::Twist>("/ares1/cmd_vel", 1000);
  ros::Subscriber laser = masterNode.subscribe("/ares1/scan", 1, laserCallBack);
  ros::Rate loop_rate(100000);

  while (ros::ok())
  {
	wander();
  }
 
  ros::spinOnce();

  return 0;
}

/**
 *  makes the robot move with a certain linear velocity, for 
 *  a certain distance either forward or backward  
 */
void move(double speed, double distance, bool isForward){
   geometry_msgs::Twist vel_msg;
   //set a random linear velocity in the x-axis and check condition for the direction
   if (isForward)
	   vel_msg.linear.x =fabs(speed);
   else
	   vel_msg.linear.x =-fabs(speed);

   vel_msg.linear.y =0;
   vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
   vel_msg.angular.x = 0;
   vel_msg.angular.y = 0;
   vel_msg.angular.z =0;

   double t0 = ros::Time::now().toSec();
   double current_distance = 0.0;
   ros::Rate loop_rate(100);
   //Condition to terminate if moved to the distance specified
   do{
	   velocity_publisher.publish(vel_msg);
	   double t1 = ros::Time::now().toSec();
	   current_distance = speed * (t1-t0);
	   ros::spinOnce();
	   loop_rate.sleep();
   }while(current_distance<distance);
   vel_msg.linear.x =0;
   //velocity_publisher.publish(vel_msg);

}

/**
 *  makes the robot turn with a certain angular velocity, for 
 *  a certain distance in either clockwise or counter-clockwise direction  
 */
void rotate (double angular_speed, double relative_angle, bool clockwise){
//angular_speed = degrees2radians(angular_speed);
//relative_angle = degrees2radians(relative_angle);
	geometry_msgs::Twist vel_msg;
	   //set a random linear velocity in the x-axis
	   vel_msg.linear.x =0;
	   vel_msg.linear.y =0;
	   vel_msg.linear.z =0;
	   //set a random angular velocity in the y-axis
	   vel_msg.angular.x = 0;
	   vel_msg.angular.y = 0;
	   //Condition to rotate clockwise or counter-clockwise
           if (clockwise)
	   		   vel_msg.angular.z =-abs(angular_speed);
	   	   else
	   		   vel_msg.angular.z =abs(angular_speed);

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);
           //Condition used to terminate if rotated to the specifed angle
	   do{
		   velocity_publisher.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
	   }while(current_angle<relative_angle);
	   vel_msg.angular.z =0;
	   velocity_publisher.publish(vel_msg);
}

/**
 *  converts angles from degree to radians  
 */

double degrees2radians(double angle_in_degrees){
	return angle_in_degrees *PI /180.0;
}

/*
 * get the euclidian distance between two points 
 */
double getDistance(double x1, double y1, double x2, double y2){
	return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
}

/*
 * Call back implementation to read and process laser data  
 */
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg)
{
//ROS_INFO("I am in: [%s]", "laser call back");
for (int i=0; i<27; i++) // I need not loop to copy, I am not familiar with std::vectors
{
intensities [i]= laser_msg->intensities[i];
mul = mul*intensities[i]; //check point if robot is blocked 270 degrees
}
}

/*
 *"Wander without hitting anything" implementation
 */
void wander(void)
{
//ROS_INFO("I am [%s]", "wandering");
int samples = 27;
int fov = 4.7123;
double inc = 0.1745; // 270/27 degrees to radians
int center = samples/2;
if (mul == 1)// blocked around 270 degrees
{
//rotate(1.0, 3.1415, 1); //about turn
}
if ((intensities [center-1] == 1)||(intensities [center] == 1)||(intensities [center+1] == 1))// obstacle in front
{
	//Check one by one on both sides of the robot to determine free space and rotate by the amount scanned in a first free direction
	for (int i = 2; i< center; i++)
	{
		if(intensities [center - i] == 0)// no obstacle
		{
		//rotate(1.0, (i+1)*inc, 1);//move by navigation
		break;
		}
		else if (intensities [center +i] == 0)// no obstacle
		{
		//rotate(1.0, (i+1)*inc, 0);//move by navigation
		break;
		}
	}
}
else
{
move(0.5, 1.0, 1);  // or move by navigation
}

}

