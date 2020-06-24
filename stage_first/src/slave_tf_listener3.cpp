

//Slave follower implementation 

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "sensor_msgs/LaserScan.h"

double intensities [27];
double mul = 1; 

ros::Publisher slave_vel;

//Function declerations to avoid, rotate and move
void avoid(void);
void rotate (double angular_speed, double relative_angle, bool clockwise);
void move(double speed, double distance, bool isForward);

// Laser call back function decleration
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg); 

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;


  slave_vel =
    node.advertise<geometry_msgs::Twist>("/ares3/cmd_vel", 10);
  ros::Subscriber laser = node.subscribe("/ares3/scan", 1, laserCallBack);

  tf::TransformListener listener;

  ros::Rate rate(1000.0);
  while (node.ok()){
    tf::StampedTransform transformSM;
    tf::StampedTransform transformMS;
    //From Slave to master transformation
    try{
      listener.lookupTransform("/ares3", "/ares1",
                               ros::Time(0), transformSM);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    
    //From Master to Slave transformation
    try{
      listener.lookupTransform("/ares1", "/ares3",
                               ros::Time(0), transformMS);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    avoid();//function call to avoid obstacles while following the Master
    
    //Priting the x, y, and angle "from the robot's perspective" of each robot to the other
    ROS_INFO("Master pose w.r.t Slave [x, y]: [%f, %f]", transformSM.getOrigin().x(), transformSM.getOrigin().y());
    ROS_INFO("Orientation: [%f]", atan2(transformSM.getOrigin().y(),
                                   transformSM.getOrigin().x()));
    ROS_INFO("---------------------------------------------");
    ROS_INFO("Slave pose w.r.t Master [x, y]: [%f, %f]", transformMS.getOrigin().x(), transformMS.getOrigin().y());
    ROS_INFO("Orientation: [%f]", atan2(transformMS.getOrigin().y(),
                                   transformMS.getOrigin().x()));
    //Proportional controller to follow the Master (Go to master behaviour)
    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = 4.0 * atan2(transformSM.getOrigin().y(),
                                   transformSM.getOrigin().x());
    vel_msg.linear.x = 0.25 * sqrt(pow(transformSM.getOrigin().x(), 2) +
                                  pow(transformSM.getOrigin().y(), 2));
    slave_vel.publish(vel_msg);
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};

/*
 * Call back implementation to read and process laser data  
 */
void laserCallBack(const sensor_msgs::LaserScan::ConstPtr & laser_msg)
{
//ROS_INFO("I am in: [%s]", "laser call back");

for (int i=0; i<27; i++) // I need not loop to copy, I not familiar with std::vectors
{
intensities [i]= laser_msg->intensities[i];
mul = mul*intensities[i]; //check point if robot is blocked 270 degrees
}
}

/*
 * Obstacle avoidance behaviour implementation 
 */
void avoid(void)
{
  //ROS_INFO("I am [%s]", "avoiding");

int samples = 27;
int fov = 4.7123;
double inc = 0.1745; // 270/27 degrees to radians
int center = samples/2;
/*if (mul == 1)// blocked around 270 degrees
{
rotate(1.0, 3.1415, 1); //about turn
}*/
if ((intensities [center-1] == 1)||(intensities [center] == 1)||(intensities [center+1] == 1))// obstacle in front
{
	//Check one by one on both sides of the robot to determine free space and rotate by the amount scanned in a first free direction
	for (int i = 2; i< center; i++)
	{
		if(intensities [center - i] == 0)// no obstacle
		{
		rotate(1.0, (i+1)*inc, 1);
		break;
		}
		else if (intensities [center +i] == 0)// no obstacle
		{
		rotate(1.0, (i+1)*inc, 0);
		break;
		}
	}
}
else
{
	//move(1.0, 1.0, 1);
}

}


/**
 *  makes the robot turn with a certain angular velocity, 
 *  either clockwise or counter-clockwise direction  
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
	   //Control strategy for clockwise and counter-clockwise directions
	   if (clockwise)
	   		   vel_msg.angular.z =-abs(angular_speed);
	   	   else
	   		   vel_msg.angular.z =abs(angular_speed);

	   double t0 = ros::Time::now().toSec();
	   double current_angle = 0.0;
	   ros::Rate loop_rate(1000);
           //Condition to teminate rotation after rotating to the required orientation 
	   do{
		   slave_vel.publish(vel_msg);
		   double t1 = ros::Time::now().toSec();
		   current_angle = angular_speed * (t1-t0);
		   ros::spinOnce();
		   loop_rate.sleep();
		   
	   }while(current_angle<relative_angle);
	   vel_msg.angular.z =0;
	   slave_vel.publish(vel_msg);
}


/**
 *  makes the robot turn with a certain linear velocity, for 
 *  a certain distance either forward or backward  
 */
void move(double speed, double distance, bool isForward){
   geometry_msgs::Twist vel_msg;
   //set a random linear velocity in the x-axis and condition to determine direction of movement
   if (isForward)
	   vel_msg.linear.x =abs(speed);
   else
	   vel_msg.linear.x =-abs(speed);
   vel_msg.linear.y =0;
   vel_msg.linear.z =0;
   //set a random angular velocity in the y-axis
   vel_msg.angular.x = 0;
   vel_msg.angular.y = 0;
   vel_msg.angular.z =0;

   double t0 = ros::Time::now().toSec();
   double current_distance = 0.0;
   ros::Rate loop_rate(100);
   //Condition to teminate movement after raeching the required distance 
   do{
	   slave_vel.publish(vel_msg);
	   double t1 = ros::Time::now().toSec();
	   current_distance = speed * (t1-t0);
	   ros::spinOnce();
	   loop_rate.sleep();
   }while(current_distance<distance);
   vel_msg.linear.x =0;
   slave_vel.publish(vel_msg);

}

