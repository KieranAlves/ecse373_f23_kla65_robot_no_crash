#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"

ros::Publisher *p_pub; 

bool closeToWall = false;

double wall_dist = 0.0;

void isCloseToWall(const sensor_msgs::LaserScan::ConstPtr& laser)
{


 std::vector<float> ranges = laser->ranges;


 for(size_t c = 0; c < ranges.size(); c++)
 {

  if (c > 264 && c < 401)
  {

    float range = ranges[c];


    if (range <= wall_dist)
    {
    
      closeToWall = true;


      break;


    }
  
    closeToWall = false;
  } else {

    float range = ranges[c];


    if (range <= wall_dist / 2)
    {
    
      closeToWall = true;


      break;


    }

  }



 }
 


}


void chatterCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  
  geometry_msgs::Twist twist_msg;

  if (closeToWall == true)
  {

    twist_msg.linear.x = 0.0;  
    twist_msg.angular.z = msg->angular.z;

  } else {

    twist_msg.linear.x = msg->linear.x;  
    twist_msg.angular.z = msg->angular.z;

  }

  p_pub->publish(twist_msg);

}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "movement_reader");

  
  ros::NodeHandle n;

  p_pub = new ros::Publisher(n.advertise<geometry_msgs::Twist>("cmd_vel", 1000));

  ros::Subscriber laserSub = n.subscribe<sensor_msgs::LaserScan>("laser_0", 1000, isCloseToWall);
  
  ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("des_vel", 1000, chatterCallback);

  n.getParamCached("wall_dist", wall_dist);

  


 
  
  ros::spin();

  delete p_pub;

  return 0;
}