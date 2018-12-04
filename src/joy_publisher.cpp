#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include "apriltags2_ros/AprilTagDetectionArray.h"
#include <math.h>
#define pi 3.14159265

geometry_msgs::Pose pose;
ros::Subscriber april_sub;
double roll,pitch,yaw,x,y,z;
ros::Publisher arm_pub;
ros::Time prevWrite;
ros::Duration writeDelay(0.1);
ros::Subscriber arm_sub;
geometry_msgs::Pose arm_posn;
tf::Vector3 quad_global;
// void viconCallback(geometry_msgs::Vector3 position)
// {
//   float bilboz = position.z - 0.3;
//   yaw = atan2(sqrt(position.x*position.x+position.y*position.y),bilboz)*180/pi;
//   roll = atan2(sqrt(bilboz*bilboz+position.x*position.x),position.y)*180/pi;
//   pitch = atan2(sqrt(bilboz*bilboz+position.y*position.y),position.x)*180/pi;
//   pose.orientation = tf::createQuaternionMsgFromRollPitchYaw((roll*pi/180),(pitch*pi/180),(yaw*pi/180));
//   float sph = 0.5/(sqrt(position.x*position.x + position.y*position.y + bilboz*bilboz));
//   pose.position.x = sph*position.x;
//   pose.position.y = sph*position.y;
//   pose.position.z = sph*bilboz;
//   if(ros::Time::now()-prevWrite>writeDelay)
//   {
//   arm_pub.publish(pose);
//    prevWrite=ros::Time::now();
//  }

// }


void aprilCallback(apriltags2_ros::AprilTagDetectionArray tag_detection_array)
{
  geometry_msgs::Pose position;
  for (unsigned int i=0; i<tag_detection_array.detections.size(); i++) 
  {
    position = tag_detection_array.detections[i].pose.pose.pose;
    tf::Vector3 quadpos(position.position.x,position.position.y,position.position.z);
    //std::cout<<"Quadpos "<<quadpos.getX()<<" "<<quadpos.getY()<<" "<<quadpos.getZ()<<std::endl;
    tf::Vector3 campos(arm_posn.position.x,arm_posn.position.y,arm_posn.position.z);
    //std::cout<<"Campos "<<campos.getX()<<" "<<campos.getY()<<" "<<campos.getZ()<<std::endl;
    tf::Quaternion q(arm_posn.orientation.x,arm_posn.orientation.y,arm_posn.orientation.z,arm_posn.orientation.w);
    tf::Matrix3x3 rotmat(q);
    quad_global = campos + rotmat*quadpos;
    x = quad_global.getX();
    y = quad_global.getY();
    z = quad_global.getZ();
    std::cout<<x<<" "<<y<<" "<<z<<std::endl;
    //std::cout<<position.position.x<<" "<<position.position.y<<" "<<position.position.z<<std::endl;
  
     yaw = atan2(y,x)*180/pi;
   roll = 0;
   pitch = atan2(z,sqrt(x*x+y*y))*180/pi;
   pose.orientation = tf::createQuaternionMsgFromRollPitchYaw((roll*pi/180),(pitch*pi/180),(yaw*pi/180));
float sph = 0.5/(sqrt(x*x + y*y + z*z));
  pose.position.x = sph*x;
  pose.position.y = sph*y;
  pose.position.z = sph*z;
  if(ros::Time::now()-prevWrite>writeDelay)
  {
  arm_pub.publish(pose);

   prevWrite=ros::Time::now();
 }
}
}

void armCallback(geometry_msgs::PoseStamped arm_pose)
{
  //std::cout<<arm_pose.pose.position.x<<" "<<arm_pose.pose.position.y<<" "<<arm_pose.pose.position.z<<std::endl;
  arm_posn = arm_pose.pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_turtle");
  ros::NodeHandle nh;
  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;
  prevWrite=ros::Time::now();
  april_sub = nh.subscribe("/tag_detections", 100, aprilCallback);
  arm_sub = nh.subscribe("/m1n6s300_driver/out/tool_pose", 100, armCallback);
  arm_pub = nh.advertise<geometry_msgs::Pose>("orbot/arm_pose",100);



  ros::spin();
}
