#include "ros/ros.h"
#include <tf/transform_broadcaster.h>

int main(int argc, char**argv){

  ros::init(argc,argv, "liiiidar");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;

  ros::Rate rate(10);
 
  while(node.ok()){
  
  transform.setOrigin(tf::Vector3(0.0,0.0,0.0));
  transform.setRotation(tf::Quaternion(0,0,0,1));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"scan1", "scan2"));
  rate.sleep();

  }
 return 0;
};
