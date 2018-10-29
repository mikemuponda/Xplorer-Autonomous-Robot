#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>




void Callback(const geometry_msgs::Pose::ConstPtr& msg){

static tf2_ros::TransformBroadcaster br;
geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "base_link";
  transformStamped.child_frame_id = "odom";
  transformStamped.transform.translation.x = msg->position.x;
  transformStamped.transform.translation.y = msg->position.y;
  transformStamped.transform.translation.z = msg->position.z;
  
  transformStamped.transform.rotation.x = msg->orientation.x;
  transformStamped.transform.rotation.y = msg->orientation.y;
  transformStamped.transform.rotation.z = msg->orientation.z;
  transformStamped.transform.rotation.w = msg->orientation.w;
   br.sendTransform(transformStamped);
      
}
  

int main(int argc, char** argv){
  ros::init(argc, argv, "base_link_odom");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/Pose", 10, &Callback);
    
  ros::spin();

  return 0;
};
