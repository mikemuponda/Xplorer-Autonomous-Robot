/*
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void Callback(const nav_msgs::Odometry::ConstPtr& msg){

tf2_ros::TransformBroadcaster br;
geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = msg->pose.pose.position.z;
  
  transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;
  br.sendTransform(transformStamped);
      
}
  

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_base_publisher");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/odom", 100, &Callback);
    
  ros::spin();

  return 0;
};
*/
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


tf::Transform transform;
tf::Quaternion q;

void pose_callback(const nav_msgs::OdometryConstPtr& msg)
{
    static tf::TransformBroadcaster br;
    q.setX(msg->pose.pose.orientation.x);
    q.setY(msg->pose.pose.orientation.y);
    q.setZ(msg->pose.pose.orientation.z);
    q.setW(msg->pose.pose.orientation.w);

    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.0));
    transform.setRotation(q);

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_base_publisher");
    ros::NodeHandle n;
    ros::Subscriber pose_sub = n.subscribe("/odom", 10, pose_callback);
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
