#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>

std::string lidar_name;

void Callback(const sensor_msgs::LaserScan::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  double min=msg->range_min;
  double max=msg->range_max;
  double x=0.0;
  double y=0.0;
  double point=0.0;
  double theta=0.0;

    for(int i=0;i<msg->ranges.size();i++){

            point = msg->ranges[i];


        if((point >= min)&&(point <= max)){
            theta = msg->angle_increment*i + msg->angle_min;
            x=point*cos(theta);
            y=point*sin(theta);
            
            transform.setOrigin(tf::Vector3(x, y, 0.0));
            tf::Quaternion q;
            //watchout for this theta...
            q.setRPY(0, 0, theta);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "Lidar", lidar_name));
         }

    }
}
  
 

int main(int argc, char** argv){
  ros::init(argc, argv, "Lidar_tf_broadcaster");
  if (argc != 2){ROS_ERROR("need lidar name as argument"); return -1;};
  lidar_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(lidar_name+"/LaserScan", 10, &Callback);

  ros::spin();
  return 0;
};
