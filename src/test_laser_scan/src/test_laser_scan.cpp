#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
//
class Listener 
{
  public:
    double min;
    double max;
    double theta;
    double point;
    double x;
    double y;
    double X [360];
    double Y [360];

     void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};

Listener listener;
void Listener::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    min=msg->range_min;
    max=msg->range_max;
    theta=0.0;
    point=0.0;
    x=0.0;
    y=0.0;
    
      for(int i=0;i<msg->ranges.size();i++){

         point = msg->ranges[i];
         
        if((point >= min)&&(point <= max)){
            theta = (msg->angle_increment*i) + msg->angle_min;
            x=point*cos(theta);
            y=point*sin(theta);
            listener.X[i]=x;
            listener.Y[i]=y;
            ROS_INFO("X: %f Y: %f DIST: %f",x,y,point);
        }

    }
   
}
    

int main(int argc,char** argv)
{
    ros::init(argc,argv,"Mapping_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan",100,&Listener::scanCallback,&listener);
    //ros::Publisher pub =nh.advertise<test_laser_scan::Points>("/cartesianpoints",1);
    ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("Occupancy",100);
    //Rplidar scan frequency is hovers around 11.310-11.325hz
    ros::Rate loop_rate(11.325);
   
    uint32_t shape = visualization_msgs::Marker::CUBE;

    int count=0;
   // int 2dGrid[20][20]={FALSE};

    while(ros::ok()){
     for (int i=0;i<360;i++){
      //ROS_INFO("xx: %f, yy: %f",listener.X[i],listener.Y[i]);
      
      
      //Visualization
         visualization_msgs::Marker marker;
         marker.header.frame_id = "/laser";
         marker.header.stamp =ros::Time::now();
         marker.ns="Mapping_node";
         marker.id=i;
         marker.type=shape;
         marker.action=visualization_msgs::Marker::ADD;
         marker.pose.position.x=listener.X[i];
         marker.pose.position.y=listener.Y[i];
         marker.pose.position.z=0.0;
         marker.pose.orientation.x = 0.0;
         marker.pose.orientation.y = 0.0;
         marker.pose.orientation.z = 0.0;
         marker.pose.orientation.w = 1.0;
         marker.scale.x = 0.5;
         marker.scale.y = 0.5;
         marker.scale.z = 1.0;
         marker.color.r = 1.0f;
         marker.color.g = 1.0f;
         marker.color.b = 0.0f;
         marker.color.a = 1.0;
         marker.lifetime =ros::Duration();
         pub.publish(marker);
         
         //GRID
         
       }
     
         ros::spinOnce();
         loop_rate.sleep();
         ++count;

       //test_laser_scan::Points msg;
       //msg.header.stamp=ros::Time::now();
       //msg.x=x;
       //msg.y=y;
       //pub.publish(msg);
    }

    return 0;
}
