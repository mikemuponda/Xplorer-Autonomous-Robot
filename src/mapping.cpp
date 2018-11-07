#include <ros/ros.h>
#include <ca_msgs/Bumper.h>
#include "mike_mapping/map.h"
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/grid_map_msgs.hpp>  //Need to verify this include
#include <nav_msgs/Odometry>

using namespace grid_map;

class Listener 
{
  public:
    bool left_bumper;
    bool right_bumper;
  
     void scanCallback(const ca_msgs::Bumper::ConstPtr& msg);
};

class Listener2 
{
  public:
    bool convert;
    GridMap map({"elevation"});

     void scanCallback2(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};

class Listener3 
{
  public:
    double x;
    double y;
    double z;

     void scanCallback3(const nav_msgs::Odometry::ConstPtr& msg);
};

Listener listener;
Listener2 listener2;
Listener3 listener3;

void Listener::scanCallback(const ca_msgs::Bumper::ConstPtr& msg) 
{
   left_bumper=msg->is_left_pressed;
   right_bumper=msg->is_right_pressed;

  //if either condition true, we need to mark that spot on the map and relative to the bumper contact side
    
}

void Listener2::scanCallback2(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
{

   //convert occupancy grid to grid map
   convert = grid_map::GridMapRosConverter::fromOccupancyGrid(msg,"elevation",map);

}   

void Listener3::scanCallback3(const nav_msgs::Odometry::ConstPtr& msg) 
{

   //get our position in terms of xyz
   x=msg->pose.pose.position.x;
   y=msg->pose.pose.position.y;
   z=msg->pose.pose.position.z;
   
}
    

int main(int argc,char** argv)
{
    //Choice..create 2 maps then concatenate or create one map and add new events 


    ros::init(argc,argv,"new_map_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/bumper",100,&Listener::scanCallback,&listener);
    ros::Subscriber sub2 = nh.subscribe("/map",100,&Listener2::scanCallback2,&listener2);
    ros::Subscriber sub2 = nh.subscribe("/odom",100,&Listener3::scanCallback3,&listener3);
    
    //Does Matt want grid_map or Occupancy grid
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    ros::Publisher pub =nh.advertise<mike_mapping::map>("/new_map",1000);
    ros::Rate loop_rate(100);
   

     int count=0;

     //fixed distance between bumper and center of robot, make this measurement..maybe split into x & y offsets
     double rbtbmprXoffst;
     double rbtbmprYoffst

   
     while(ros::ok()){

    	mike_mapping::map msg;
    
  	msg.header.stamp=ros::Time::now();
  	msg.header.frame_id="/MSG_FRAME_TODO";

        //iterator to go through grid to find the right cells to mark according to odometry data
   	for (grid_map::GridMapIterator iterator(listener2.map); !iterator.isPastEnd(); ++iterator)
           {
               if(listener.left_bumper)
                  {
   	          // msg.new_map=SOMETHING;
   	          }
                 
               if(listener.right_bumper){
                 //Do something to grid map and add to msg
                }
           }

   
    	 pub.publish(msg);
         ros::spinOnce();
         loop_rate.sleep();
         count++;

     }

    return 0;
}

