#include <ros/ros.h>
#include <ca_msgs/Bumper.h>
#include "mike_mapping/map.h"
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Odometry>

using namespace grid_map;

//Need another listener and callback for nav messages odometry to get position

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
Listener listener;
Listener2 listener2;


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

  //Need an iterator to go through grid to find th right cells to mark according to odometry data
   	
   
}
    

int main(int argc,char** argv)
{
    //Choice..create 2 maps then concatenate or create one map and add new events 


    ros::init(argc,argv,"new_map_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/bumper",100,&Listener::scanCallback,&listener);
    ros::Subscriber sub2 = nh.subscribe("/map",100,&Listener2::scanCallback2,&listener2);
    
    //Does Matt want grid_map or Occupancy grid
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);

    ros::Publisher pub =nh.advertise<mike_mapping::map>("/new_map",1000);
    ros::Rate loop_rate(100);
   

    int count=0;
   
     while(ros::ok()){

    	mike_mapping::map msg;
    
  	msg.header.stamp=ros::Time::now();
  	msg.header.frame_id="/MSG_FRAME_TODO";

  	  if(1==2/* Add bumper event*/){
   	   // msg.new_map=SOMETHING;
           //case right bumper, case left bumper & case both bumpers
   	   }
   
    	 pub.publish(msg);
         ros::spinOnce();
         loop_rate.sleep();
         count++;

     }

    return 0;
}

