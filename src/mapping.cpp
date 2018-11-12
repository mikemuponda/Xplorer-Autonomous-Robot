#include <ros/ros.h>
#include <ca_msgs/Bumper.h>
#include "mike_mapping/map.h"
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8MultiArray.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <string>
#include "nav_msgs/MapMetaData.h"

using namespace grid_map;

class Bumper 
{
  public:
    bool left_bumper;
    bool right_bumper;
  
     void bumperCallback(const ca_msgs::Bumper::ConstPtr& msg);
};

class Occupancy 
{
  public:
   bool convert;
   GridMap map;

     void occupancyCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};

class Odom 
{
  public:
    double x;
    double y;
    double z;

     void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

Bumper  bumper;
Occupancy occupancy;
Odom odom;

void Bumper::bumperCallback(const ca_msgs::Bumper::ConstPtr& msg) 
{
   left_bumper=msg->is_left_pressed;
   right_bumper=msg->is_right_pressed;
    
}

void Occupancy::occupancyCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
{
   
   map.setFrameId("map");
   //map.setGeometry(Length(1.2, 2.0), 0.03);
   map.add("occupancy");
   convert=GridMapRosConverter::fromOccupancyGrid(*msg,"occupancy",map);
   
}   

void Odom::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
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
    ros::Subscriber bmpr_sub = nh.subscribe("/bumper",100,&Bumper::bumperCallback,&bumper);
    ros::Subscriber occu_sub = nh.subscribe("/map",100,&Occupancy::occupancyCallback,&occupancy);
    ros::Subscriber odom_sub= nh.subscribe("/odom",100,&Odom::odomCallback,&odom);
    
    //Does Matt want grid_map or Occupancy grid
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    //ros::Publisher publisher = nh.advertise<nav_msgs::OccupancyGrid>("new_occupancy_grid",1);
    ros::Publisher pub =nh.advertise<mike_mapping::map>("/new_map",1000);
    ros::Rate loop_rate(100);
     
    

     int count=0;

     //fixed distance between bumper and center of robot, make this measurement..maybe split into x & y offsets
     double rbtbmprXoffst;
     double rbtbmprYoffst;

   
     while(ros::ok()){

    	mike_mapping::map msg;
    
  	msg.header.stamp=ros::Time::now();
  	msg.header.frame_id="/MSG_FRAME_TODO";

        //iterator to go through grid to find the right cells to mark according to odometry data
   	for (grid_map::GridMapIterator iterator(occupancy.map); !iterator.isPastEnd(); ++iterator)
           {
               if(bumper.left_bumper)
                  {
   	          // msg.new_map=SOMETHING;
   	          }
                 
               if(bumper.right_bumper){
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

