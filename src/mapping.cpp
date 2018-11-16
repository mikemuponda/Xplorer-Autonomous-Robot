#include <ros/ros.h>
#include <ca_msgs/Bumper.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int8MultiArray.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <string.h>
#include "nav_msgs/MapMetaData.h"
#include <stdlib.h> 
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
   double cell_size;
   double width;
   double height;
   bool isInitialized;
   std::string frame_id;
   GridMap map;
   //make this a pointer if you run into problems

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
   if(occupancy.isInitialized){
   left_bumper=msg->is_left_pressed;
   right_bumper=msg->is_right_pressed;
   }
    
}

void Occupancy::occupancyCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
{
   //Get occupancy grid parameters and set new grid map parameters
   //Either create 2 layers or create 2 maps
   //you want one object to be permanent and the other to be concatenated as they come in
if(isInitialized){
   map.setFrameId("map");
   width=msg->info.width;
   height=msg->info.height;
   frame_id=msg->header.frame_id;
   cell_size=msg->info.resolution;
   map.setGeometry(Length(height, width), cell_size);
   map.add("occupancy", -1.0);
   Position origin=Position(msg->info.origin.position.x, msg->info.origin.position.y);
   map.setPosition(origin);
   isInitalized=true;
}
//try to do as much of the code as possible within the callback
   GridMapRosConverter::fromOccupancyGrid(*msg,"occupancy", map);
   
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
    
    //Remember to uncomment and broadcast Occupancy grid
    ros::Publisher publisher = nh.advertise<nav_msgs::OccupancyGrid>("/new_occupancy_grid",1);
    ros::Rate loop_rate(30);
     
    

     int count=0;

     //fixed distance between bumper and center of robot, radius to bumper is about 6.83 inches
     double rbtbmprXoffst=0.174;
     double rbtbmprYoffst=0.174;

   
     while(ros::ok()){

    	nav_msgs::OccupancyGrid msg;
  	msg.header.stamp=ros::Time::now();
  	msg.header.frame_id=occupancy.frame_id;
        msg.info.map_load_time=ros::Time::now();
        msg.info.resolution=occupancy.cell_size;
        msg.info.width=occupancy.width;
        msg.info.height=occupancy.height;
        
        //This improves efficiciency in traversing grid_map 
        grid_map::Matrix& data = occupancy.map["occupancy"];
        //iterator to go through grid to find the right cells to mark according to odometry data
   	for (grid_map::GridMapIterator iterator(occupancy.map); !iterator.isPastEnd(); ++iterator)
           {
              Position curr_pos;
              occupancy.map.getPosition(*iterator, curr_pos); 

              Position odom_pos=Position(odom.x,odom.y);
              Position bmprOffst=Position(0.0,0.0);
             

              Index curr_index;
              occupancy.map.getIndex(curr_pos,curr_index);

              Index dest_index;
              
             
              //OVERSIMPLIFIED VERSION, Marking Bumper detected obstacle positions
               if(bumper.left_bumper)
                  {
                    if(curr_pos==odom_pos){
                          occupancy.map.atPosition("occupancy",odom_pos+bmprOffst)=100;
                      }

                //local space
                //global 
                    //case going forward
                    //case left
                    //case right
                    //case backwards
   	          }
                 
               if(bumper.right_bumper){
                   if(curr_pos==odom_pos){
                        occupancy.map.atPosition("occupancy",odom_pos+bmprOffst)=100; 
                      }
                 }

              /** fix lidar blind spots
               if(occupancy.map.atPosition("occupancy", odom_pos)==-1){
                  Position maxPos=Position(odom.x+0.00,odom.y+0.00);
                  while(curr_pos < maxPos){
                      }
                }
            **/ 
           }
         Publish as grid map
         GridMapRosConverter::toOccupancyGrid(occupancy.map,"occupancy",0,100,msg);
    	 publisher.publish(msg);
         ros::spinOnce();
         loop_rate.sleep();
         count++;

     }

    return 0;
}

