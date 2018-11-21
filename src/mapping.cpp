#include <ros/ros.h>
#include <ca_msgs/Bumper.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <string.h>
#include "nav_msgs/MapMetaData.h"
#include <stdlib.h> 
#include <tf/tf.h>
#include <tf/transform_listener.h>

using namespace grid_map;
ros::Publisher* publisher;//Adjust code for publisher

class Bumper 
{
  public:
    bool left_bumper,right_bumper;
    bool isInitialized;
 
     void bumperCallback(const ca_msgs::Bumper::ConstPtr& msg);
};

class Occupancy 
{
  public:
   double width, height,cell_size;
   bool isInit, convert;
   std::string frame_id;
   Index startlft;
   Index startrgt;
   Index endlft;
   Index endrgt;
   GridMap map;
   //make this a pointer if you run into problems

     void occupancyCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};

class Odom 
{
  public:
    double x,y,z;
    double X,Y,Z;
    // stamped_out=PoseStamped();
    geometry_msgs::PoseStamped ps_in;
    geometry_msgs::PoseStamped ps_out;
     tf::TransformListener* ptrListener;
     void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
};

Bumper  bumper;
Occupancy occupancy;
Odom odom;

void Bumper::bumperCallback(const ca_msgs::Bumper::ConstPtr& msg) 
{
   if(occupancy.isInit){
   left_bumper=msg->is_left_pressed;
   right_bumper=msg->is_right_pressed;
   }
   
    //This improves efficiciency in traversing grid_map 
         grid_map::Matrix& data = occupancy.map["bumper"];

        //left bumper
        if(bumper.left_bumper)
           {
            for(grid_map::LineIterator iterator(occupancy.map,startlft,endlft); !iterator.isPastEnd();++iterator)
             {
               map.at("bumper",*iterator)=100;
	      }
          }
          //Right bumper    
          if(bumper.right_bumper){

               for(grid_map::LineIterator iterator(occupancy.map,startrgt,endrgt); !iterator.isPastEnd();++iterator){
                        map.at("bumper",*iterator)=100; 
                      }
             }

        grid_map_msgs::GridMap message;
        GridMapRosConverter::toMessage(map,message);
        publisher.publish(message);
    
}

void Occupancy::occupancyCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
{
   
	if(!isInit)
         {
  	  map.add("occupancy",-1.0);
  	  Position origin=Position(msg->info.origin.position.x, msg->info.origin.position.y);
  	  map.setPosition(origin);
  	  isInit=true;
	 }

  	GridMapRosConverter::fromOccupancyGrid(*msg,"occupancy", map);

         if(!bumper.isInitialized){
  	         map.add("bumper", -1.0);
             bumper.isInitialized=true;
           }
  //publish original map anyways since map is always needed
  
  //also publish conatenated occupancy grid
  //publish mike_grid_map
  //publish mike_occupancy_grid-map
 	
}  

void Odom::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
{
   //get our position in terms of xyz
    x=msg->pose.pose.position.x;
    y=msg->pose.pose.position.y;
    z=msg->pose.pose.position.z;
    ps_in.header.frame_id = "base_footprint";
    ps_in.header.stamp=ros::Time::now();

    //left bumper start_pos
    ps_in.pose.position.x = 0.0; 
    ps_in.pose.position.y = 0.174; 
    ps_in.pose.position.z = 0.0;   
  
    ptrListener->transformPose("map",ps_in,ps_out);
    occupancy.startlft(ps_out.pose.position.x,ps_out.pose.position.y);

   //left bumper end_pos
    ps_in.pose.position.x = 0.174; 
    ps_in.pose.position.y = 0.0; 
    ps_in.pose.position.z = 0.0;   
  
    ptrListener->transformPose("map",ps_in,ps_out);
    occupancy.endlft(ps_out.pose.position.x,ps_out.pose.position.y);

    //Right bumper start_pos
    ps_in.pose.position.x = 0.0; 
    ps_in.pose.position.y = -0.174; 
    ps_in.pose.position.z = 0.0;   
  
    ptrListener->transformPose("map",ps_in,ps_out);
    occupancy.startrgt(ps_out.pose.position.x,ps_out.pose.position.y);

   
    //Right bumper end_pos
    ps_in.pose.position.x = -0.174; 
    ps_in.pose.position.y = 0.0; 
    ps_in.pose.position.z = 0.0;   
  
    ptrListener->transformPose("map",ps_in,ps_out);
    occupancy.endrgt(ps_out.pose.position.x,ps_out.pose.position.y);
}


  
    

int main(int argc,char** argv)
{
    ros::init(argc,argv,"new_map_node");
    ros::NodeHandle nh;
    publisher = nh.advertise<grid_map_msgs::GridMap>("/grid_map",1,true);
    ros::Subscriber bmpr_sub = nh.subscribe("/bumper",100,&Bumper::bumperCallback,&bumper);
    ros::Subscriber occu_sub = nh.subscribe("/map",100,&Occupancy::occupancyCallback,&occupancy);
    ros::Subscriber odom_sub= nh.subscribe("/odom",100,&Odom::odomCallback,&odom);
    
   
    odom.ptrListener=new(tf::TransformListener);
    ros::spin();
    //delete the publisher pointer
       
    return 0;
}

