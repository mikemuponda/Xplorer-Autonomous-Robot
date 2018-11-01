#include <ros/ros.h>
#include <ca_msgs/Bumper.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mike_mapping/map.h>

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

     void scanCallback(const ca_msgs::Bumper::ConstPtr& msg);
};

class Listener2 
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

     void scanCallback2(const nav_msgs::OccupancyGrid::ConstPtr& msg);
};
Listener listener;
Listener2 listener2;

void Listener::scanCallback(const ca_msgs::Bumper::ConstPtr& msg) 
{
    min=msg->range_min;
    max=msg->range_max;
    theta=0.0;
    point=0.0;
    x=0.0;
    y=0.0;
    
   
}

void Listener2::scanCallback2(const nav_msgs::OccupancyGrid::ConstPtr& msg) 
{
   
   
}
    

int main(int argc,char** argv)
{
    ros::init(argc,argv,"Compound_Map_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/map",100,&Listener::scanCallback,&listener);
    ros::Subscriber sub = nh.subscribe("/bumper",100,&Listener2::scanCallback2,&listener2);

    ros::Publisher pub =nh.advertise<mike_mapping::map>("/new_map",1000);
    //ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("Occupancy",100);

    ros::Rate loop_rate(100);
   
    uint32_t shape = visualization_msgs::Marker::CUBE;

    int count=0;
   
     while(ros::ok()){

    	mike_mapping::map msg;
    
  	msg.header.stamp=ros::Time::now();
  	msg.header.frame_id="/MSG_FRAME_TODO";

  	  if(/*bumper event*/){
   	    msg.new_map=100;
   	   }
   
    	 pub.publish(msg);
         ros::spinOnce();
         loop_rate.sleep();
         count++;

     }

    return 0;
}

