#include "lucia_sim_benchmark_asp_2014.h"

#include <kobuki_msgs/BumperEvent.h>
#include <kobuki_msgs/Sound.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <ros/time.h>
#include "std_msgs/Float32.h"
#include "rosoclingo/ROSoClingoActionGoal.h"
#include <move_base_msgs/MoveBaseActionGoal.h>

bool bool_start = false;
bool bool_time_pb1 = false;
bool bool_time_pb2 = false;
bool bool_time_out = false;

bool bool_panels[4];

long double time_to_pb1 = 110.0;
long double time_to_pb2 = 180.0;
long double time_out    = 220.0;

uint BENCH_PB = 0;
uint BENCH_A = 0;

#define num_nim_bumper 200


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  map.header = msg->header;
  map.info = msg->info;
  map.data = msg->data;
}

void startCallbak(const move_base_msgs::MoveBaseActionGoal msg){ bool_start = true;}

void achievCallback(const kobuki_msgs::Sound msg){ BENCH_A++; ROS_INFO("A: Panel achieved"); }

//============================================================
// Def Bumper Events
//============================================================
void bumper1Callback(const kobuki_msgs::BumperEvent bumperMessage){
//   ROS_INFO("Bumper event on Turtulebot 1: [Bumper ID, Value] = [%d, %d]", bumperMessage.bumper, bumperMessage.state);
  if(bumperMessage.state==1){
    ROS_INFO("PB: Robot hit");
    BENCH_PB++;
  }
}

//============================================================
// Def Laser Events
//============================================================
void laser1Callback(const sensor_msgs::LaserScan laser){

  int nnans = 0;
  int notnan = 0;
  for(int iter = 0; iter < laser.ranges.size(); iter++){
    if(isnan(laser.ranges.at(iter)))
      nnans++;
    else
      notnan++;
  }

// std::cout<< "NAN: "<< nnans << "  NotNAN: " << notnan <<std::endl;
// std::cout<<laser.angle_min << ", "<< laser.angle_increment << ", " << laser.angle_max << ", " <<std::endl;
  if(nnans > num_nim_bumper){
    ROS_INFO("PB: Turtlebot 1 Bumper Event");
    BENCH_PB++;
  }
}


//======================================================================================//
// Main
//======================================================================================//
int main(int argc, char** argv){

   ros::init(argc, argv, "lucia_sim_2014_benchmark_node_asp");
   ros::Time::init();

   ros::NodeHandle n;
   ros::Subscriber sub = n.subscribe("map", 10000, mapCallback);
   
   std::cout<< "Four Achievments:"<<std::endl;
   std::cout<< "Penalization Behaviors:"<<std::endl;
   std::cout<< "  -> Robot bumps something:"<<std::endl;
   std::cout<< "  -> Excedes time: " << time_to_pb1 <<std::endl;   
   std::cout<< "  -> Excedes time: " << time_to_pb2 <<std::endl;
   
   // Bumper callback
   ros::Subscriber bumper_1 = n.subscribe("/turtlebot_1/events/bumper",100,bumper1Callback);
   // ros::Subscriber bumper_1 = n.subscribe("/turtlebot_1/scan",1,laser1Callback);
   
   ros::Subscriber achiev = n.subscribe("/turtlebot_1/mobile_base/commands/sound", 100, achievCallback);
   
   // Start signal
   // ros::Subscriber start_info = n.subscribe("/ROSoClingo/goal", 100, startCallbak);
   ros::Subscriber start_info = n.subscribe("/turtlebot_1/move_base/goal", 100, startCallbak);
   
   //=====================================================

   //QR code:
   ros::ServiceClient getQR_tb1 = n.serviceClient<services::getQR>("turtlebot_1/getQR");
  
   //send goal:
   ros::ServiceClient sendGoal_tb1 = n.serviceClient<services::sendGoal>("turtlebot_1/sendGoal");

   //get location:
   ros::ServiceClient getLoc_tb1 = n.serviceClient<services::getLocation>("turtlebot_1/getLocation");
   services::getLocation Loc_tb1,Loc_tb2,Loc_tb3,Loc_tb4;
   Loc_tb1.request.read=READ;
  
   //start signal
   ros::ServiceClient getPanelLoc = n.serviceClient<services::getPanel>("/getPanel");

  //QR code:
  services::getQR QR_tb1;
  QR_tb1.request.read=READ;

  //=====================================================
  // Benchmarking

  ros::Rate loop_rate(FREQUENCY);

  ROS_INFO("BENCHMARK is waiting...");
  while(ros::ok()){
  //   ros::spinOnce();
    if (bool_start==true)
	break;
    else
    ros::spinOnce();
    loop_rate.sleep();
  }

  
  double loc_x, loc_y, loc_z, loc_x0, loc_y0, loc_z0, total_distance;
  total_distance = 0.0;
  
  long double time_begin = ros::Time::now().toSec();
  ROS_INFO("Strat BENCHMARK: time %f", (float)time_begin);
  
  //get locatiolns:
  int l1 = getLoc_tb1.call(Loc_tb1);
  if (l1){
       loc_x0 = Loc_tb1.response.x;
       loc_y0 = Loc_tb1.response.y;
       loc_z0 = Loc_tb1.response.z;
     }
     else{
       ROS_ERROR("Failed to call 'location' services");
     }

     
  
  
   while (ros::ok())
   {
     //=====================================================
     // Check for PBs Regarding Time
     //=====================================================
     long double time_to_PB = ros::Time::now().toSec();
     
     //get locatiolns:
     int l1 = getLoc_tb1.call(Loc_tb1);
     if (l1){
       loc_x = Loc_tb1.response.x;
       loc_y = Loc_tb1.response.y;
       loc_z = Loc_tb1.response.z;
       
       loc_x0 = loc_x - loc_x0;
       loc_y0 = loc_y - loc_y0;
       loc_z0 = loc_z - loc_z0;
       
       total_distance += sqrt(loc_x0*loc_x0 + loc_y0*loc_y0 + loc_z0*loc_z0);
       
       loc_x0 = loc_x;
       loc_y0 = loc_y;
       loc_z0 = loc_z;
       
//        ROS_INFO("LOC: [%f][%f][%f]", Loc_tb1.response.x,Loc_tb1.response.y,Loc_tb1.response.theta);
       std::cout<<"\r";
       std::cout<<"Total distance: "<< total_distance;
     }
     else{
       ROS_ERROR("Failed to call 'location' services");
     }

     
     
//      ROS_INFO("Iter BENCHMARK: time %f", (float)time_to_PB);
     if ((time_to_PB-time_begin) > time_to_pb1 && bool_time_pb1==false){
       bool_time_pb1=true;
       BENCH_PB += 1;
       ROS_INFO("PB: Time");
     }
     
     if ((time_to_PB-time_begin) > time_to_pb2 && bool_time_pb2==false){
       bool_time_pb2=true;
       BENCH_PB += 1;
       ROS_INFO("PB: Time");
     }
     
     if ((time_to_PB-time_begin) > time_out && bool_time_out==false){
       bool_time_out = true;
       ROS_INFO("Time out");
       break;
     }

      if(BENCH_A==4)
	break;
      
    ros::spinOnce();
    loop_rate.sleep();
   }
   
   
   long double time_end = ros::Time::now().toSec();
   ROS_INFO("Stop time: %f", (float)time_end);
  
   std::cout<<"BENCHMARK INFO: "<<std::endl;
   std::cout<<"Achievments: "<< BENCH_A <<std::endl;
   std::cout<<"Penalization Behaviors: "<< BENCH_PB <<std::endl;
   if (bool_time_out)
     std::cout<<"Time: TIME OUT"<<std::endl;
   else
     std::cout<<"Time: " << (float)(time_end-time_begin) << std::endl;
   std::cout<<"Complete distance: "<< total_distance << std::endl;
  
 return 0;

}

//======================================================================================//
// EOF
//======================================================================================//

