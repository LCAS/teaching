#include "lucia_sim_benchmark_csp_2014.h"

#include <kobuki_msgs/BumperEvent.h>
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include <ros/time.h>
#include "std_msgs/Float32.h"

bool bool_start = false;
bool bool_time_pb1 = false;
bool bool_time_pb2 = false;
bool bool_time_out = false;

bool bool_found_panel_c1 = false;
bool bool_found_panel_c2 = false;
bool bool_found_panel_c3 = false;
bool bool_found_panel_c4 = false;

bool bool_bumper_state_tb1 = false;
bool bool_bumper_state_tb2 = false;
bool bool_bumper_state_tb3 = false;
bool bool_bumper_state_tb4 = false;


bool bool_panels[4];


long double time_to_pb1 = 25.0;
long double time_to_pb2 = 50.0;
long double time_out    = 100.0;

uint BENCH_PB = 0;
uint BENCH_A = 0;

#define num_nim_bumper 30


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  map.header = msg->header;
  map.info = msg->info;
  map.data = msg->data;
}

void startCallbak(const std_msgs::String::ConstPtr& msg){
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  
  if (msg->data == "active"){
    bool_start = true;
  }
}

//============================================================
// Def Bumper Events
//============================================================
void bumper1Callback(const kobuki_msgs::BumperEvent bumperMessage){
  ROS_INFO("Bumper event on Turtulebot 1: [Bumper ID, Value] = [%d, %d]", bumperMessage.bumper, bumperMessage.state);
  if(bumperMessage.state==1 & bool_bumper_state_tb1 == false ){
    ROS_INFO("PB: Robot hit");
    BENCH_PB++;
    bool_bumper_state_tb1 = true;
  }
}

void bumper2Callback(const kobuki_msgs::BumperEvent bumperMessage){
  ROS_INFO("Bumper event on Turtulebot 2: [Bumper ID, Value] = [%d, %d]", bumperMessage.bumper, bumperMessage.state);
  if(bumperMessage.state==1 ){
    ROS_INFO("PB: Robot hit");
    BENCH_PB++;
  }
}

void bumper3Callback(const kobuki_msgs::BumperEvent bumperMessage){
  ROS_INFO("Bumper event on Turtulebot 3: [Bumper ID, Value] = [%d, %d]", bumperMessage.bumper, bumperMessage.state);
  if(bumperMessage.state==1 ){
    ROS_INFO("PB: Robot hit");
    BENCH_PB++;
  }
}

void bumper4Callback(const kobuki_msgs::BumperEvent bumperMessage){
  ROS_INFO("Bumper event on Turtulebot 4: [Bumper ID, Value] = [%d, %d]", bumperMessage.bumper, bumperMessage.state);
  if(bumperMessage.state==1 ){
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

//   std::cout<< "NAN: "<< nnans << "  NotNAN: " << notnan <<std::endl;
//   std::cout<<laser.angle_min << ", "<< laser.angle_increment << ", " << laser.angle_max << ", " <<std::endl;
  if(nnans > num_nim_bumper){
    ROS_INFO("PB: Turtlebot 1 Bumper Event");
    BENCH_PB++;
  }
}

void laser2Callback(const sensor_msgs::LaserScan laser){

  int nnans = 0;
  int notnan = 0;
  for(int iter = 0; iter < laser.ranges.size(); iter++){
    if(isnan(laser.ranges.at(iter)))
      nnans++;
    else
      notnan++;
  }

  if(nnans > num_nim_bumper){
    ROS_INFO("PB: Turtlebot 2 Bumper Event");
    BENCH_PB++;
  }
}

void laser3Callback(const sensor_msgs::LaserScan laser){
  int nnans = 0;
  int notnan = 0;
  for(int iter = 0; iter < laser.ranges.size(); iter++){
    if(isnan(laser.ranges.at(iter)))
      nnans++;
    else
      notnan++;
  }

  if(nnans > num_nim_bumper){
    ROS_INFO("PB: Turtlebot 3 Bumper Event");
    BENCH_PB++;
  }
}

void laser4Callback(const sensor_msgs::LaserScan laser){
  int nnans = 0;
  int notnan = 0;
  for(int iter = 0; iter < laser.ranges.size(); iter++){
    if(isnan(laser.ranges.at(iter)))
      nnans++;
    else
      notnan++;
  }

  if(nnans > num_nim_bumper){
    ROS_INFO("PB: Turtlebot 4 Bumper Event");
    BENCH_PB++;
  }
}


//======================================================================================//
// Main
//======================================================================================//
int main(int argc, char** argv){

   ros::init(argc, argv, "lucia_sim_2014_benchmark_node_scp");
   ros::Time::init();

   ros::NodeHandle n;
   ros::Subscriber sub = n.subscribe("map", 10000, mapCallback);
   
   bool_panels[0] = true;
   bool_panels[1] = true;
   bool_panels[2] = true;
   bool_panels[3] = true;
   
   std::vector<int> param;
   n.getParam("/Ex789/used_panels", param);
   std::cout<< "Number of Panel (Achievements): " << param.size()<<std::endl;
   std::cout << "Panels list: [ ";
   
   if(param.size()==0){
      ROS_ERROR("Missing Using Panels Parameters");
      return 0;
   }
   
   std::cout<< "List of Achievments:"<<std::endl;
   for(int iter = 0; iter < param.size(); iter++){
     std::cout<< "  -> Reach to panel: "<<param.at(iter) << std::endl;
     bool_panels[iter] = false;
   }
   
   std::cout<< "Penalization Behaviors:"<<std::endl;
   std::cout<< "  -> Robot bumps something:"<<std::endl;
   std::cout<< "  -> Excedes time: " << time_to_pb1 <<std::endl;   
   std::cout<< "  -> Excedes time: " << time_to_pb2 <<std::endl;
   
   ros::Subscriber start = n.subscribe("active_sensing", 100, startCallbak);
   
  //    ros::Subscriber bumper_1 = n.subscribe("/turtlebot_1/scan",1,laser1Callback);
  //    ros::Subscriber bumper_2 = n.subscribe("/turtlebot_2/scan",1,laser2Callback);
  //    ros::Subscriber bumper_3 = n.subscribe("/turtlebot_3/scan",1,laser3Callback);
  //    ros::Subscriber bumper_4 = n.subscribe("/turtlebot_4/scan",1,laser4Callback);
   
  ros::Subscriber bumper_1 = n.subscribe("/turtlebot_1/events/bumper",1,bumper1Callback);
  ros::Subscriber bumper_2 = n.subscribe("/turtlebot_2/events/bumper",1,bumper2Callback);
  ros::Subscriber bumper_3 = n.subscribe("/turtlebot_3/events/bumper",1,bumper3Callback);
  ros::Subscriber bumper_4 = n.subscribe("/turtlebot_4/events/bumper",1,bumper4Callback);
   
  //=====================================================

  //QR code:
  ros::ServiceClient getQR_tb1 = n.serviceClient<services::getQR>("turtlebot_1/getQR");
  ros::ServiceClient getQR_tb2 = n.serviceClient<services::getQR>("turtlebot_2/getQR");
  ros::ServiceClient getQR_tb3 = n.serviceClient<services::getQR>("turtlebot_3/getQR");
  ros::ServiceClient getQR_tb4 = n.serviceClient<services::getQR>("turtlebot_4/getQR");

  
  //send goal
  ros::ServiceClient sendGoal_tb1 = n.serviceClient<services::sendGoal>("turtlebot_1/sendGoal");
  ros::ServiceClient sendGoal_tb2 = n.serviceClient<services::sendGoal>("turtlebot_2/sendGoal");
  ros::ServiceClient sendGoal_tb3 = n.serviceClient<services::sendGoal>("turtlebot_3/sendGoal");
  ros::ServiceClient sendGoal_tb4 = n.serviceClient<services::sendGoal>("turtlebot_4/sendGoal");

  //get location:
  ros::ServiceClient getLoc_tb1 = n.serviceClient<services::getLocation>("turtlebot_1/getLocation");
  ros::ServiceClient getLoc_tb2 = n.serviceClient<services::getLocation>("turtlebot_2/getLocation");
  ros::ServiceClient getLoc_tb3 = n.serviceClient<services::getLocation>("turtlebot_3/getLocation");
  ros::ServiceClient getLoc_tb4 = n.serviceClient<services::getLocation>("turtlebot_4/getLocation");

  ros::ServiceClient getPanelLoc = n.serviceClient<services::getPanel>("/getPanel");


//QR code:
services::getQR QR_tb1,QR_tb2,QR_tb3,QR_tb4;
QR_tb1.request.read=READ;
QR_tb2.request.read=READ;
QR_tb3.request.read=READ;
QR_tb4.request.read=READ;


//=====================================================
// Benchmarking

  ros::Rate loop_rate(FREQUENCY);

ROS_INFO("BENCHMARK is waiting...");
while(ros::ok()){
  
  if (bool_start)
      break;

  ros::spinOnce();
  loop_rate.sleep();
  
}

  
  long double time_begin = ros::Time::now().toSec();
  ROS_INFO("Strat BENCHMARK: time %f", (float)time_begin);
  
   while (ros::ok())
   {

     
     //=====================================================
     // Check for PBs Regarding Time
     //=====================================================
     long double time_to_PB = ros::Time::now().toSec();
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

      // get QR codes:
      int q1= getQR_tb1.call(QR_tb1);
      int q2= getQR_tb2.call(QR_tb2);
      int q3= getQR_tb3.call(QR_tb3);
      int q4= getQR_tb4.call(QR_tb4);

      if(q1 && q2 && q3 && q4){}
      else{
	ROS_ERROR("Failed to call 'QR' services");
	break;
      }
      
      for(int iter=0; iter<param.size(); iter++ ){
	  if(bool_panels[iter]==false){
	    if(QR_tb1.response.qrcode == param.at(iter)){
	      bool_panels[iter]=true;
	      ROS_INFO("A: Reach Panel %d", param.at(iter));
	      BENCH_A+=1;
	      continue;
	    }
	    if(QR_tb2.response.qrcode == param.at(iter)){
	      bool_panels[iter]=true;
	      ROS_INFO("A: Reach Panel %d", param.at(iter));
	      BENCH_A+=1;
	      continue;
	    }
	    if(QR_tb3.response.qrcode == param.at(iter)){
	      bool_panels[iter]=true;
	      ROS_INFO("A: Reach Panel %d", param.at(iter));
	      BENCH_A+=1;
	      continue;
	    }
	    if(QR_tb4.response.qrcode == param.at(iter)){
	      bool_panels[iter]=true;
	      ROS_INFO("A: Reach Panel %d", param.at(iter));
	      BENCH_A+=1;
	      continue;
	    }
	  }
      }

      if(bool_panels[0] == true && bool_panels[1] == true && bool_panels[2] == true && bool_panels[3] == true)
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
  
 return 0;

}

//======================================================================================//
// EOF
//======================================================================================//

