/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c)  2013, Örebro University, Sweden
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.

*Authors: Ali Abdul Khaliq on 12/11/2014
*********************************************************************/

#include "lucia_sim_2014.h"

//======================================================================================//
//					Main						//
//======================================================================================//
int main(int argc, char** argv)
{

   ros::init(argc, argv, "lucia_sim_2014");
   ros::NodeHandle n;
   ros::Subscriber sub = n.subscribe("map", 10000, mapCallback);

//=====================================================

//QR code:
  ros::ServiceClient getQR_tb1 = n.serviceClient<services::getQR>("turtlebot_1/getQR");
  ros::ServiceClient getQR_tb2 = n.serviceClient<services::getQR>("turtlebot_2/getQR");
  ros::ServiceClient getQR_tb3 = n.serviceClient<services::getQR>("turtlebot_3/getQR");
  ros::ServiceClient getQR_tb4 = n.serviceClient<services::getQR>("turtlebot_4/getQR");

//send goal:
  ros::ServiceClient sendGoal_tb1 = n.serviceClient<services::sendGoal>("turtlebot_1/sendGoal");
  ros::ServiceClient sendGoal_tb2 = n.serviceClient<services::sendGoal>("turtlebot_2/sendGoal");
  ros::ServiceClient sendGoal_tb3 = n.serviceClient<services::sendGoal>("turtlebot_3/sendGoal");
  ros::ServiceClient sendGoal_tb4 = n.serviceClient<services::sendGoal>("turtlebot_4/sendGoal");

//get location:
  ros::ServiceClient getLoc_tb1 = n.serviceClient<services::getLocation>("turtlebot_1/getLocation");
  ros::ServiceClient getLoc_tb2 = n.serviceClient<services::getLocation>("turtlebot_2/getLocation");
  ros::ServiceClient getLoc_tb3 = n.serviceClient<services::getLocation>("turtlebot_3/getLocation");
  ros::ServiceClient getLoc_tb4 = n.serviceClient<services::getLocation>("turtlebot_4/getLocation");

//get panle location:
  ros::ServiceClient getPanelLoc = n.serviceClient<services::getPanel>("/getPanel");

//=====================================================

//Goal positions:
services::sendGoal goal_tb1,goal_tb2,goal_tb3,goal_tb4;
goal_tb1.request.x =7.4;
goal_tb1.request.y = 7.4;
goal_tb1.request.theta = 3.14;
goal_tb1.request.rotationAfter = 1;

goal_tb2.request.x =-7.4;
goal_tb2.request.y = -7.4;
goal_tb2.request.theta = 2.0;
goal_tb2.request.rotationAfter = 1;

goal_tb3.request.x =7.4;
goal_tb3.request.y = -7.4;
goal_tb3.request.theta = 2.0;
goal_tb3.request.rotationAfter = 1;

goal_tb4.request.x =-7.4;
goal_tb4.request.y = 7.4;
goal_tb4.request.theta = 1.0;
goal_tb4.request.rotationAfter = 1;

//=====================================================

//QR code:
services::getQR QR_tb1,QR_tb2,QR_tb3,QR_tb4;
QR_tb1.request.read=READ;
QR_tb2.request.read=READ;
QR_tb3.request.read=READ;
QR_tb4.request.read=READ;

//=====================================================

// Robot locations:
services::getLocation Loc_tb1,Loc_tb2,Loc_tb3,Loc_tb4;
Loc_tb1.request.read=READ;
Loc_tb2.request.read=READ;
Loc_tb3.request.read=READ;
Loc_tb4.request.read=READ;

//=====================================================

// Panel locations:
services::getPanel Panel;
Panel.request.read=READ;

//=====================================================

//Sending turtlebots to Goals:
int g1,g2,g3,g4;

if(!goal_tb1.response.result) {g1 = sendGoal_tb1.call(goal_tb1);}
if(!goal_tb2.response.result) {g2 = sendGoal_tb2.call(goal_tb2);}
if(!goal_tb3.response.result) {g3 = sendGoal_tb3.call(goal_tb3);}
if(!goal_tb4.response.result) {g4 = sendGoal_tb4.call(goal_tb4);}

if(g1 && g2)
    ROS_INFO("going to goals");
  else
    ROS_ERROR("Failed to call 'Goal' services");

//=====================================================

  ros::Rate loop_rate(FREQUENCY);

   while (ros::ok())
   {

//=====================================================

// get QR codes:
int q1= getQR_tb1.call(QR_tb1);
int q2= getQR_tb2.call(QR_tb2);
int q3= getQR_tb3.call(QR_tb3);
int q4= getQR_tb4.call(QR_tb4);

if(q1 && q2 && q3 && q4)
 ROS_INFO("QR: [%d]  [%d] [%d] [%d]", QR_tb1.response.qrcode, QR_tb2.response.qrcode,
                                      QR_tb3.response.qrcode, QR_tb4.response.qrcode);
  //ROS_INFO("QR: [%d]", QR_tb1.response.qrcode);
else
    ROS_ERROR("Failed to call 'QR' services");

//=====================================================

//get locatiolns:
int l1 = getLoc_tb1.call(Loc_tb1);
int l2 = getLoc_tb2.call(Loc_tb2);
int l3 = getLoc_tb3.call(Loc_tb3);
int l4 = getLoc_tb4.call(Loc_tb4);
/*
if (l1 && l2 && l3 && l4)
    ROS_INFO("LOC: [%f][%f][%f] - [%f][%f][%f] - [%f][%f][%f] - [%f][%f][%f]",
                          Loc_tb1.response.x,Loc_tb1.response.y,Loc_tb1.response.theta,
                          Loc_tb2.response.x,Loc_tb2.response.y,Loc_tb2.response.theta,
                          Loc_tb3.response.x,Loc_tb3.response.y,Loc_tb3.response.theta,
                          Loc_tb4.response.x,Loc_tb4.response.y,Loc_tb4.response.theta);
  else
    ROS_ERROR("Failed to call 'location' services");
*/
//=====================================================

//print map data:
/*    if(map.data.size()>0)
        {
       ROS_INFO("Map info: width[%d], height[%d], resolution[%f]", map.info.width,
                                                                   map.info.height,
                                                                   map.info.resolution); 
}*/

//=====================================================

//panel locatiolns:
int pnl = getPanelLoc.call(Panel);

if (pnl)
ROS_INFO("PANEL 1: [%f][%f][%f][%f]",
                          Panel.response.panel1_x1,Panel.response.panel1_y1,
                          Panel.response.panel1_x2,Panel.response.panel1_y2);

//=====================================================

    ros::spinOnce();
    loop_rate.sleep();
   }

 return 0;

}

//============================================================
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
//============================================================
{
  map.header = msg->header;
  map.info = msg->info;
  map.data = msg->data;
}


//======================================================================================//
//					EOF						//
//======================================================================================//

