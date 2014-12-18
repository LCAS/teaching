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
#include "bumpers.h"
//======================================================================================//
//					Main						//
//======================================================================================//
int main(int argc, char** argv)
{

   ros::init(argc, argv, "bumbers_node");
   ros::NodeHandle n;

//=====================================================

  ros::Subscriber sub1 = n.subscribe("/turtlebot_1/gazebo/pose", 100, tb1_pose);
  ros::Subscriber sub2 = n.subscribe("/turtlebot_2/gazebo/pose", 100, tb2_pose);
  ros::Subscriber sub3 = n.subscribe("/turtlebot_3/gazebo/pose", 100, tb3_pose);
  ros::Subscriber sub4 = n.subscribe("/turtlebot_4/gazebo/pose", 100, tb4_pose);

//get panle location:
  ros::ServiceClient getPanelLoc = n.serviceClient<services::getPanel>("/getPanel");

   bump_tb[1] = n.advertise<kobuki_msgs::BumperEvent>("/turtlebot_1/events/bumper", 100);
   bump_tb[2] = n.advertise<kobuki_msgs::BumperEvent>("/turtlebot_2/events/bumper", 100);
   bump_tb[3] = n.advertise<kobuki_msgs::BumperEvent>("/turtlebot_3/events/bumper", 100);
   bump_tb[4] = n.advertise<kobuki_msgs::BumperEvent>("/turtlebot_4/events/bumper", 100);


//=====================================================
// Panel locations:
//lucia_sim_2014::getPanel Panel;
//Panel.request.read=READ;

//=====================================================

set_walls ();
  ros::Rate loop_rate(FREQUENCY);


   while (ros::ok())
   {

//=====================================================


//panel locatiolns:
//int pnl = getPanelLoc.call(Panel);
/*
if (pnl)
ROS_INFO("PANEL 1: [%f][%f][%f][%f]",
                          Panel.response.panel1_x1,Panel.response.panel1_y1,
                          Panel.response.panel1_x2,Panel.response.panel1_y2);
*/
//=====================================================


for(int i=1;i<=4;i++)
{
check_collisions (n,i);

}

    ros::spinOnce();
    loop_rate.sleep();
   }

 return 0;

}

//=================================================
int check_collisions (ros::NodeHandle n,int id)
//=================================================
{

//ROS_INFO("id==%lf  %lf   %lf\n",tb_pose[1].x,tb_pose[1].y,tb_pose[1].theta);
bump.bumper = kobuki_msgs::BumperEvent::LEFT;
bump.state = kobuki_msgs::BumperEvent::PRESSED;

//==========================
//boundries
#define XMIN -3.13000032086
#define XMAX  3.54612345061
#define YMIN -2.86996187667
#define YMAX  1.48429727858

  if ((tb_pose[id].x < XMIN) || (tb_pose[id].x > XMAX)) bump_tb[id].publish(bump);
  if ((tb_pose[id].y < YMIN) || (tb_pose[id].y > YMAX)) bump_tb[id].publish(bump);
//==========================
//internel walls, panels and obstacles:


  Wall *w;
  for (int i=0; i<env.nwalls; i++)
  {
    w = &(env.walls[i]);
    if ((tb_pose[id].x > (w->xmin - OFFSET)) &&
	(tb_pose[id].x < (w->xmax + OFFSET)) &&
	(tb_pose[id].y > (w->ymin - OFFSET)) &&
	(tb_pose[id].y < (w->ymax + OFFSET)))
    {
     bump_tb[id].publish(bump);
    }
  }

  for (int r_no=0;r_no<=4;r_no++)
   {
   if(id != r_no)
     {
     if( sqrt(pow((tb_pose[id].x - tb_pose[r_no].x),2) + 
         pow((tb_pose[id].y - tb_pose[r_no].y),2)) <= 
         BOTRADIUS*2)
         bump_tb[id].publish(bump);
     }
   }
}

//=================================================
void set_walls (void)
//=================================================
{

env.nwalls = 10;
env.walls = (Wall*)calloc(env.nwalls, sizeof(Wall));

//box
  env.walls[0].xmin = -3.37196536483;
  env.walls[0].xmax = -2.85821156674;
  env.walls[0].ymin = -1.72500585019;
  env.walls[0].ymax = -0.849856112543;

//inner1
  env.walls[1].xmin = -1.24584571279;
  env.walls[1].xmax = -0.921083392976;
  env.walls[1].ymin = -2.87497034165;
  env.walls[1].ymax = -1.70351858025;

//inner2
  env.walls[2].xmin = 0.949558821893;
  env.walls[2].xmax = 1.36027947064;
  env.walls[2].ymin = -2.87477212164;
  env.walls[2].ymax = -1.6968691993;

//inner3
  env.walls[3].xmin = 1.16738511667;
  env.walls[3].ymin = 0.280180916434;
  env.walls[3].xmax = 2.81422810367;
  env.walls[3].ymax = 0.521559329715;

//inner4
  env.walls[4].xmin = 1.16738511667;
  env.walls[4].ymin = 0.280180916434;
  env.walls[4].xmax = 1.58436830795;
  env.walls[4].ymax = 1.68164550073;

//inner5
  env.walls[5].xmin = -2.04899769595;
  env.walls[5].ymin = 0.239863539424;
  env.walls[5].xmax = -1.61482344847;
  env.walls[5].ymax = 1.6623469325;

//inner6
  env.walls[6].xmin = -3.37308221324;
  env.walls[6].ymin = -1.73143898386;
  env.walls[6].xmax = -2.16869838201;
  env.walls[6].ymax = -1.36799827379;

//cylinder
  env.walls[7].xmin = 0.841030593916;
  env.walls[7].ymin = 0.8221837993;
  env.walls[7].xmax = 1.37873155049;
  env.walls[7].ymax = 1.29673910389;

}

//==================================================
void tb1_pose(const geometry_msgs::Pose2D& msg)
//==================================================
 {
  tb_pose[1]=msg;
 }

//==================================================
void tb2_pose(const geometry_msgs::Pose2D& msg)
//==================================================
 {
  tb_pose[2]=msg;
 }

//==================================================
void tb3_pose(const geometry_msgs::Pose2D& msg)
//==================================================
 {
  tb_pose[3]=msg;
 }

//==================================================
void tb4_pose(const geometry_msgs::Pose2D& msg)
//==================================================
 {
  tb_pose[4]=msg;
 }

//======================================================================================//
//					EOF						//
//======================================================================================//











