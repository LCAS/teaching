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

#include "lucia_services.h"

//======================================================================================//
//					Main						//
//======================================================================================//
int main(int argc, char** argv)
{

  ros::init(argc, argv, "lucia_services");

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_;

  image_sub_                    = it_.subscribe("camera/rgb/image_raw", 1, &imageCb);
  ros::ServiceServer Statusserv = nh_.advertiseService("getStatus", sendStatus);
  ros::ServiceServer QRserv     = nh_.advertiseService("getQR", sendQR);
  ros::ServiceServer Goalserv   = nh_.advertiseService("sendGoal", sendGoal);
  ros::ServiceServer Rotateserv = nh_.advertiseService("rotate", sendRot);
  ros::ServiceServer Locserv    = nh_.advertiseService("getLocation", getLocation);
  ros::Subscriber    amcl_sub   = nh_.subscribe("amcl_pose", 10, amclCallback);
  ros::Subscriber sub = nh_.subscribe("move_base/status", 10, goalStatus);
  ros::Publisher rotate_pub = nh_.advertise<geometry_msgs::Twist>("commands/velocity", 100);

  ros::Rate loop_rate(FREQUENCY);

  nh_.getParam("lucia_services/robot_id",robot_id);

   while (ros::ok())
   {

if(rotationAfter) //this veriable is handled by sendGoal and rotate services
    rotation(nh_,rotate_pub);


    ros::spinOnce();
    loop_rate.sleep();
   }

 return 0;
}

//==================================================
void imageCb(const sensor_msgs::ImageConstPtr& msg)
//==================================================
 {
  cv_bridge::CvImagePtr cv_ptr;
  try
   {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
   }
  catch (cv_bridge::Exception& e)
   {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
   }

  cv::Vec3b intensity;
  int red=0;
  int green=0;
  int blue=0;
  int black=0;
  int purple=0;
  int yellow=0;

  for(int i=0;i<(cv_ptr->image.rows);i++)
     {
     for(int j=0;j<(cv_ptr->image.cols);j++)
       {
   intensity = cv_ptr->image.at<cv::Vec3b>(i, j);
   if(intensity.val[0]>PIX_MAX && intensity.val[1]<PIX_MIN && intensity.val[2]<PIX_MIN) {red++;  }
   if(intensity.val[0]<PIX_MIN && intensity.val[1]>PIX_MAX && intensity.val[2]<PIX_MIN) {green++;}
   if(intensity.val[0]<PIX_MIN && intensity.val[1]<PIX_MIN && intensity.val[2]>PIX_MAX) {blue++; }
   if(intensity.val[0]<PIX_MIN && intensity.val[1]<PIX_MIN && intensity.val[2]<PIX_MIN) {black++;}

   if(intensity.val[0]>PIX_MAX && intensity.val[1]<PIX_MIN && intensity.val[2]>PIX_MAX) {purple++;}

   if(intensity.val[0]>PIX_MAX && intensity.val[1]>PIX_MAX && intensity.val[2]<PIX_MIN) {yellow++;}
       }
      }

  code=-1;

  if(red   > PIX_THRESHOLD) {code=1;}
  if(green > PIX_THRESHOLD) {code=2;}
  if(blue  > PIX_THRESHOLD) {code=3;}
  if(black > PIX_THRESHOLD) {code=4;}
  if(purple > PIX_THRESHOLD) {code=5;}
  if(yellow > PIX_THRESHOLD) {code=6;}

 red = green = blue = black = purple = yellow = 0;

  }

//========================================================================
bool sendQR(services::getQR::Request &req, services::getQR::Response &res)
//========================================================================
 {
  res.qrcode= code;
  return true;
 }

//========================================================================
bool sendStatus(services::getStatus::Request &req, services::getStatus::Response &res)
//========================================================================
 {
  res.status = statusOfMove;
  return true;
 }

//========================================================================
bool sendRot(services::rotate::Request &req, services::rotate::Response &res)
//========================================================================
 {
  rotationAfter = req.rotate;
    last_yaw=0;
    curr_yaw=0;
    init=true;

  return true;
 }

//================================================================================
bool sendGoal(services::sendGoal::Request &req, services::sendGoal::Response &res)
//================================================================================
 {
   //  MoveBaseClient ac("move_base", true);
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  statusOfMove = 1;
  rotationAfter = req.rotationAfter;
  while (!ac.waitForServer(ros::Duration(2.0)))
    {
    ROS_INFO("Waiting for the move_base action server to come up");
    }

  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id  = "/map"; 
  goal.target_pose.header.stamp =  ros::Time::now();
  geometry_msgs::Point goalPoint;

  goalPoint.x = req.x;
  goalPoint.y = req.y;
  goal.target_pose.pose.position = goalPoint;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(req.theta);//orientationPoint;

  ac.sendGoal(goal);

  res.result = 1;
  return true;
 }

//=========================================================================================
bool getLocation(services::getLocation::Request &req, services::getLocation::Response &res)
//=========================================================================================
 {
  btScalar roll, pitch, yaw;
  btQuaternion q(amcl_pos.pose.pose.orientation.x,
                 amcl_pos.pose.pose.orientation.y,
                 amcl_pos.pose.pose.orientation.z,
                 amcl_pos.pose.pose.orientation.w);

  btMatrix3x3(q).getEulerYPR(yaw, pitch,roll );

  res.id = 1;//robot_id;
  res.x= amcl_pos.pose.pose.position.x;
  res.y= amcl_pos.pose.pose.position.y;
  res.z= amcl_pos.pose.pose.position.z;
  res.theta = yaw;

  return true;
 }

//====================================================================
void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
//====================================================================
 {
  amcl_pos.pose.pose.position.x = msg.pose.pose.position.x;
  amcl_pos.pose.pose.position.y = msg.pose.pose.position.y;
  amcl_pos.pose.pose.position.z = msg.pose.pose.position.z;

  amcl_pos.pose.pose.orientation.x= msg.pose.pose.orientation.x;
  amcl_pos.pose.pose.orientation.y= msg.pose.pose.orientation.y;
  amcl_pos.pose.pose.orientation.z= msg.pose.pose.orientation.z;
  amcl_pos.pose.pose.orientation.w= msg.pose.pose.orientation.w;
 }

//==================================================
void goalStatus(const actionlib_msgs::GoalStatusArray& msg)
//==================================================
 {
  status.status_list= msg.status_list; 
 }

//==================================================
void rotation(ros::NodeHandle nh_, ros::Publisher rotate_pub)
//==================================================
 {

//static double curr_yaw =0;
//static double last_yaw =0;

  geometry_msgs::Twist pose;

  pose.linear.x = 0.0;
  pose.linear.y = 0.0;
  pose.linear.z = 0.0;
  pose.angular.x = 0.0;
  pose.angular.y = 0.0;
  pose.angular.z = 0.5;

  btScalar roll, pitch, yaw;

  btQuaternion q(amcl_pos.pose.pose.orientation.x,
                 amcl_pos.pose.pose.orientation.y,
                 amcl_pos.pose.pose.orientation.z,
                 amcl_pos.pose.pose.orientation.w);

  btMatrix3x3(q).getEulerYPR(yaw, pitch,roll );

  if(!status.status_list.empty()           &&
     (int)status.status_list[0].status==SUCCEEDED  &&
     code<0 && curr_yaw<=(2*M_PI))
     {
     if(init)
       {
       last_yaw=yaw;
       init=false;
       }
     else
       {
       curr_yaw+= abs(abs(yaw)- abs(last_yaw)); 
       last_yaw = yaw;
       }
    rotate_pub.publish(pose);
     }

  //Set status to report if asked thru getStatus service
  if (status.status_list.empty()) {
    statusOfMove = -1;
    std::cout << robot_id <<" ++ empty (-1)" << std::endl;
  }
  else if ((int)status.status_list[0].status==SUCCEEDED && code<0 && curr_yaw<=(2*M_PI)) {
    statusOfMove = 1;
    std::cout << robot_id <<" ++ succed but still turning (1)" << std::endl;
  }
  else if ((int)status.status_list[0].status==SUCCEEDED && (code>0 || curr_yaw > (2*M_PI))) {
    statusOfMove = -1;
    curr_yaw = 2.5*M_PI;
    std::cout << robot_id <<" ++ succed and finished turning or didn't need to turn (-1)" << std::endl;
  }
  else if ((int)status.status_list[0].status==ABORTED) {
    statusOfMove = -1;
    std::cout << robot_id <<" ++ aborted (-1)" << std::endl;
  }
  else if ((int)status.status_list[0].status==PREEMPTED) {
    statusOfMove = -1;
    std::cout << robot_id <<" ++ pre-empted (-1)" << std::endl;
  }
  else if ((int)status.status_list[0].status==REJECTED) {
    statusOfMove = -1;
    std::cout << robot_id <<" ++ rejected (-1)" << std::endl;
  }
  else if ((int)status.status_list[0].status==LOST) {
    statusOfMove = -1;
    std::cout << robot_id <<" ++ lost (-1)" << std::endl;
  }
  else if ((int)status.status_list[0].status==RECALLED) {
    statusOfMove = -1;
    std::cout << robot_id <<" ++ recalled (-1)" << std::endl;
  }
  else if ((int)status.status_list[0].status==ACTIVE) {
    statusOfMove = 1;
    last_yaw=0;
    curr_yaw=0;
    init=true;
    std::cout << robot_id <<" ++ active (1)" << std::endl;
    }
  else {
    statusOfMove = -1;
    std::cout << robot_id <<" ++ 'PENDING' 'PREEMPTING', 'RECALLING'" << std::endl;
    }
}

//======================================================================================//
//					EOF						//
//======================================================================================//
