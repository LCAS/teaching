#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <services/getQR.h>
#include <services/sendGoal.h>
#include <services/getLocation.h>
#include <services/getPanel.h>

#define FREQUENCY	10
#define READ 1

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
nav_msgs::OccupancyGrid map;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);

