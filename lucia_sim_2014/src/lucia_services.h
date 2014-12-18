#include <ros/ros.h>
#include <tf/message_filter.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <bullet/LinearMath/btMatrix3x3.h>
#include <services/getQR.h>
#include <services/getStatus.h>
#include <services/sendGoal.h>
#include <services/getLocation.h>
#include <services/getPanel.h>
#include <services/rotate.h>

#define FREQUENCY	100
#define FAIL		-1
#define PIX_THRESHOLD    80000
#define PIX_MIN          10
#define PIX_MAX          100
//'PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED', 'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST'
#define SUCCEEDED        3
#define ACTIVE           1
#define PREEMPTED        2
#define ABORTED          4
#define REJECTED         5
#define PENDING          0
#define PREEMPTING       6
#define RECALLING        7
#define RECALLED         8
#define LOST             9


 bool init = true;
 double curr_yaw =0;
 double last_yaw =0;
 int    rotationAfter=0;
 std::string   robot_id;
 int goal_id;
 

 int code;   // 1:red, 2:green, 3:blue, 0:black, -1:non

 double panel1_x1, panel1_x2, panel1_y1, panel1_y2,
        panel2_x1, panel2_x2, panel2_y1, panel2_y2,
        panel3_x1, panel3_x2, panel3_y1, panel3_y2,
        panel4_x1, panel4_x2, panel4_y1, panel4_y2,
        panel5_x1, panel5_x2, panel5_y1, panel5_y2,
        panel6_x1, panel6_x2, panel6_y1, panel6_y2;

//statusOfMove < 0 <=> not moving
int statusOfMove = -1;

 using namespace std;
 typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
 geometry_msgs::PoseWithCovarianceStamped amcl_pos;
 actionlib_msgs::GoalStatusArray status;


  void imageCb(const sensor_msgs::ImageConstPtr& msg);
  bool sendQR(services::getQR::Request &req, services::getQR::Response &res);
  bool sendStatus(services::getStatus::Request &req, services::getStatus::Response &res);
  bool sendGoal(services::sendGoal::Request &req, services::sendGoal::Response &res);
  bool getLocation(services::getLocation::Request &req, services::getLocation::Response &res);
  void amclCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void goalStatus(const actionlib_msgs::GoalStatusArray& msg);
  void rotation(ros::NodeHandle nh_, ros::Publisher rotate_pub);
  bool getPanel(services::getPanel::Request &req, services::getPanel::Response &res);
  bool sendRot(services::rotate::Request &req, services::rotate::Response &res);

//======================================================================================//
//					EOF						//
//======================================================================================//

