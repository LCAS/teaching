#include "lucia_services.h"

//======================================================================================//
//					Main						//
//======================================================================================//
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_service");

  ros::NodeHandle nh_;

  ros::ServiceServer QRserv   = nh_.advertiseService("/getPanel", getPanel);

//=====================================================
//panels:
  nh_.getParam("/panel_serv/panel1_x1",panel1_x1);
  nh_.getParam("/panel_serv/panel1_x2",panel1_x2);
  nh_.getParam("/panel_serv/panel1_y1",panel1_y1);
  nh_.getParam("/panel_serv/panel1_y2",panel1_y2);

  nh_.getParam("/panel_serv/panel2_x1",panel2_x1);
  nh_.getParam("/panel_serv/panel2_x2",panel2_x2);
  nh_.getParam("/panel_serv/panel2_y1",panel2_y1);
  nh_.getParam("/panel_serv/panel2_y2",panel2_y2);

  nh_.getParam("/panel_serv/panel3_x1",panel3_x1);
  nh_.getParam("/panel_serv/panel3_x2",panel3_x2);
  nh_.getParam("/panel_serv/panel3_y1",panel3_y1);
  nh_.getParam("/panel_serv/panel3_y2",panel3_y2);

  nh_.getParam("/panel_serv/panel4_x1",panel4_x1);
  nh_.getParam("/panel_serv/panel4_x2",panel4_x2);
  nh_.getParam("/panel_serv/panel4_y1",panel4_y1);
  nh_.getParam("/panel_serv/panel4_y2",panel4_y2);

  nh_.getParam("/panel_serv/panel5_x1",panel5_x1);
  nh_.getParam("/panel_serv/panel5_x2",panel5_x2);
  nh_.getParam("/panel_serv/panel5_y1",panel5_y1);
  nh_.getParam("/panel_serv/panel5_y2",panel5_y2);

  nh_.getParam("/panel_serv/panel6_x1",panel6_x1);
  nh_.getParam("/panel_serv/panel6_x2",panel6_x2);
  nh_.getParam("/panel_serv/panel6_y1",panel6_y1);
  nh_.getParam("/panel_serv/panel6_y2",panel6_y2);

//=====================================================

  ros::Rate loop_rate(FREQUENCY);

   while (ros::ok())
   {
    ros::spinOnce();
    loop_rate.sleep();
   }

 return 0;
}

//================================================================================
bool getPanel(services::getPanel::Request &req, services::getPanel::Response &res)
//================================================================================
 {
 res.panel1_x1 = panel1_x1;
 res.panel1_x2 = panel1_x2;
 res.panel1_y1 = panel1_y1;
 res.panel1_y2 = panel1_y2;
 res.panel2_x1 = panel2_x1;
 res.panel2_x2 = panel2_x2;
 res.panel2_y1 = panel2_y1;
 res.panel2_y2 = panel2_y2;
 res.panel3_x1 = panel3_x1;
 res.panel3_x2 = panel3_x2;
 res.panel3_y1 = panel3_y1;
 res.panel3_y2 = panel3_y2;
 res.panel4_x1 = panel4_x1;
 res.panel4_x2 = panel4_x2;
 res.panel4_y1 = panel4_y1;
 res.panel4_y2 = panel4_y2;
 res.panel5_x1 = panel5_x1;
 res.panel5_x2 = panel5_x2;
 res.panel5_y1 = panel5_y1;
 res.panel5_y2 = panel5_y2;
 res.panel6_x1 = panel6_x1;
 res.panel6_x2 = panel6_x2;
 res.panel6_y1 = panel6_y1;
 res.panel6_y2 = panel6_y2;

  return true;
 }

//======================================================================================//
//					EOF						//
//======================================================================================//
