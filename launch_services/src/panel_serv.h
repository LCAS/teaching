#include <ros/ros.h>
#include <services/getPanel.h>

#define FREQUENCY	100




 double panel1_x1, panel1_x2, panel1_y1, panel1_y2,
        panel2_x1, panel2_x2, panel2_y1, panel2_y2,
        panel3_x1, panel3_x2, panel3_y1, panel3_y2,
        panel4_x1, panel4_x2, panel4_y1, panel4_y2,
        panel5_x1, panel5_x2, panel5_y1, panel5_y2,
        panel6_x1, panel6_x2, panel6_y1, panel6_y2;


 using namespace std;


 bool getPanel(services::getPanel::Request &req, services::getPanel::Response &res);

//======================================================================================//
//					EOF						//
//======================================================================================//

