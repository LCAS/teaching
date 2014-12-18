#include <ros/ros.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


#include <services/getLocation.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int64.h>

#include <kobuki_msgs/BumperEvent.h>
#include <geometry_msgs/Pose2D.h>

#define FREQUENCY	10
#define READ 1

#define BOTRADIUS   0.18
#define OFFSET  0.1
 
geometry_msgs::Pose2D tb_pose[5];

kobuki_msgs::BumperEvent bump;

ros::Publisher bump_tb[5];

typedef struct {
    double xmin, xmax, ymin, ymax;
} Wall;

typedef struct {
    int     nwalls;
    Wall   *walls;
} Environment;

static Environment   env;

//void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
int check_collisions (ros::NodeHandle n,int id);
void set_walls (void);

void tb1_pose(const geometry_msgs::Pose2D& msg);
void tb2_pose(const geometry_msgs::Pose2D& msg);
void tb3_pose(const geometry_msgs::Pose2D& msg);
void tb4_pose(const geometry_msgs::Pose2D& msg);
