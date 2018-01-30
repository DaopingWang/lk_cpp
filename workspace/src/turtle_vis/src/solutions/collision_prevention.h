#ifndef COLLISION_PREVENTION_H
#define COLLISION_PREVENTION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <turtle_vis/myClass/TurtleClass.h>
#include <turtle_vis/call_driver.h>
#include "turtle_vis/pose_transfer.h"
#include "turtle_vis/pg_info_transfer.h"
#include <std_srvs/Empty.h>


enum Decision {
    F,
    L,
    R,
    AGGRESIVE_L,
    AGGRESIVE_R,
    STOP
};

enum Status {
    IDLE,
    GOTOCONE,
    GOTOGOAL,
    EXITGOAL,
    FINISHED,
    OUTBOUND
};

struct Reservoir{
    float max_range;
    float avg_range;
    int max_range_index;
};

struct Range{
    float range;
    int index;
};

struct Odom{
    float x;
    float y;
    float yaw;
};


class collision_prevention
{
public:
    collision_prevention();

    const static bool ROS_INFO_ON = false;
    const static double MIN_SCAN_ANGLE_RAD = -40.0/180*M_PI;
    const static double MAX_SCAN_ANGLE_RAD = 40.0/180*M_PI;
    const static double MIN_FRONT_ANGLE_RAD = -12.0/180*M_PI;
    const static double MAX_FRONT_ANGLE_RAD = 12.0/180*M_PI;
    const static float MIN_PROXIMITY_RANGE_M = 0.5;
    const static float SIDE_AREA_ANGLE_RAD = 10.0/180*M_PI;
    const static float ANGLE_MIN = -3.12413907051;
    const static float ANGLE_INCREMENT = 0.0174532923847;

    Status driver_status;
    Status previous_status;
    bool approaching;
    bool pg_info_set;
    bool fronted;

    float A;
    float B;
    float odom_x_min_outbound;
    float odom_x_max_outbound;
    float odom_y_outbound;

    float desired_x;
    float desired_y;

    geometry_msgs::Twist getVelMsg(const float &lin_x,
                                   const float &ang_z);
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    bool driveSrvCallback(turtle_vis::call_driver::Request &req,
                          turtle_vis::call_driver::Response &res);
    bool pgInfoCallback(turtle_vis::pg_info_transfer::Request &req,
                        turtle_vis::pg_info_transfer::Response &res);
    bool pauseCallback(std_srvs::Empty::Request &req,
                       std_srvs::Empty::Response &res);
    void poseReached();

private:
    float mid_index;

    int right_index;
    int left_index;
    int right_front_index;
    int left_front_index;
    int lL_index;
    int rL_index;
    int lR_index;
    int rR_index;

    bool makingDecision;
    bool movingForward;

    double forward_speed_mps;
    float correct_theta_rad;
    float ang_z;
    float dodge_ang_z;

    float navi_linx;
    float navi_angz;

    Decision decision;
    Decision previous_decision;

    void switchMovingDirection();
    void makeDecision(const Reservoir &left_rsv,
                      const Reservoir &right_rsv,
                      const Range &closest_front_range,
                      const Range &closest_range,
                      const int &mid_index);
    Range getClosestRange(const sensor_msgs::LaserScan::ConstPtr& scan,
                          const int &right_index,
                          const int &left_index);
    Reservoir getReservoir(const sensor_msgs::LaserScan::ConstPtr& scan,
                           const int &right_index,
                           const int &left_index);
};

#endif // COLLISION_PREVENTION_H
