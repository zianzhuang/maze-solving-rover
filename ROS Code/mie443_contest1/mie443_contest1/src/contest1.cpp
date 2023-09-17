/*
    ___________________________________
    |         DISTANCE TABLE          |
    |       | OBS AVOID < WALL FOLLOW |
    | FRONT |   0.55    <     0.6     |
    | SIDE  |   0.65    <  0.6 - 0.7  |
    +++++++++++++++++++++++++++++++++++
    ________________________________________________________________________________________________
    |                             TIME TABLE (360 ROTATE: 15 SEC)                                  |
    | BUMPER RESPONSE < IDLE TIME < ABS AVOID < START POINT CHECK < PERIOID SPIN < WHILE TRAP      |
    |    3 - 4 ONCE   <    12     <     15    <     15 - 20       <   25 - 35    <   120 - 180     |
    |                     3 * 4       12 + 3                                                       |
    ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

// LIBRARIES =======================================================================================================
#include <ros/console.h>
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <kobuki_msgs/BumperEvent.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <stdio.h>
#include <cmath>
#include <chrono>
#include <vector>
#define N_BUMPER (3)
#define RAD2DEG(rad) ((rad)*180. / M_PI)
#define DEG2RAD(deg) ((deg)*M_PI / 180.)

// FUNCTION PROTOTYPES =============================================================================================
// CALLBACKS
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg);
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);
// BASICS
float calcDistance(float x1, float y1, float x2, float y2);
void run(ros::Publisher vel_pub, geometry_msgs::Twist vel);
void stop(ros::Publisher vel_pub, geometry_msgs::Twist vel);
bool is_idle();
void rotate_deg(ros::Publisher vel_pub, geometry_msgs::Twist vel, float angle);
void face_max_dist(ros::Publisher vel_pub, geometry_msgs::Twist vel);
void take_action(ros::Publisher vel_pub, geometry_msgs::Twist vel);
// ALGORITHM
void avoid_obstacle(ros::Publisher vel_pub, geometry_msgs::Twist vel);
void wall_follow_R(ros::Publisher vel_pub, geometry_msgs::Twist vel);
void wall_follow_L(ros::Publisher vel_pub, geometry_msgs::Twist vel);
// STRATEGY
void wall_finder(ros::Publisher vel_pub, geometry_msgs::Twist vel);
void wall_explorer(ros::Publisher vel_pub, geometry_msgs::Twist vel);

// GLOBAL VARIABLES ================================================================================================
// SPEED VARIABLES [angular_max = +/- M_PI/6 | linear_max = +/- 0.25 (navigating) | +/- 0.1 (close to obstacle)]
float linear = 0.0, angular = 0.0;
// ODOM VARIABLES
float posX = 0.0, posY = 0.0, posX_prev = 0.0, posY_prev = 0.0;
float yaw = 0.0, yaw_deg = 0.0, max_dist_yaw_deg = 0.0;
float rotate_precision = 1.0; // DEFAULT 1.0 DEG
std::pair<float, float> wall_start_pos;
std::vector<std::pair<float, float>> path;
// BUMPER VARIABLES
uint8_t bumper[3] = {kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED, kobuki_msgs::BumperEvent::RELEASED};
bool any_bumper_pressed = false;
// LASER VARIABLES
float maxLaserDist = 0.0;
float maxLaserDist_temp = 0.0;
float minLaserDist = std::numeric_limits<float>::infinity();
float minRightDist = std::numeric_limits<float>::infinity();
//float minRightDist_1 = std::numeric_limits<float>::infinity();
//float minRightDist_2 = std::numeric_limits<float>::infinity();
float minFrontDist = std::numeric_limits<float>::infinity();
float minFrontLeftDist = std::numeric_limits<float>::infinity();
float minFrontRightDist = std::numeric_limits<float>::infinity();
//float minFrontDist_1 = std::numeric_limits<float>::infinity();
//float minFrontDist_2 = std::numeric_limits<float>::infinity();
float minLeftDist = std::numeric_limits<float>::infinity();
//float minLeftDist_1 = std::numeric_limits<float>::infinity();
//float minLeftDist_2 = std::numeric_limits<float>::infinity();
//float avgRightDist = 0.0;
//float avgRightDist_1 = 0.0;
//float avgRightDist_2 = 0.0;
//float avgFrontDist = 0.0;
//float avgFrontDist_1 = 0.0;
//float avgFrontDist_2 = 0.0;
//float avgLeftDist = 0.0;
//float avgLeftDist_1 = 0.0;
//float avgLeftDist_2 = 0.0;
int32_t nLasers = 0, desiredNLasers = 0, desiredAngle = 1; // DEFAULT 1 DEG FoV
// TIME VARIABLES
std::chrono::time_point<std::chrono::system_clock> start;
uint64_t secondsElapsed = 0;
uint64_t idle_checktime = 0;
// STATE VARIABLES
bool fault = false;
bool while_trap = false;
bool forbid_spin = false;
bool right_rule = true;
bool wall_following = false;

// SUBSCRIBER CALLBACK =============================================================================================
void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr &msg)
{ // 0 - LEFT; 1 - CENTER; 2 - RIGHT
    //uint8_t leftState = bumper[kobuki_msgs::BumperEvent::LEFT];
    //uint8_t centerState = bumper[kobuki_msgs::BumperEvent::CENTER];
    //uint8_t rightState = bumper[kobuki_msgs::BumperEvent::RIGHT];
    //ROS_INFO("Left: %u Center: %u Right: %u", leftState, centerState, rightState);
    any_bumper_pressed = false;
    bumper[msg->bumper] = msg->state;
    for (uint32_t b_idx = 0; b_idx < N_BUMPER; ++b_idx)
    {
        any_bumper_pressed |= (bumper[b_idx] == kobuki_msgs::BumperEvent::PRESSED);
    }
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    maxLaserDist = 0.0;
    minLaserDist = std::numeric_limits<float>::infinity();
    minRightDist = std::numeric_limits<float>::infinity();
    //minRightDist_1 = std::numeric_limits<float>::infinity();
    //minRightDist_2 = std::numeric_limits<float>::infinity();
    minFrontDist = std::numeric_limits<float>::infinity();
    minFrontLeftDist = std::numeric_limits<float>::infinity();
    minFrontRightDist = std::numeric_limits<float>::infinity();
    //minFrontDist_1 = std::numeric_limits<float>::infinity();
    //minFrontDist_2 = std::numeric_limits<float>::infinity();
    minLeftDist = std::numeric_limits<float>::infinity();
    //minLeftDist_1 = std::numeric_limits<float>::infinity();
    //minLeftDist_2 = std::numeric_limits<float>::infinity();
    //float totalLeftDist = 0.0, totalFrontDist = 0.0, totalRightDist = 0.0;
    //float LeftDist_count = 0.0, FrontDist_count = 0.0, RightDist_count = 0.0;
    //float totalLeftDist_1 = 0.0, totalFrontDist_1 = 0.0, totalRightDist_1 = 0.0;
    //float LeftDist_count_1 = 0.0, FrontDist_count_1 = 0.0, RightDist_count_1 = 0.0;
    //float totalLeftDist_2 = 0.0, totalFrontDist_2 = 0.0, totalRightDist_2 = 0.0;
    //float LeftDist_count_2 = 0.0, FrontDist_count_2 = 0.0, RightDist_count_2 = 0.0;
    nLasers = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    desiredNLasers = DEG2RAD(desiredAngle) / msg->angle_increment;
    if (DEG2RAD(desiredAngle) < msg->angle_max && -DEG2RAD(desiredAngle) > msg->angle_min)
    { // DESIRED FoV < SENSOR FoV
        for (uint32_t laser_idx = nLasers / 2 - desiredNLasers; laser_idx < nLasers / 2 + desiredNLasers; ++laser_idx)
        { // FIND MIN & MAX IN DESIRED FoV
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            if (msg->ranges[laser_idx] != std::numeric_limits<float>::infinity())
            { // EXCLUDE INF
                maxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
            }
        }
    }
    else
    { // SENSOR FoV < DESIRED FoV
        for (uint32_t laser_idx = 0; laser_idx < nLasers; ++laser_idx)
        { // FIND MIN & MAX IN SENSOR FoV
            minLaserDist = std::min(minLaserDist, msg->ranges[laser_idx]);
            if (msg->ranges[laser_idx] != std::numeric_limits<float>::infinity())
            { // EXCLUDE INF
                maxLaserDist = std::max(maxLaserDist, msg->ranges[laser_idx]);
            }
        }
    }
    for (uint32_t laser_idx = 501; laser_idx <= 639; ++laser_idx)
    { // 139
        minLeftDist = std::min(minLeftDist, msg->ranges[laser_idx]);
        //if (!std::isnan(msg->ranges[laser_idx]))
        //{
        //    totalLeftDist = totalLeftDist + msg->ranges[laser_idx];
        //    LeftDist_count = LeftDist_count + 1.0;
        //}
    }
    for (uint32_t laser_idx = 139; laser_idx <= 500; ++laser_idx)
    { // 362
        minFrontDist = std::min(minFrontDist, msg->ranges[laser_idx]);
        //if (!std::isnan(msg->ranges[laser_idx]))
        //{
        //    totalFrontDist = totalFrontDist + msg->ranges[laser_idx];
        //    FrontDist_count = FrontDist_count + 1.0;
        //}
    }
    for (uint32_t laser_idx = 320; laser_idx <= 500; ++laser_idx)
    { // 181
        minFrontLeftDist = std::min(minFrontLeftDist, msg->ranges[laser_idx]);
    }
    for (uint32_t laser_idx = 139; laser_idx <= 319; ++laser_idx)
    { // 181
        minFrontRightDist = std::min(minFrontRightDist, msg->ranges[laser_idx]);
    }
    for (uint32_t laser_idx = 0; laser_idx <= 138; ++laser_idx)
    { // 139
        minRightDist = std::min(minRightDist, msg->ranges[laser_idx]);
        //if (!std::isnan(msg->ranges[laser_idx]))
        //{
        //    totalRightDist = totalRightDist + msg->ranges[laser_idx];
        //    RightDist_count = RightDist_count + 1.0;
        //}
    }
    /*
    for (uint32_t laser_idx = 534; laser_idx <= 639; ++laser_idx)
    { // leftmost 10 deg
        minLeftDist_2 = std::min(minLeftDist_2, msg->ranges[laser_idx]);
        //if (!std::isnan(msg->ranges[laser_idx]))
        //{
        //    totalLeftDist_2 = totalLeftDist_2 + msg->ranges[laser_idx];
        //    LeftDist_count_2 = LeftDist_count_2 + 1.0;
        //}
    }
    for (uint32_t laser_idx = 427; laser_idx <= 533; ++laser_idx)
    { // left 10 deg
        minLeftDist_1 = std::min(minLeftDist_1, msg->ranges[laser_idx]);
        //if (!std::isnan(msg->ranges[laser_idx]))
        //{
        //    totalLeftDist_1 = totalLeftDist_1 + msg->ranges[laser_idx];
        //    LeftDist_count_1 = LeftDist_count_1 + 1.0;
        //}
    }
    for (uint32_t laser_idx = 320; laser_idx <= 426; ++laser_idx)
    { // frontleft 10 deg
        minFrontDist_2 = std::min(minFrontDist_2, msg->ranges[laser_idx]);
        //if (!std::isnan(msg->ranges[laser_idx]))
        //{
        //    totalFrontDist_2 = totalFrontDist_2 + msg->ranges[laser_idx];
        //    FrontDist_count_2 = FrontDist_count_2 + 1.0;
        //}
    }
    for (uint32_t laser_idx = 213; laser_idx <= 319; ++laser_idx)
    { // frontright 10 deg
        minFrontDist_1 = std::min(minFrontDist_1, msg->ranges[laser_idx]);
        //if (!std::isnan(msg->ranges[laser_idx]))
        //{
        //    totalFrontDist_1 = totalFrontDist_1 + msg->ranges[laser_idx];
        //    FrontDist_count_1 = FrontDist_count_1 + 1.0;
        //}
    }
    for (uint32_t laser_idx = 106; laser_idx <= 212; ++laser_idx)
    { // right 10 deg
        minRightDist_2 = std::min(minRightDist_2, msg->ranges[laser_idx]);
        //if (!std::isnan(msg->ranges[laser_idx]))
        //{
        //    totalRightDist_2 = totalRightDist_2 + msg->ranges[laser_idx];
        //    RightDist_count_2 = RightDist_count_2 + 1.0;
        //}
    }
    for (uint32_t laser_idx = 0; laser_idx <= 105; ++laser_idx)
    { // rightmost 10 deg
        minRightDist_1 = std::min(minRightDist_1, msg->ranges[laser_idx]);
        //if (!std::isnan(msg->ranges[laser_idx]))
        //{
        //    totalRightDist_1 = totalRightDist_1 + msg->ranges[laser_idx];
        //    RightDist_count_1 = RightDist_count_1 + 1.0;
        //}
    }
    */
    //avgLeftDist = totalLeftDist / LeftDist_count;
    //avgFrontDist = totalFrontDist / FrontDist_count;
    //avgRightDist = totalRightDist / RightDist_count;
    //ROS_INFO("totalLeftDist: %f totalFrontDist: %f totalRightDist: %f", totalLeftDist, totalFrontDist, totalRightDist);
    //ROS_INFO("avgLeftDist: %f avgFrontDist: %f avgRightDist: %f", avgLeftDist, avgFrontDist, avgRightDist);
    //ROS_INFO("minLeftDist: %f minFrontDist: %f minRightDist: %f", minLeftDist, minFrontDist, minRightDist);
    //ROS_INFO("minLaserDist: %f maxLaserDist: %f", minLaserDist, maxLaserDist);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    posX = msg->pose.pose.position.x;
    posY = msg->pose.pose.position.y;
    yaw = tf::getYaw(msg->pose.pose.orientation);
    yaw_deg = RAD2DEG(yaw);
    //ROS_INFO("Position: (%f, %f) Orientation: %f rad or %f degrees.", posX, posY, yaw, yaw_deg);
}

// FUNCTIONS VERIFIED ==============================================================================================
float calcDistance(float x1, float y1, float x2, float y2)
{
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

void run(ros::Publisher vel_pub, geometry_msgs::Twist vel)
{
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);
    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
    if (secondsElapsed > 480)
    { // SYSTEM TOTAL RUNTIME REACHED
        fault = true;
    }
}

void stop(ros::Publisher vel_pub, geometry_msgs::Twist vel)
{
    angular = 0.0;
    linear = 0.0;
    vel.angular.z = angular;
    vel.linear.x = linear;
    vel_pub.publish(vel);
    secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
    if (secondsElapsed > 480)
    { // SYSTEM TOTAL RUNTIME REACHED
        fault = true;
    }
}

bool is_idle()
{                                                                  // EVERY 12 SEC MUST MOVE AT LEAST 0.5 M
    if ((secondsElapsed - idle_checktime) >= 12)                   // (10 - 12)
    {                                                              // DEFAULT 12 SECS FOR AT MOST 2 TIMES BUMPER TRIGGERS
        if (calcDistance(posX, posY, posX_prev, posY_prev) <= 0.5) // (0.3 - 0.7)
        {                                                          // DEFAULT 0.5 M
            ROS_INFO("IDLE --> PROGRAM REBOOTED.");
            posX_prev = posX;
            posY_prev = posY;
            idle_checktime = secondsElapsed;
            return true;
        }
        else
        {
            posX_prev = posX;
            posY_prev = posY;
            idle_checktime = secondsElapsed;
            return false;
        }
    }
    else
    {
        return false;
    }
}

void rotate_deg(ros::Publisher vel_pub, geometry_msgs::Twist vel, float angle)
{ // ROTATE +CCW -CW (RANGE: 0 - 180)
    ros::spinOnce();
    float yaw_goal = yaw_deg + angle;
    if (yaw_goal < -180.0)
    { // SET YAW_GOAL BETWEEN -180 - +180
        yaw_goal = yaw_goal + 360.0;
    }
    else if (yaw_goal > 180.0)
    {
        yaw_goal = yaw_goal - 360.0;
    }
    if (angle > 0.0)
    { // SET ROTATE DIRECTION
        angular = M_PI / 6;
        linear = 0.0;
    }
    else
    {
        angular = -M_PI / 6;
        linear = 0.0;
    }
    while (rotate_precision < abs(yaw_goal - yaw_deg) && abs(yaw_goal - yaw_deg) < (360 - rotate_precision))
    { // ROTATE TILL YAW_DEG = YAW_GOAL
        run(vel_pub, vel);
        ros::spinOnce();
        if (maxLaserDist > maxLaserDist_temp)
        { // UPDATE MAX DIST YAW
            max_dist_yaw_deg = yaw_deg;
            maxLaserDist_temp = maxLaserDist;
        }
    }
    posX_prev = posX;
    posY_prev = posY;
    idle_checktime = secondsElapsed;
    stop(vel_pub, vel);
}

void face_max_dist(ros::Publisher vel_pub, geometry_msgs::Twist vel)
{
    maxLaserDist_temp = 0.0;
    rotate_deg(vel_pub, vel, -181);
    rotate_deg(vel_pub, vel, -181);
    angular = M_PI / 6;
    linear = 0.0;
    while (rotate_precision < abs(max_dist_yaw_deg - yaw_deg) && abs(max_dist_yaw_deg - yaw_deg) < (360 - rotate_precision))
    { // ROTATE TILL YAW = MAX_DIST_YAW
        run(vel_pub, vel);
        ros::spinOnce();
    }
    posX_prev = posX;
    posY_prev = posY;
    idle_checktime = secondsElapsed;
    stop(vel_pub, vel);
}

void take_action(ros::Publisher vel_pub, geometry_msgs::Twist vel)
{ // RUN WITH BUMPER PRIORITY 2 SEC BACK OFF TIME
    std::chrono::time_point<std::chrono::system_clock> bumper_pressed_start;
    bumper_pressed_start = std::chrono::system_clock::now();
    if (any_bumper_pressed)
    {
        if (bumper[0] == 1 && bumper[2] == 1)
        {                   // LR or LFR
            linear = -0.13; // DEFAULT -0.12
            angular = 0.0;
            if (wall_following)
            { // TEST VALUE
                if (right_rule)
                {
                    linear = -0.12;      // DEFAULT -0.12
                    angular = M_PI / 10; // DEFAULT PI/10
                }
                else
                {
                    linear = -0.12;       // DEFAULT -0.12
                    angular = -M_PI / 10; // DEFAULT -PI/10
                }
            }
        }
        else if (bumper[0] == 1)
        {                        // L or LF
            linear = -0.12;      // DEFAULT -0.12
            angular = -M_PI / 6; // DEFAULT -PI/6
        }
        else if (bumper[2] == 1)
        {                       // R or RF
            linear = -0.12;     // DEFAULT -0.12
            angular = M_PI / 6; // DEFAULT PI/6
        }
        else if (bumper[1] == 1)
        {                   // F
            linear = -0.13; // DEFAULT -0.12
            angular = 0.0;
            if (wall_following)
            { // TEST VALUE
                if (right_rule)
                {
                    linear = -0.12;      // DEFAULT -0.12
                    angular = M_PI / 10; // DEFAULT PI/10
                }
                else
                {
                    linear = -0.12;       // DEFAULT -0.12
                    angular = -M_PI / 10; // DEFAULT -PI/10
                }
            }
        }
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - bumper_pressed_start).count() <= 1)
        { // DEFAULT 2 SEC BACK OFF TIME
            run(vel_pub, vel);
            ros::spinOnce();
        }
    }
    else
    {
        run(vel_pub, vel);
    }
}

void avoid_obstacle(ros::Publisher vel_pub, geometry_msgs::Twist vel)
{                           // FRONTDIST 0.55; SIDEDIST 0.65
    float FrontDist = 0.55; // DEFAULT 0.55 M
    float SideDist = 0.65;  // DEFAULT 0.65 M
    float ErrDist = std::numeric_limits<float>::infinity();
    forbid_spin = false;
    ros::spinOnce();
    float Left = minLeftDist, Front = minFrontDist, Right = minRightDist; // A1: USE minDist
    //float Left = avgLeftDist, Front = avgFrontDist, Right = avgRightDist; // A2: USE avgDist
    if (Right == ErrDist && Left == ErrDist)
    { // RIGHT & LEFT TOO CLOSE
        linear = 0.0;
        angular = M_PI / 6;
        forbid_spin = true;
    }
    else if (Right == ErrDist)
    { // RIGHT TOO CLOSE
        linear = 0.0;
        angular = M_PI / 6;
        forbid_spin = true;
    }
    else if (Left == ErrDist)
    { // LEFT TOO CLOSE
        linear = 0.0;
        angular = -M_PI / 6;
        forbid_spin = true;
    }
    else if (Front == ErrDist)
    { // FRONT TOO CLOSE
        linear = 0.0;
        angular = M_PI / 6;
        forbid_spin = true;
    }
    else if (Left >= SideDist && Front >= FrontDist && Right >= SideDist)
    { // NONE
        if (Left < (2 * SideDist) || Front < (2 * FrontDist) || Right < (2 * SideDist))
        {
            linear = 0.2;
            angular = 0.0;
        }
        else
        {
            linear = 0.25;
            angular = 0.0;
        }
    }
    else if (Left < SideDist && Front > FrontDist && Right < SideDist)
    { // RL
        linear = 0.15;
        angular = 0.0;
        forbid_spin = true;
    }
    else if (Right < SideDist)
    { // RFL RF R
        linear = 0.0;
        angular = M_PI / 6;
        forbid_spin = true;
    }
    else if (Left < SideDist)
    { // FL L
        linear = 0.0;
        angular = -M_PI / 6;
        forbid_spin = true;
    }
    else if (Front < FrontDist)
    { // F
        linear = 0.0;
        angular = M_PI / 6;
        forbid_spin = true;
    }
    else
    { // UNKOWN
        linear = 0.0;
        angular = 0.0;
        forbid_spin = true;
    }
    take_action(vel_pub, vel);
}

void wall_follow_R(ros::Publisher vel_pub, geometry_msgs::Twist vel)
{                         // CRITDIST 0.6; WALLDIST 0.6 - 0.7
    float wallDist = 0.7; // DEFAULT 0.7
    float critDist = 0.6; // DEFAULT 0.6
    float ErrDist = std::numeric_limits<float>::infinity();
    forbid_spin = false;
    ros::spinOnce();
    if (minFrontRightDist <= critDist || minFrontRightDist == ErrDist)
    {
        linear = 0;
        angular = M_PI / 6;
    }
    else if ((minFrontRightDist > critDist && minRightDist < critDist) || (minFrontRightDist > critDist && minRightDist == ErrDist))
    {
        linear = 0.12;       // DEFAULT 0.12
        angular = M_PI / 14; // DEFAULT PI/14
        forbid_spin = true;
    }
    else if (minFrontRightDist > critDist && minRightDist > wallDist)
    {
        linear = 0.12;        // DEFAULT 0.12
        angular = -M_PI / 14; // DEFAULT -PI/14
        forbid_spin = true;
    }
    else if (minFrontRightDist > critDist && minRightDist <= wallDist && minRightDist >= critDist)
    {
        linear = 0.25;
        angular = 0.0;
    }
    take_action(vel_pub, vel);
}

void wall_follow_L(ros::Publisher vel_pub, geometry_msgs::Twist vel)
{                         // CRITDIST 0.6; WALLDist 0.6 - 0.7
    float wallDist = 0.7; // DEFAULT 0.7
    float critDist = 0.6; // DEFAULT 0.6
    float ErrDist = std::numeric_limits<float>::infinity();
    forbid_spin = false;
    ros::spinOnce();
    if (minFrontLeftDist <= critDist || minFrontLeftDist == ErrDist)
    {
        linear = 0;
        angular = -M_PI / 6;
    }
    else if ((minFrontLeftDist > critDist && minLeftDist < critDist) || (minFrontLeftDist > critDist && minLeftDist == ErrDist))
    {
        linear = 0.12;        // DEFAULT 0.12
        angular = -M_PI / 14; // DEFAULT -PI/14
        forbid_spin = true;
    }
    else if (minFrontLeftDist > critDist && minLeftDist > wallDist)
    {
        linear = 0.12;       // DEFAULT 0.12
        angular = M_PI / 14; // DEFAULT PI/14
        forbid_spin = true;
    }
    else if (minFrontLeftDist > critDist && minLeftDist <= wallDist && minLeftDist >= critDist)
    {
        linear = 0.25;
        angular = 0.0;
    }
    take_action(vel_pub, vel);
}

void wall_finder(ros::Publisher vel_pub, geometry_msgs::Twist vel)
{ // FRONTDIST 0.6 ; SIDEDIST 0.7 || PATH CHECK 0.8 || TURN EVERY 15 SEC || PERIODIC SPIN EVERY 40 SEC
    std::chrono::time_point<std::chrono::system_clock> finder_start;
    finder_start = std::chrono::system_clock::now();
    uint64_t finder_duration = 0;
    uint64_t finder_duration_turn = 0;
    uint64_t finder_duration_rotate = 0;
    bool wall_found = false;
    while (wall_found == false)
    {
        finder_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - finder_start).count();
        if (finder_duration > 120)
        { // CHECK IF STUCK IN WHILE DEFAULT 120 SEC
            ROS_INFO("STUCK IN FINDER --> PROGRAM REBOOTED.");
            while_trap = true;
        }
        if (is_idle() == true || while_trap == true || fault == true)
        { // CHECK PROGRAM FAULT
            fault = true;
            break;
        }
        if (minLeftDist < 0.7 || minFrontDist < 0.6 || minRightDist < 0.7) // FRONTDIST 0.6; SIDEDIST 0.7
        {                                                                  // CONDITION 1: CHECK IF CLOSE TO WALL SO WALL FOLLOWING ALGO CAN APPLY
            for (auto it = path.begin(); it != path.end(); it++)
            { // CONDITION 2: CHECK IF REPEATED WALL
                if (calcDistance(posX, posY, it->first, it->second) <= 0.8)
                { // PATH CHECK RANGE DEFAULT 0.8 M
                    if ((finder_duration - finder_duration_turn) > 15)
                    { // TRIGGER TURN EVERY 15 SEC IF OPEN AREA AVAILABLE
                        ROS_INFO("Found repeated wall");
                        if (minLeftDist < 0.7 && minFrontDist > 0.6 && minRightDist > 0.7)
                        { // RIGHT AREA OPEN
                            ROS_INFO("Found repeated wall - right area open");
                            rotate_deg(vel_pub, vel, -45);
                            finder_duration_turn = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - finder_start).count();
                        }
                        else if (minRightDist < 0.7 && minFrontDist > 0.6 && minLeftDist > 0.7)
                        { // LEFT AREA OPEN
                            ROS_INFO("Found repeated wall - left area open");
                            rotate_deg(vel_pub, vel, 45);
                            finder_duration_turn = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - finder_start).count();
                        }
                    }
                    goto found_repeated_wall;
                }
            }
            // BOTH CONDITIONS MEET, A NEW WALL IS FOUND!
            if (minFrontRightDist < 0.6 || minRightDist < 0.7)
            { // USE RIGHT HAND RULE
                right_rule = true;
            }
            else
            { // USE LEFT HAND RULE
                right_rule = false;
            }
            ROS_INFO("Found new wall");
            wall_found = true;
        }
        else
        { // IF CONDITIONS FAIL, DO OBSTACLE AVOIDANCE TILL CONDITIONS MEET
        found_repeated_wall:;
            avoid_obstacle(vel_pub, vel);
            if ((finder_duration - finder_duration_rotate) > 35 && forbid_spin == false)
            { // PERIODIC SPIN EVERY 40 SEC & SPIN ALLOWED
                ROS_INFO("Periodic Rotation");
                rotate_deg(vel_pub, vel, 181);
                rotate_deg(vel_pub, vel, 181);
                finder_duration_rotate = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - finder_start).count();
            }
        }
    }
}

void wall_explorer(ros::Publisher vel_pub, geometry_msgs::Twist vel)
{ // CHECK IF RETURN TO STARTING POSITION AFTER 20 SECONDS || PATH CHECK 0.5 || PERIODIC SPIN EVERY 30 SEC
    std::chrono::time_point<std::chrono::system_clock> explorer_start;
    explorer_start = std::chrono::system_clock::now();
    uint64_t explorer_duration = 0;
    uint64_t explorer_duration_path = 0;
    uint64_t explorer_duration_rotate = 0;
    bool wall_complete = false;
    // REFRESH IDLE STATE
    posX_prev = posX;
    posY_prev = posY;
    idle_checktime = secondsElapsed;
    // RECORD STARTING POSITION
    wall_start_pos.first = posX;
    wall_start_pos.second = posY;
    ROS_INFO("Start Wall Following");
    ROS_INFO("Starting Position: (%f, %f)", wall_start_pos.first, wall_start_pos.second);
    while (wall_complete == false)
    {
        explorer_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - explorer_start).count();
        if (explorer_duration > 180)
        { // CHECK IF STUCK IN WHILE DEFAULT 180 SEC
            ROS_INFO("STUCK IN EXPLORER --> PROGRAM REBOOTED.");
            while_trap = true;
        }
        if (is_idle() == true || while_trap == true || fault == true)
        { // CHECK PROGRAM FAULT
            fault = true;
            break;
        }
        wall_following = true;
        if (right_rule)
        { // FOLLOW THE RIGHT HAND WALL
            wall_follow_R(vel_pub, vel);
        }
        else
        { // FOLLOW THE LEFT HAND WALL
            wall_follow_L(vel_pub, vel);
        }
        if ((explorer_duration - explorer_duration_path) >= 1)
        { // RECORD POSITION EVERY SECOND
            path.push_back(std::make_pair(posX, posY));
            explorer_duration_path = explorer_duration;
            // ROS_INFO("Recorded Path Position: (%f, %f)", posX, posY);
        }
        if (explorer_duration > 20)
        { // CHECK IF RETURN TO STARTING POSITION AFTER 20 SECONDS
            if (calcDistance(posX, posY, wall_start_pos.first, wall_start_pos.second) <= 0.5)
            { // IF RETURN TO STARTING POINT WITHIN 0.5, FIND MAX DIST TO EXIT
                ROS_INFO("Complete Wall Following");
                face_max_dist(vel_pub, vel);
                wall_complete = true;
                break;
            }
        }
        if ((explorer_duration - explorer_duration_rotate) > 25 && forbid_spin == false)
        { // PERIODIC SPIN EVERY 30 SEC & SPIN ALLOWED
            ROS_INFO("Periodic Rotation");
            rotate_deg(vel_pub, vel, 181);
            rotate_deg(vel_pub, vel, 181);
            explorer_duration_rotate = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - explorer_start).count();
        }
    }
    ROS_INFO("Exiting Position: (%f, %f)", posX, posY);
    wall_following = false;
}

// MAIN BLOCK ======================================================================================================
int main(int argc, char **argv)
{
    // INITIALIZATION
    ros::init(argc, argv, "maze_explorer");
    ros::NodeHandle nh;
    // SUBSCRIBER - BUMPER, SCAN, ODOM
    ros::Subscriber bumper_sub = nh.subscribe("mobile_base/events/bumper", 10, &bumperCallback);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, &laserCallback);
    ros::Subscriber odom = nh.subscribe("odom", 1, &odomCallback);
    // PUBLISHER - VELOCITY
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
    geometry_msgs::Twist vel;
    // CONTEST TIMER
    ros::Rate loop_rate(10);
    start = std::chrono::system_clock::now();
    // STOP WHEN TERIMINATION SIGNAL RECEIVED OR 480 SECS PASSED
    while (ros::ok() && secondsElapsed <= 480)
    {
        fault = false;      // SYSTEM NO ERROR
        while_trap = false; // SYSTEM NO ERROR
        face_max_dist(vel_pub, vel);
        while (!fault)
        { // CONTROL LOGIC
            // CHECK SUBSCRIBED MSGS
            ros::spinOnce();
            wall_finder(vel_pub, vel);
            wall_explorer(vel_pub, vel);
        }
        // UPDATE TIMER
        secondsElapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start).count();
        loop_rate.sleep();
    }
    stop(vel_pub, vel);
    return 0;
}