#ifndef CALLBACK_HPP
#define CALLBACK_HPP

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

// グローバル変数の宣言（各トピックの絶対位置を保持）
extern geometry_msgs::PoseStamped base_link_pose;
extern bool base_link_pose_received;

extern geometry_msgs::PoseStamped carrier_pose;
extern bool carrier_pose_received;

extern geometry_msgs::PoseStamped linkage_point1_pose;
extern bool linkage_point1_pose_received;

extern geometry_msgs::PoseStamped linkage_point2_pose;
extern bool linkage_point2_pose_received;

extern geometry_msgs::PoseStamped linkage_point3_pose;
extern bool linkage_point3_pose_received;

extern geometry_msgs::PoseStamped v1_rear_left_steering_pose;
extern bool v1_rear_left_steering_pose_received;

extern geometry_msgs::PoseStamped v1_front_left_steering_pose;
extern bool v1_front_left_steering_pose_received;

inline double delta_pos[3] = {0.0, 0.0, 0.0};

// コールバック関数の宣言
void trueBaseLinkCallback(const nav_msgs::Odometry::ConstPtr& msg);
void trueCarrierCallback(const nav_msgs::Odometry::ConstPtr& msg);
void trueLinkagePoint1Callback(const nav_msgs::Odometry::ConstPtr& msg);
void trueLinkagePoint2Callback(const nav_msgs::Odometry::ConstPtr& msg);
void trueLinkagePoint3Callback(const nav_msgs::Odometry::ConstPtr& msg);
void trueV1RearLeftSteeringCallback(const nav_msgs::Odometry::ConstPtr& msg);
void trueV1FrontLeftSteeringCallback(const nav_msgs::Odometry::ConstPtr& msg);
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);

#endif // CALLBACK_HPP
