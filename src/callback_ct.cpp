#include "cooperative_transportation_4ws_backstepping/callback_ct.hpp"
#include "cooperative_transportation_4ws_backstepping/initial.hpp"
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

// グローバル変数の定義
geometry_msgs::PoseStamped base_link_pose;
bool base_link_pose_received = false;

geometry_msgs::PoseStamped carrier_pose;
bool carrier_pose_received = false;

geometry_msgs::PoseStamped linkage_point1_pose;
bool linkage_point1_pose_received = false;

geometry_msgs::PoseStamped linkage_point2_pose;
bool linkage_point2_pose_received = false;

geometry_msgs::PoseStamped linkage_point3_pose;
bool linkage_point3_pose_received = false;

geometry_msgs::PoseStamped v1_rear_left_steering_pose;
bool v1_rear_left_steering_pose_received = false;

geometry_msgs::PoseStamped v1_front_left_steering_pose;
bool v1_front_left_steering_pose_received = false;

geometry_msgs::PoseStamped v2_rear_left_steering_pose;
bool v2_rear_left_steering_pose_received = false;

geometry_msgs::PoseStamped v2_front_left_steering_pose;
bool v2_front_left_steering_pose_received = false;

geometry_msgs::PoseStamped v3_rear_left_steering_pose;
bool v3_rear_left_steering_pose_received = false;

geometry_msgs::PoseStamped v3_front_left_steering_pose;
bool v3_front_left_steering_pose_received = false;

geometry_msgs::PoseStamped v1body_pose;
bool v1body_pose_received= false;

geometry_msgs::PoseStamped v2body_pose;
bool v2body_pose_received = false;

geometry_msgs::PoseStamped v3body_pose;
bool v3body_pose_received = false;



void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (size_t i = 0; i < msg->name.size(); ++i) {
        const auto& joint = msg->name[i];
        double pos = msg->position[i];  // [m] for prismatic

        // — プリズマティックジョイントの変位を delta_pos に格納 —
        if (joint == "linkage_point_to_v1") {
            delta_pos[0] = pos;
            linkage_point1_pose_received = true;
            // （PoseStamped の更新は必要に応じて）
        }
        else if (joint == "linkage_point_to_v2") {
            delta_pos[1] = pos;
            linkage_point2_pose_received = true;
        }
        else if (joint == "linkage_point_to_v3") {
            delta_pos[2] = pos;
            linkage_point3_pose_received = true;
        }
        else if (joint == "v1_rear_left_steering") {
            true_vehicle1_steering_yaw[1] = pos;  
        }
        else if (joint == "v2_rear_left_steering") {
            true_vehicle2_steering_yaw[1] = pos;  
        }
        else if (joint == "v3_rear_left_steering") {
            true_vehicle3_steering_yaw[1] = pos;  
        }
    }
}

// base_link 用のコールバック
void trueBaseLinkCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    base_link_pose.header = msg->header;
    base_link_pose.pose = msg->pose.pose;
    base_link_pose_received = true;
    
    tf2::Quaternion q;
    q.setX(base_link_pose.pose.orientation.x);
    q.setY(base_link_pose.pose.orientation.y);
    q.setZ(base_link_pose.pose.orientation.z);
    q.setW(base_link_pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    // ROS_INFO("True position of [base_link] (world): x=%f, y=%f, z=%f | Orientation: roll=%f, pitch=%f, yaw=%f",
    //          base_link_pose.pose.position.x,
    //          base_link_pose.pose.position.y,
    //          base_link_pose.pose.position.z,
    //          roll, pitch, yaw);
    
}

// carrier 用のコールバック
void trueCarrierCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    carrier_pose.header = msg->header;
    carrier_pose.pose = msg->pose.pose;
    carrier_pose_received = true;
    
    tf2::Quaternion q;
    q.setX(carrier_pose.pose.orientation.x);
    q.setY(carrier_pose.pose.orientation.y);
    q.setZ(carrier_pose.pose.orientation.z);
    q.setW(carrier_pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    true_carrier_pos[0] = carrier_pose.pose.position.x;
    true_carrier_pos[1] = carrier_pose.pose.position.y;
    true_carrier_yaw  = yaw;
    x_old[1] = carrier_pose.pose.position.x;
    x_old[2] = carrier_pose.pose.position.y;
    x_old[5] = yaw;//theta1

    got_body_pos = true;

    
    // ROS_INFO("True position of [carrier] (world): x=%f, y=%f, z=%f | Orientation: roll=%f, pitch=%f, yaw=%f",
    //          carrier_pose.pose.position.x,
    //          carrier_pose.pose.position.y,
    //          carrier_pose.pose.position.z,
    //          roll, pitch, yaw);
}


// vehicle1の姿勢角のコールバック
void trueV1BodyCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    v1body_pose.header = msg->header;
    v1body_pose.pose = msg->pose.pose;
    v1body_pose_received = true;
    
    tf2::Quaternion q;
    q.setX(v1body_pose.pose.orientation.x);
    q.setY(v1body_pose.pose.orientation.y);
    q.setZ(v1body_pose.pose.orientation.z);
    q.setW(v1body_pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //X：theta4の座標
    true_vehicle_yaw[0] = yaw;
    x_old[11] = yaw;

    //q：x1,y1,theta1の座標
    q_twist[3] = v1body_pose.pose.position.x;
    q_twist[4] = v1body_pose.pose.position.y;
    q_twist[5] = yaw; 
    qdot_twist[3] = msg->twist.twist.linear.x;
    qdot_twist[4] = msg->twist.twist.linear.y;
    qdot_twist[5] = msg->twist.twist.angular.z;
    x_d[3] = qdot_twist[3];
    x_d[4] = qdot_twist[4];
    x_d[5] = qdot_twist[5];
    
//     ROS_INFO("True position of [linkage_point1] (world): x=%f, y=%f, z=%f | Orientation: roll=%f, pitch=%f, yaw=%f",
//              linkage_point1_pose.pose.position.x,
//              linkage_point1_pose.pose.position.y,
//              linkage_point1_pose.pose.position.z,
//              roll, pitch, yaw);
}






// vehicle2の姿勢角 用のコールバック
void trueV2BodyCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    v2body_pose.header = msg->header;
    v2body_pose.pose = msg->pose.pose;
    v2body_pose_received = true;
    
    tf2::Quaternion q;
    q.setX(v2body_pose.pose.orientation.x);
    q.setY(v2body_pose.pose.orientation.y);
    q.setZ(v2body_pose.pose.orientation.z);
    q.setW(v2body_pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    true_vehicle_yaw[1] = yaw;
    x_old[17] = yaw;

    //q：x2,y2,theta2の座標
    q_twist[7] = v2body_pose.pose.position.x;
    q_twist[8] = v2body_pose.pose.position.y;
    q_twist[9] = yaw; 
    qdot_twist[7] = msg->twist.twist.linear.x;
    qdot_twist[8] = msg->twist.twist.linear.y;
    qdot_twist[9] = msg->twist.twist.angular.z;
    x_d[11] = qdot_twist[7];
    x_d[12] = qdot_twist[8];
    x_d[13] = qdot_twist[9];
    
    // ROS_INFO("True position of [linkage_point2] (world): x=%f, y=%f, z=%f | Orientation: roll=%f, pitch=%f, yaw=%f",
    //          linkage_point2_pose.pose.position.x,
    //          linkage_point2_pose.pose.position.y,
    //          linkage_point2_pose.pose.position.z,
    //          roll, pitch, yaw);
}



// vehicle3の姿勢角 用のコールバック
void trueV3BodyCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    v3body_pose.header = msg->header;
    v3body_pose.pose = msg->pose.pose;
    v3body_pose_received = true;
    
    tf2::Quaternion q;
    q.setX(v3body_pose.pose.orientation.x);
    q.setY(v3body_pose.pose.orientation.y);
    q.setZ(v3body_pose.pose.orientation.z);
    q.setW(v3body_pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    true_vehicle_yaw[2] = yaw;
    x_old[23] = yaw;

    //q：x3,y3,theta3の座標
    q_twist[11] = v3body_pose.pose.position.x;
    q_twist[12] = v3body_pose.pose.position.y;
    q_twist[13] = yaw; 
    qdot_twist[11] = msg->twist.twist.linear.x;
    qdot_twist[12] = msg->twist.twist.linear.y;
    qdot_twist[13] = msg->twist.twist.angular.z;
    x_d[19] = qdot_twist[11];
    x_d[20] = qdot_twist[12];
    x_d[21] = qdot_twist[13];
    
    // ROS_INFO("True position of [linkage_point3] (world): x=%f, y=%f, z=%f | Orientation: roll=%f, pitch=%f, yaw=%f",
    //          linkage_point3_pose.pose.position.x,
    //          linkage_point3_pose.pose.position.y,
    //          linkage_point3_pose.pose.position.z,
    //          roll, pitch, yaw);
}


// vehicle1の姿勢角のコールバック
void trueLinkagePoint1Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    linkage_point1_pose.header = msg->header;
    linkage_point1_pose.pose = msg->pose.pose;
    linkage_point1_pose_received = true;
    
    tf2::Quaternion q;
    q.setX(linkage_point1_pose.pose.orientation.x);
    q.setY(linkage_point1_pose.pose.orientation.y);
    q.setZ(linkage_point1_pose.pose.orientation.z);
    q.setW(linkage_point1_pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    true_vehicle_yaw[0] = yaw;
    x_old[11] = yaw + PAI;
    
//     ROS_INFO("True position of [linkage_point1] (world): x=%f, y=%f, z=%f | Orientation: roll=%f, pitch=%f, yaw=%f",
//              linkage_point1_pose.pose.position.x,
//              linkage_point1_pose.pose.position.y,
//              linkage_point1_pose.pose.position.z,
//              roll, pitch, yaw);
}

// vehicle2の姿勢角 用のコールバック
void trueLinkagePoint2Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    linkage_point2_pose.header = msg->header;
    linkage_point2_pose.pose = msg->pose.pose;
    linkage_point2_pose_received = true;
    
    tf2::Quaternion q;
    q.setX(linkage_point2_pose.pose.orientation.x);
    q.setY(linkage_point2_pose.pose.orientation.y);
    q.setZ(linkage_point2_pose.pose.orientation.z);
    q.setW(linkage_point2_pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    
    true_vehicle_yaw[1] = yaw;
    x_old[17] = yaw;
    
    // ROS_INFO("True position of [linkage_point2] (world): x=%f, y=%f, z=%f | Orientation: roll=%f, pitch=%f, yaw=%f",
    //          linkage_point2_pose.pose.position.x,
    //          linkage_point2_pose.pose.position.y,
    //          linkage_point2_pose.pose.position.z,
    //          roll, pitch, yaw);
}

// vehicle3の姿勢角 用のコールバック
void trueLinkagePoint3Callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    linkage_point3_pose.header = msg->header;
    linkage_point3_pose.pose = msg->pose.pose;
    linkage_point3_pose_received = true;
    
    tf2::Quaternion q;
    q.setX(linkage_point3_pose.pose.orientation.x);
    q.setY(linkage_point3_pose.pose.orientation.y);
    q.setZ(linkage_point3_pose.pose.orientation.z);
    q.setW(linkage_point3_pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    true_vehicle_yaw[2] = yaw;
    x_old[23] = yaw;
    
    // ROS_INFO("True position of [linkage_point3] (world): x=%f, y=%f, z=%f | Orientation: roll=%f, pitch=%f, yaw=%f",
    //          linkage_point3_pose.pose.position.x,
    //          linkage_point3_pose.pose.position.y,
    //          linkage_point3_pose.pose.position.z,
    //          roll, pitch, yaw);
}


//  v1_front用のコールバック
void trueV1FrontLeftSteeringCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    v1_front_left_steering_pose.header = msg->header;
    v1_front_left_steering_pose.pose = msg->pose.pose;
    v1_front_left_steering_pose_received = true;
    
    tf2::Quaternion q;
    q.setX(v1_front_left_steering_pose.pose.orientation.x);
    q.setY(v1_front_left_steering_pose.pose.orientation.y);
    q.setZ(v1_front_left_steering_pose.pose.orientation.z);
    q.setW(v1_front_left_steering_pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    true_vehicle1_steering_yaw[0] = yaw;
    //x_old[8] = yaw - x_old[8];
    
    // ROS_INFO("True position of [v1_front_left_steering] (world): x=%f, y=%f, z=%f | Orientation: roll=%f, pitch=%f, yaw=%f",
    //          v1_front_left_steering_pose.pose.position.x,
    //          v1_front_left_steering_pose.pose.position.y,
    //          v1_front_left_steering_pose.pose.position.z,
    //          roll, pitch, yaw);
}





//  v2_front用のコールバック
void trueV2FrontLeftSteeringCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    v2_front_left_steering_pose.header = msg->header;
    v2_front_left_steering_pose.pose = msg->pose.pose;
    v2_front_left_steering_pose_received = true;
    
    tf2::Quaternion q;
    q.setX(v2_front_left_steering_pose.pose.orientation.x);
    q.setY(v2_front_left_steering_pose.pose.orientation.y);
    q.setZ(v2_front_left_steering_pose.pose.orientation.z);
    q.setW(v2_front_left_steering_pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    true_vehicle2_steering_yaw[0] = yaw;
    //x_old[15] = x_old[14] + yaw;

    // ROS_INFO("True position of [v2_front_left_steering] (world): x=%f, y=%f, z=%f | Orientation: roll=%f, pitch=%f, yaw=%f",
    //          v2_front_left_steering_pose.pose.position.x,
    //          v2_front_left_steering_pose.pose.position.y,
    //          v2_front_left_steering_pose.pose.position.z,
    //          roll, pitch, yaw);
}




//  v3_front用のコールバック
void trueV3FrontLeftSteeringCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    v3_front_left_steering_pose.header = msg->header;
    v3_front_left_steering_pose.pose = msg->pose.pose;
    v3_front_left_steering_pose_received = true;
    
    tf2::Quaternion q;
    q.setX(v3_front_left_steering_pose.pose.orientation.x);
    q.setY(v3_front_left_steering_pose.pose.orientation.y);
    q.setZ(v3_front_left_steering_pose.pose.orientation.z);
    q.setW(v3_front_left_steering_pose.pose.orientation.w);
    
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    true_vehicle3_steering_yaw[0] = yaw;
    //x_old[21] = x_old[20] + yaw;
    
    // ROS_INFO("True position of [v3_front_left_steering] (world): x=%f, y=%f, z=%f | Orientation: roll=%f, pitch=%f, yaw=%f",
    //          v3_front_left_steering_pose.pose.position.x,
    //          v3_front_left_steering_pose.pose.position.y,
    //          v3_front_left_steering_pose.pose.position.z,
    //          roll, pitch, true_vehicle3_steering_yaw[1]);
}

