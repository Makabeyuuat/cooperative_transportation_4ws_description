#include "vehicle.hpp"

// コンストラクタの実装
Vehicle::Vehicle(ros::NodeHandle& nh, const std::string& vehicle_id) : vehicle_id_(vehicle_id) {
    // トピック名の接頭辞（名前空間）
    std::string ns = "/cooperative_transportation_4ws/";

    // 車輪用パブリッシャーの初期化
    front_left_wheel_pub = nh.advertise<std_msgs::Float64>(ns + vehicle_id_ + "_front_left_wheel_rotation_velocity_controller/command", 1);
    front_right_wheel_pub = nh.advertise<std_msgs::Float64>(ns + vehicle_id_ + "_front_right_wheel_rotation_velocity_controller/command", 1);
    rear_left_wheel_pub = nh.advertise<std_msgs::Float64>(ns + vehicle_id_ + "_rear_left_wheel_rotation_velocity_controller/command", 1);
    rear_right_wheel_pub = nh.advertise<std_msgs::Float64>(ns + vehicle_id_ + "_rear_right_wheel_rotation_velocity_controller/command", 1);

    // STEERING用パブリッシャーの初期化
    front_left_steering_pub = nh.advertise<std_msgs::Float64>(ns + vehicle_id_ + "_front_left_steering_position_controller/command", 1);
    front_right_steering_pub = nh.advertise<std_msgs::Float64>(ns + vehicle_id_ + "_front_right_steering_position_controller/command", 1);
    rear_left_steering_pub = nh.advertise<std_msgs::Float64>(ns + vehicle_id_ + "_rear_left_steering_position_controller/command", 1);
    rear_right_steering_pub = nh.advertise<std_msgs::Float64>(ns + vehicle_id_ + "_rear_right_steering_position_controller/command", 1);
}

// 各steeringジョイントへ指令値を送信するメソッド
void Vehicle::publishSteeringCommand(double cmd_value) {
    std_msgs::Float64 msg;
    msg.data = cmd_value;
    front_left_steering_pub.publish(msg);
    front_right_steering_pub.publish(msg);
    rear_left_steering_pub.publish(msg);
    rear_right_steering_pub.publish(msg);
}

// 各車輪へ回転速度コマンドを送信するメソッド
void Vehicle::publishWheelCommand(double cmd_value) {
    std_msgs::Float64 msg;
    msg.data = cmd_value;
    front_left_wheel_pub.publish(msg);
    front_right_wheel_pub.publish(msg);
    rear_left_wheel_pub.publish(msg);
    rear_right_wheel_pub.publish(msg);
}
