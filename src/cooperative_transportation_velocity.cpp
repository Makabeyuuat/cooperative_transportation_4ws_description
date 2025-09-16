#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <cmath>
#include <string>
#include <map>
#include <vector>

#define PI 3.141592653589793

// 各steeringジョイントの実測値を保持するグローバルマップ
std::map<std::string, double> steering_measured;

// /cooperative_transportation_4ws/joint_states から、名前に"steering"を含むジョイントの値を更新
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    for (size_t i = 0; i < msg->name.size(); i++) {
        std::string joint_name = msg->name[i];
        if (joint_name.find("steering") != std::string::npos) {
            steering_measured[joint_name] = msg->position[i];
            ROS_INFO("Measured %s: %f", joint_name.c_str(), msg->position[i]);
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "steering_desired_controller");
    ros::NodeHandle nh;

    // --- 車輪用Publisherの作成 ---
    ros::Publisher vehicle1_front_left_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v1_front_left_wheel_rotation_velocity_controller/command", 1);
    ros::Publisher vehicle1_front_right_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v1_front_right_wheel_rotation_velocity_controller/command", 1);
    ros::Publisher vehicle1_rear_left_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v1_rear_left_wheel_rotation_velocity_controller/command", 1);
    ros::Publisher vehicle1_rear_right_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v1_rear_right_wheel_rotation_velocity_controller/command", 1);

    ros::Publisher vehicle2_front_left_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v2_front_left_wheel_rotation_velocity_controller/command", 1);
    ros::Publisher vehicle2_front_right_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v2_front_right_wheel_rotation_velocity_controller/command", 1);
    ros::Publisher vehicle2_rear_left_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v2_rear_left_wheel_rotation_velocity_controller/command", 1);
    ros::Publisher vehicle2_rear_right_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v2_rear_right_wheel_rotation_velocity_controller/command", 1);

    ros::Publisher vehicle3_front_left_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v3_front_left_wheel_rotation_velocity_controller/command", 1);
    ros::Publisher vehicle3_front_right_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v3_front_right_wheel_rotation_velocity_controller/command", 1);
    ros::Publisher vehicle3_rear_left_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v3_rear_left_wheel_rotation_velocity_controller/command", 1);
    ros::Publisher vehicle3_rear_right_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v3_rear_right_wheel_rotation_velocity_controller/command", 1);

    // --- STEERING用Publisherの作成 ---
    ros::Publisher vehicle1_front_left_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v1_front_left_steering_position_controller/command", 1);
    ros::Publisher vehicle1_front_right_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v1_front_right_steering_position_controller/command", 1);
    ros::Publisher vehicle1_rear_left_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v1_rear_left_steering_position_controller/command", 1);
    ros::Publisher vehicle1_rear_right_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v1_rear_right_steering_position_controller/command", 1);

    ros::Publisher vehicle2_front_left_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v2_front_left_steering_position_controller/command", 1);
    ros::Publisher vehicle2_front_right_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v2_front_right_steering_position_controller/command", 1);
    ros::Publisher vehicle2_rear_left_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v2_rear_left_steering_position_controller/command", 1);
    ros::Publisher vehicle2_rear_right_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v2_rear_right_steering_position_controller/command", 1);

    ros::Publisher vehicle3_front_left_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v3_front_left_steering_position_controller/command", 1);
    ros::Publisher vehicle3_front_right_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v3_front_right_steering_position_controller/command", 1);
    ros::Publisher vehicle3_rear_left_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v3_rear_left_steering_position_controller/command", 1);
    ros::Publisher vehicle3_rear_right_steering_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/v3_rear_right_steering_position_controller/command", 1);

    // --- 摺動機構用Publisherの作成 ---
    ros::Publisher sliding_machine1_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/linkage_point_to_v1_velocity_controller/command", 1);
    ros::Publisher sliding_machine2_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/linkage_point_to_v2_velocity_controller/command", 1);
    ros::Publisher sliding_machine3_pub = nh.advertise<std_msgs::Float64>(
        "/cooperative_transportation_4ws/linkage_point_to_v3_velocity_controller/command", 1);

    // --- joint_statesの購読 ---
    ros::Subscriber joint_state_sub = nh.subscribe("/cooperative_transportation_4ws/joint_states", 10, jointStateCallback);

    // 車輪用Publisherを vector にまとめる
    std::vector<ros::Publisher> wheel_publishers = {
        vehicle1_front_left_pub, vehicle1_front_right_pub, vehicle1_rear_left_pub, vehicle1_rear_right_pub,
        vehicle2_front_left_pub, vehicle2_front_right_pub, vehicle2_rear_left_pub, vehicle2_rear_right_pub,
        vehicle3_front_left_pub, vehicle3_front_right_pub, vehicle3_rear_left_pub, vehicle3_rear_right_pub
    };

    // 摺動機構用Publisherを vector にまとめる
    std::vector<ros::Publisher> sliding_publishers = {
        sliding_machine1_pub, sliding_machine2_pub, sliding_machine3_pub
    };

    // STEERING用Publisherは map にまとめる（後で for ループで処理）
    std::map<std::string, ros::Publisher> steering_pub_map;
    steering_pub_map["v1_front_left_steering"] = vehicle1_front_left_steering_pub;
    steering_pub_map["v1_front_right_steering"] = vehicle1_front_right_steering_pub;
    steering_pub_map["v1_rear_left_steering"]  = vehicle1_rear_left_steering_pub;
    steering_pub_map["v1_rear_right_steering"] = vehicle1_rear_right_steering_pub;
    steering_pub_map["v2_front_left_steering"] = vehicle2_front_left_steering_pub;
    steering_pub_map["v2_front_right_steering"] = vehicle2_front_right_steering_pub;
    steering_pub_map["v2_rear_left_steering"]  = vehicle2_rear_left_steering_pub;
    steering_pub_map["v2_rear_right_steering"] = vehicle2_rear_right_steering_pub;
    steering_pub_map["v3_front_left_steering"] = vehicle3_front_left_steering_pub;
    steering_pub_map["v3_front_right_steering"] = vehicle3_front_right_steering_pub;
    steering_pub_map["v3_rear_left_steering"]  = vehicle3_rear_left_steering_pub;
    steering_pub_map["v3_rear_right_steering"] = vehicle3_rear_right_steering_pub;

    // steering ジョイント名（ログ出力用）
    std::vector<std::string> steering_joints = {
        "v1_front_left_steering", "v1_front_right_steering",
        "v1_rear_left_steering",  "v1_rear_right_steering",
        "v2_front_left_steering", "v2_front_right_steering",
        "v2_rear_left_steering",  "v2_rear_right_steering",
        "v3_front_left_steering", "v3_front_right_steering",
        "v3_rear_left_steering",  "v3_rear_right_steering"
    };

    ros::Rate loop_rate(50); // 50Hzループ

    while (ros::ok()) {
        double current_time = ros::Time::now().toSec();
        // 例：全steeringジョイントの目標値をsin波で生成
        double desired_steering = 0.5 * sin(current_time);

        // STEERING用のコマンドを送信（内部PIDはcontroller.yaml側で処理）
        for (const auto& joint : steering_joints) {
            std_msgs::Float64 cmd;
            cmd.data = desired_steering;
            if (steering_pub_map.find(joint) != steering_pub_map.end()) {
                steering_pub_map[joint].publish(cmd);
            }
            if (steering_measured.find(joint) != steering_measured.end()) {
                double error = desired_steering - steering_measured[joint];
                ROS_INFO("%s: desired=%f measured=%f error=%f",
                         joint.c_str(), desired_steering, steering_measured[joint], error);
            }
        }

        // 車輪へのコマンドをfor文で一括送信
        std_msgs::Float64 wheel_cmd;
        wheel_cmd.data = 1.5;
        for (auto& pub : wheel_publishers) {
            pub.publish(wheel_cmd);
        }

        // 摺動機構へのコマンドをfor文で一括送信
        std_msgs::Float64 slide_cmd;
        slide_cmd.data = 0.1;
        for (auto& pub : sliding_publishers) {
            pub.publish(slide_cmd);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
