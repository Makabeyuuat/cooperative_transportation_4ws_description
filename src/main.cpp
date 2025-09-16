#include <ros/ros.h>
#include <cmath>
#include <vector>
#include "vehicle.hpp"       // Vehicle クラスの宣言
#include "callback.hpp"     // コールバック関数の宣言
#include "global.hpp"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#define PI 3.141592653589793

int main(int argc, char** argv)
{
    ros::init(argc, argv, "steering_desired_controller");
    ros::NodeHandle nh;

    //joint_statesを購読
    ros::Subscriber joint_state_sub    = nh.subscribe("/cooperative_transportation_4ws/joint_states", 10, jointStateCallback);

    // 各トピックを別々のコールバック関数で購読
    ros::Subscriber true_baselink_sub = nh.subscribe("/cooperative_transportation_4ws/true_base_link", 10, trueBaseLinkCallback);
    ros::Subscriber true_carrier_sub = nh.subscribe("/cooperative_transportation_4ws/true_carrier", 10, trueCarrierCallback);
    // ros::Subscriber true_linkage_point1_sub = nh.subscribe("/cooperative_transportation_4ws/true_linkage_point1", 10, trueLinkagePoint1Callback);
    // ros::Subscriber true_linkage_point2_sub = nh.subscribe("/cooperative_transportation_4ws/true_linkage_point2", 10, trueLinkagePoint2Callback);
    // ros::Subscriber true_linkage_point3_sub = nh.subscribe("/cooperative_transportation_4ws/true_linkage_point3", 10, trueLinkagePoint3Callback);
    // ros::Subscriber true_v1_front_sub = nh.subscribe("/cooperative_transportation_4ws/true_v1_front", 10, trueV1FrontLeftSteeringCallback);
    // ros::Subscriber true_v1_rear_sub = nh.subscribe("/cooperative_transportation_4ws/true_v1_rear", 10, trueV1RearLeftSteeringCallback);

    // Vehicle クラスのインスタンス生成（v1, v2, v3 の3台）
    Vehicle vehicle1(nh, "v1");
    Vehicle vehicle2(nh, "v2");
    Vehicle vehicle3(nh, "v3");

    // 各ジョイントフレームのリスト（例：ヒンジや linkage_point のフレーム）
    std::vector<std::string> tf_joint_frames = {
         "cariier", "v1_front_left_hinge_1", "v1_front_right_hinge_1",
         "v1_rear_left_hinge_1", "v1_rear_right_hinge_1",
         "v2_front_left_hinge_1", "v2_front_right_hinge_1",
         "v2_rear_left_hinge_1", "v2_rear_right_hinge_1",
         "v3_front_left_hinge_1", "v3_front_right_hinge_1",
         "v3_rear_left_hinge_1", "v3_rear_right_hinge_1",
         "linkage_point1_1", "linkage_point2_1", "linkage_point3_1"
    };

    ros::Rate loop_rate(50); // 50Hz の制御ループ

    while (ros::ok()) {
        double current_time = ros::Time::now().toSec();
        double desired_steering = 0.5 * sin(current_time);

        // 各車両へ steering コマンドと車輪の回転速度コマンドを送信
        vehicle1.publishSteeringCommand(desired_steering);
        vehicle2.publishSteeringCommand(desired_steering);
        vehicle3.publishSteeringCommand(desired_steering);

        double wheel_command = 0.5;
        vehicle1.publishWheelCommand(wheel_command);
        vehicle2.publishWheelCommand(wheel_command);
        vehicle3.publishWheelCommand(wheel_command);


        // inline変数を出力（ここではROS_INFOを使用）
        ROS_INFO("Carrier Position: x = %f, y = %f", true_carrier_pos[0], true_carrier_pos[1]);

        ROS_INFO("Prismatic displace: linkage1 = %f m, linkage2 = %f m, linkage3 = %f m", delta_pos[0], delta_pos[1], delta_pos[2]);

        // ※ TF2 を用いた追加の変換処理等があればここに記述可能

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

