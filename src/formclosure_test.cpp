#include <ros/ros.h>
#include <std_msgs/Float64.h>

#define PI 3.141592653589793


int main(int argc, char** argv) {
    // ROSノードの初期化
    ros::init(argc, argv, "velocity_command_publisher");
    ros::NodeHandle nh;

    // vehicle1のvelocityトピックのPublisherを作成
    ros::Publisher sliding_machine1_pub = nh.advertise<std_msgs::Float64>("/cooperative_transportation_4ws/linkage_point_to_v1_velocity_controller/command", 1);
    ros::Publisher sliding_machine2_pub = nh.advertise<std_msgs::Float64>("/cooperative_transportation_4ws/linkage_point_to_v2_velocity_controller/command", 1);
    ros::Publisher sliding_machine3_pub = nh.advertise<std_msgs::Float64>("/cooperative_transportation_4ws/linkage_point_to_v3_velocity_controller/command", 1);

    ros::Rate loop_rate(50); // 50Hzでループ

    // 指令する速度値
    double velocity_value = 0.1;   
    double velocity_re_value = -0.1;        // 車輪の速度 [m/s]

    while (ros::ok()) {
        // メッセージ作成
        std_msgs::Float64 velocity_msg;
        std_msgs::Float64 velocity_re_msg;
        //std_msgs::Float64 steering_position_msg;

        //double current_time = ros::Time::now().toSec();
        //double steering_position_value = PI/6 * sin(current_time); // -1.57 ~ 1.57 の範囲で変化
        velocity_msg.data = velocity_value;
        velocity_re_msg.data = velocity_re_value;
        //steering_position_msg.data = steering_position_value;


        sliding_machine1_pub.publish(velocity_re_msg);
        sliding_machine2_pub.publish(velocity_msg);
        sliding_machine3_pub.publish(velocity_msg);
        // vehicle1_front_left_pub.publish(velocity_msg);
        // vehicle1_front_right_pub.publish(velocity_msg);
        // vehicle1_rear_left_pub.publish(velocity_msg);
        // vehicle1_rear_right_pub.publish(velocity_msg);
        // // ステアリングに速度を送信
        // vehicle1_front_steering_left_pub.publish(steering_position_msg);
        // vehicle1_front_steering_right_pub.publish(steering_position_msg);
        // vehicle1_rear_steering_left_pub.publish(steering_position_msg);
        // vehicle1_rear_steering_right_pub.publish(steering_position_msg);
        
        // // 第２車両の各車輪に速度を送信
        // vehicle2_front_left_pub.publish(velocity_msg);
        // vehicle2_front_right_pub.publish(velocity_msg);
        // vehicle2_rear_left_pub.publish(velocity_msg);
        // vehicle2_rear_right_pub.publish(velocity_msg);
        // //ステアリングに速度を送信
        // vehicle2_front_steering_left_pub.publish(steering_position_msg);
        // vehicle2_front_steering_right_pub.publish(steering_position_msg);
        // vehicle2_rear_steering_left_pub.publish(steering_position_msg);
        // vehicle2_rear_steering_right_pub.publish(steering_position_msg);

        // // 第３車両の各車輪に速度を送信
        // vehicle3_front_left_pub.publish(velocity_msg);
        // vehicle3_front_right_pub.publish(velocity_msg);
        // vehicle3_rear_left_pub.publish(velocity_msg);
        // vehicle3_rear_right_pub.publish(velocity_msg);
        // //ステアリングに速度を送信
        // vehicle3_front_steering_left_pub.publish(steering_position_msg);
        // vehicle3_front_steering_right_pub.publish(steering_position_msg);
        // vehicle3_rear_steering_left_pub.publish(steering_position_msg);
        // vehicle3_rear_steering_right_pub.publish(steering_position_msg);

        // ループのスリープ
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
