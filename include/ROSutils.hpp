#pragma once

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "Config.hpp"
#include "Objects.hpp"

Imu fromROS(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
    Imu imu;
    imu.stamp = rclcpp::Time(msg->header.stamp).seconds();

    imu.ang_vel(0) = msg->angular_velocity.x;
    imu.ang_vel(1) = msg->angular_velocity.y;
    imu.ang_vel(2) = msg->angular_velocity.z;

    imu.lin_accel(0) = msg->linear_acceleration.x;
    imu.lin_accel(1) = msg->linear_acceleration.y;
    imu.lin_accel(2) = msg->linear_acceleration.z;

    imu.q = Eigen::Quaterniond(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w
    );

    return imu;
}

Cones fromROS(const geometry_msgs::msg::PoseArray::ConstSharedPtr &msg) {
    
    Config &cfg = Config::getInstance();
    
    Cones cones;
    for (const auto &p : msg->poses) {
        Eigen::Vector3d cone(p.position.x, p.position.y, p.position.z);
        
        cones.push_back(cfg.lidar2baselink * cone);
        // cones.push_back(cone);
    }

    return cones;
}

void fill_config(Config &cfg, rclcpp::Node *node) {

    node->get_parameter("topics.input.impu", cfg.topics.input.imu);
    node->get_parameter("topics.output.state", cfg.topics.output.state);

    std::vector<double> t, R;
    node->get_parameter("lidar2baselink.t", t);

    node->get_parameter("lidar2baselink.R", R);

    Eigen::Quaterniond q =
        Eigen::AngleAxisd(R[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(R[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(R[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ());

    q.normalize();

    cfg.lidar2baselink = Eigen::Translation3d(Eigen::Vector3d(t[0], t[1], t[2])) * q;


}
