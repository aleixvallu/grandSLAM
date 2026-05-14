#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "cat_msgs/msg/cone_array.hpp"

#include "Config.hpp"
#include "Objects.hpp"
#include "Graph.hpp"

Imu fromROS(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
    Config &cfg = Config::getInstance();

    Imu imu;
    imu.stamp = rclcpp::Time(msg->header.stamp).seconds();

    tf2::fromMsg(msg->angular_velocity, imu.ang_vel);

    tf2::fromMsg(msg->linear_acceleration, imu.lin_accel);
 
    tf2::fromMsg(msg->orientation, imu.q);
    imu.q.normalize();


    return imu;
}


Cones fromROS(const cat_msgs::msg::ConeArray::ConstSharedPtr &msg) {
    
    Config &cfg = Config::getInstance();
    
    Cones cones;
    for (const auto &p : msg->cones) {
        Cone cone(p.position_base_link.x, p.position_base_link.y, p.position_base_link.z);
        cones.push_back(cone);
    }

    return cones;
}





nav_msgs::msg::Odometry toROS(const Graph &state, rclcpp::Time stamp) {
    Config &cfg = Config::getInstance();

    nav_msgs::msg::Odometry msg;

    msg.header.frame_id = "global";
    msg.header.stamp = stamp;

    Eigen::Isometry3d T_W_B = state.isometry() * cfg.imu2baselink.inverse();

    msg.pose.pose.position = tf2::toMsg(Eigen::Vector3d(T_W_B.translation()));
    msg.pose.pose.orientation = tf2::toMsg(Eigen::Quaterniond(T_W_B.linear()));

    // TODO: mirar com posar w
    msg.twist.twist = tf2::toMsg(state.v_w());

    // LIMO
    // auto &T_B_I = cfg.sensors.extrinsics.imu2baselink;
    // Eigen::Matrix3d R_BI = T_B_I.linear();
    // Eigen::Vector3d t_BI = T_B_I.translation();

    // Eigen::Vector3d w_B = R_BI * (state.w - state.b_w());
    // out.twist.twist.angular.x = w_B.x();
    // out.twist.twist.angular.y = w_B.y();
    // out.twist.twist.angular.z = w_B.z();

    // Eigen::Vector3d v_B = R_BI * state.R().transpose() * state.v() + t_BI.cross(w_B);
    // out.twist.twist.linear.x = v_B.x();
    // out.twist.twist.linear.y = v_B.y();
    // out.twist.twist.linear.z = v_B.z();


    return msg;
}


visualization_msgs::msg::MarkerArray toROS(const Cones &cones, rclcpp::Time stamp) {
    Config &cfg = Config::getInstance();
    
    visualization_msgs::msg::MarkerArray msg;


    msg.markers.resize(cones.size());

    for (int i = 0; i < cones.size(); i++) {

        msg.markers[i].header.stamp    = stamp;
        msg.markers[i].header.frame_id = "global";
        msg.markers[i].ns              = "cones";
        msg.markers[i].id              = i;
        msg.markers[i].type            = visualization_msgs::msg::Marker::CYLINDER;
        msg.markers[i].action          = visualization_msgs::msg::Marker::ADD;

        // pose        
        // Eigen::Vector3d rCone = cfg.imu2baselink * cones[i].toEigen();
        Eigen::Vector3d rCone = cones[i].toEigen();

        msg.markers[i].pose.position.x = rCone.x();
        msg.markers[i].pose.position.y = rCone.y();
        msg.markers[i].pose.position.z = rCone.z();

        // fixed cylinder scale
        msg.markers[i].scale.x = 0.2;
        msg.markers[i].scale.y = 0.2;
        msg.markers[i].scale.z = 0.3;

        // fully opaque
        msg.markers[i].color.a = 1.0f;
        msg.markers[i].color.r = 0.5f; 
        msg.markers[i].color.g = 0.5f;
        msg.markers[i].color.b = 0.5f;

    }
    return msg;

}

geometry_msgs::msg::TransformStamped toTF(const Eigen::Isometry3d &T, const std::string &parent,
                                          const std::string &child, const rclcpp::Time &stamp) {

    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = parent;
    msg.child_frame_id = child;

    Eigen::Vector3d p = T.translation();
    Eigen::Quaterniond q(T.linear());

    msg.transform.translation.x = p.x();
    msg.transform.translation.y = p.y();
    msg.transform.translation.z = p.z();
    msg.transform.rotation = tf2::toMsg(q);

    return msg;
}

void publishTFs(const Graph &state, tf2_ros::TransformBroadcaster &br, const rclcpp::Time stamp) {

    Config &cfg = Config::getInstance();

    Eigen::Isometry3d T_B_I = cfg.imu2baselink;
    Eigen::Isometry3d T_I_B = T_B_I.inverse();
    Eigen::Isometry3d T_M_B = state.isometry() * T_I_B;
    Eigen::Isometry3d T_B_L = T_B_I; // * state.L2I_isometry();

    br.sendTransform(toTF(T_M_B, "global", "base_link", stamp));
    br.sendTransform(toTF(T_B_I, "base_link", "imu_link", stamp));
    br.sendTransform(toTF(T_B_L, "base_link", "lidar_link", stamp));
}




void fill_config(Config &cfg, rclcpp::Node *node) {

    node->get_parameter("gtsam_debug", cfg.gtsam_debug);
    

    node->get_parameter("topics.input.imu", cfg.topics.input.imu);
    node->get_parameter("topics.input.cones", cfg.topics.input.cones);
    node->get_parameter("topics.output.state", cfg.topics.output.state);
    node->get_parameter("topics.output.cones", cfg.topics.output.cones);

    std::vector<double> bA, bG;
    node->get_parameter("bias.accel", bA);
    node->get_parameter("bias.gyro", bG);
    cfg.bias.accel = Eigen::Vector3d(bA[0], bA[1], bA[2]);
    cfg.bias.gyro = Eigen::Vector3d(bG[0], bG[1], bG[2]);
    
    node->get_parameter("bias.gravity", cfg.bias.gravity);

    node->get_parameter("maxSqDist", cfg.maxSqDist);
    node->get_parameter("closingDist", cfg.closingDist);


    node->get_parameter("covariance.gyro", cfg.cov.gyro);
    node->get_parameter("covariance.accel", cfg.cov.accel);
    node->get_parameter("covariance.biasGyro", cfg.cov.biasG);
    node->get_parameter("covariance.biasAccel", cfg.cov.biasA);
    node->get_parameter("covariance.process", cfg.cov.process);

    node->get_parameter("covariance.initial.pose", cfg.cov.initial.pose);
    node->get_parameter("covariance.initial.lidar", cfg.cov.initial.lidar); 
    node->get_parameter("covariance.initial.velocity", cfg.cov.initial.vel);
    node->get_parameter("covariance.initial.bias", cfg.cov.initial.bias);


    node->get_parameter("ISAM.skip", cfg.isam.skip);
    node->get_parameter("ISAM.threshold", cfg.isam.th);
    node->get_parameter("ISAM.PartReliCheck", cfg.isam.relCheck);

    node->get_parameter("calibration.time", cfg.cal.time);
    node->get_parameter("calibration.accel", cfg.cal.accel);
    node->get_parameter("calibration.gyro", cfg.cal.gyro);


    std::vector<double> tL, RL;
    node->get_parameter("lidar2baselink.t", tL);
    node->get_parameter("lidar2baselink.R", RL);

    Eigen::Quaterniond qL =
        Eigen::AngleAxisd(RL[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(RL[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(RL[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    qL.normalize();
    cfg.lidar2baselink = Eigen::Translation3d(Eigen::Vector3d(tL[0], tL[1], tL[2])) * qL;

    std::vector<double> tI, RI;
    node->get_parameter("imu2baselink.t", tI);
    node->get_parameter("imu2baselink.R", RI);

    Eigen::Quaterniond qI =
        Eigen::AngleAxisd(RI[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(RI[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(RI[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    qI.normalize();
    cfg.imu2baselink = Eigen::Translation3d(Eigen::Vector3d(tI[0], tI[1], tI[2])) * qI;


}
