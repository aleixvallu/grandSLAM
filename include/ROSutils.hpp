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


Cones fromROS(const cat_msgs::msg::ConeArray::ConstSharedPtr &msg, const Eigen::Isometry3d &lidar2imu) {
    
    Config &cfg = Config::getInstance();
    
    Cones cones;
    for (const auto &p : msg->cones) {
        Cone cone(p.position_base_link.x, p.position_base_link.y,  p.position_base_link.z, p.confidence);
        cone = lidar2imu * cone;
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

    // msg.twist.twist = tf2::toMsg(state.v_w());
    // TODO: mirar com posar w

    // LIMO
    auto &T_B_I = cfg.imu2baselink;
    Eigen::Matrix3d R_BI = T_B_I.linear();
    Eigen::Vector3d t_BI = T_B_I.translation();


    auto V_W = state.v_w();
    Eigen::Vector3d w_B = R_BI * V_W.tail<3>();
    msg.twist.twist.angular.x = w_B.x();
    msg.twist.twist.angular.y = w_B.y();
    msg.twist.twist.angular.z = w_B.z();

    Eigen::Vector3d v_B = R_BI * state.R().toRotationMatrix().transpose() * V_W.head<3>() + t_BI.cross(w_B);
    msg.twist.twist.linear.x = v_B.x();
    msg.twist.twist.linear.y = v_B.y();
    msg.twist.twist.linear.z = v_B.z();


    return msg;
}


visualization_msgs::msg::MarkerArray toROS(const Cones &cones, const rclcpp::Time &stamp) {
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
        // msg.markers[i].color.r = 0.5f; 
        // msg.markers[i].color.g = 0.5f;
        float confidence = cones[i].confidence;
        msg.markers[i].color.r =  (confidence < 0.75f) ? 1.f : 1.f - confidence;
        msg.markers[i].color.g = (confidence < 0.75f) ? confidence : 1.f;
        msg.markers[i].color.b = 0.0f;

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
    Eigen::Isometry3d T_B_L = T_B_I * state.L2I_isometry();

    br.sendTransform(toTF(T_M_B, "global", "base_link", stamp));
    br.sendTransform(toTF(T_B_I, "base_link", "imu_link", stamp));
    br.sendTransform(toTF(T_B_L, "base_link", "lidar_link", stamp));
}




void fill_config(Config &cfg, rclcpp::Node *node) {


    std::vector<std::string> missing;

    auto load_param = [&](const std::string &name, auto &dest) {
        if (!node->get_parameter(name, dest)) {
            missing.push_back(name);
        }
    };



    load_param("gtsam_debug", cfg.gtsam_debug);
    

    load_param("topics.input.imu", cfg.topics.input.imu);
    load_param("topics.input.cones", cfg.topics.input.cones);
    load_param("topics.output.state", cfg.topics.output.state);
    load_param("topics.output.cones", cfg.topics.output.cones);

    std::vector<double> bA, bG;
    load_param("bias.accel", bA);
    load_param("bias.gyro", bG);
    cfg.bias.accel = Eigen::Vector3d(bA[0], bA[1], bA[2]);
    cfg.bias.gyro = Eigen::Vector3d(bG[0], bG[1], bG[2]);
    
    load_param("bias.gravity", cfg.bias.gravity);

    double dist;
    load_param("asociationDist", dist);
    cfg.maxSqDist = dist * dist;
    load_param("closingDist", cfg.closingDist);


    load_param("covariance.gyro", cfg.cov.gyro);
    load_param("covariance.accel", cfg.cov.accel);
    load_param("covariance.biasGyro", cfg.cov.biasG);
    load_param("covariance.biasAccel", cfg.cov.biasA);
    load_param("covariance.process", cfg.cov.process);

    load_param("covariance.initial.pose", cfg.cov.initial.pose);
    load_param("covariance.initial.lidar", cfg.cov.initial.lidar); 
    load_param("covariance.initial.velocity", cfg.cov.initial.vel);
    load_param("covariance.initial.bias", cfg.cov.initial.bias);


    load_param("ISAM.skip",      cfg.isam.skip);
    load_param("ISAM.lag",       cfg.isam.lag);

    load_param("ISAM.relinearizeThreshold.pose",     cfg.isam.threshold.pose);
    load_param("ISAM.relinearizeThreshold.vel",      cfg.isam.threshold.vel);
    load_param("ISAM.relinearizeThreshold.bias",     cfg.isam.threshold.bias);
    load_param("ISAM.relinearizeThreshold.landmark", cfg.isam.threshold.landmark);

    load_param("calibration.time", cfg.cal.time);
    load_param("calibration.accel", cfg.cal.accel);
    load_param("calibration.gyro", cfg.cal.gyro);


    std::vector<double> tL, RL;
    load_param("lidar2baselink.t", tL);
    load_param("lidar2baselink.R", RL);

    Eigen::Quaterniond qL =
        Eigen::AngleAxisd(RL[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(RL[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(RL[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    qL.normalize();
    cfg.lidar2baselink = Eigen::Translation3d(Eigen::Vector3d(tL[0], tL[1], tL[2])) * qL;

    std::vector<double> tI, RI;
    load_param("imu2baselink.t", tI);
    load_param("imu2baselink.R", RI);

    Eigen::Quaterniond qI =
        Eigen::AngleAxisd(RI[0] * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(RI[1] * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(RI[2] * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    qI.normalize();
    cfg.imu2baselink = Eigen::Translation3d(Eigen::Vector3d(tI[0], tI[1], tI[2])) * qI;


}
