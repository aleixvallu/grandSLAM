#pragma once

#include <rclcpp/rclcpp.hpp>

#include "Config.hpp"

void fill_config(Config &cfg, rclcpp::Node *node) {

    node->get_parameter("topics.input.impu", cfg.topics.input.imu);
    node->get_parameter("topics.output.state", cfg.topics.output.state);
}
