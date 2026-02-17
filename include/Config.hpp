#pragma once

#include <string>
#include <Eigen/Geometry>



struct Config {

    struct {
        struct {
            std::string imu;
        } input;
        struct {
            std::string state;
        } output;

        
        
    } topics;
    
    Eigen::Affine3d lidar2baselink;

    
    // Singleton pattern
    static Config &getInstance() {
        static Config *config = new Config();
        return *config;
    }

  private:
    Config() = default;

    // Delete copy/move so extra instances can't be created/moved.
    Config(const Config &) = delete;
    Config &operator=(const Config &) = delete;
    Config(Config &&) = delete;
    Config &operator=(Config &&) = delete;
};