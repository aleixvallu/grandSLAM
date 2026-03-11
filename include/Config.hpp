#pragma once

#include <string>
#include <Eigen/Geometry>



struct Config {

    bool gtsam_debug;
    
    struct {
        struct {
            std::string imu;
            std::string cones;    
        } input;
        struct {
            std::string state;
            std::string cones;
        } output;       
        
    } topics;

    struct {
        double gravity;
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
    } bias;

    struct{
        double pose;
        double lidar;
        double vel;
        double gyro;
        double accel;
        double biasG;
        double biasA;
        double process;
    } cov;

    struct{
        int skip;
        double th;
    } isam;

    struct{
        double time;
        bool accel;
        bool gyro;
    } cal;

    double maxSqDist;
    
    Eigen::Isometry3d lidar2baselink;
    Eigen::Isometry3d imu2baselink;

    
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