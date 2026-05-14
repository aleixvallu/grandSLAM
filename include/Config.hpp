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
        float gravity;
        Eigen::Vector3d accel;
        Eigen::Vector3d gyro;
    } bias;

    struct{
        double gyro;
        double accel;
        double biasG;
        double biasA;
        double process;
        struct {
            double pose;
            double vel;
            double lidar;
            double bias;
        } initial;
    } cov;

    struct{
        int skip;
        bool relCheck;
        double th;
    } isam;

    struct{
        double time;
        bool accel;
        bool gyro;
    } cal;

    double maxSqDist;
    double closingDist;

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