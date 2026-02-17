
#include "Config.hpp"
#include "ROSutils.hpp"
#include "Graph.hpp"

class Manager : public rclcpp::Node {

    Imu prevImu;
    bool firstCallback = true;

    Graph g;

  public:

    Manager() : Node("g_slam",
                rclcpp::NodeOptions().
                allow_undeclared_parameters(true).
                automatically_declare_parameters_from_overrides(true)) {

        Config &cfg = Config::getInstance();
        fill_config(cfg, this);
        

    }



    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
        
        Imu imu = fromROS(msg);

        if(firstCallback) {
            prevImu = imu;
            firstCallback = false;
            return;
        }

        double dt = imu.stamp - prevImu.stamp;

        g.addImu(imu, dt);


        prevImu = imu;

    }

    void cones_callback(const geometry_msgs::msg::PoseArray::ConstSharedPtr& msg) {

        Cones cones = fromROS(msg);

        // g.addCones(cones);

    }

};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<Manager>();
}