#include <as_lib/utils/Profiler.hpp>

#include "Config.hpp"
#include "ROSutils.hpp"
#include "Graph.hpp"



class Manager : public rclcpp::Node {

    Imu prevImu;
    int firstStamp = -1;
    bool calibratedImu = false;

    Graph g;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr coneSub;
    // rclcpp::Subscription<cat_msgs::msg::ConeArray>::SharedPtr coneSub;


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr statePub;
    // rclcpp::Publisher<cat_msgs::msg::ConeArray>::SharedPtr conesPub;

  public:

    Manager() : Node("g_slam",
                rclcpp::NodeOptions().
                allow_undeclared_parameters(true).
                automatically_declare_parameters_from_overrides(true)) {

        Config &cfg = Config::getInstance();
        fill_config(cfg, this);
    
        imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
            cfg.topics.input.imu, rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&Manager::imuCallback, this, std::placeholders::_1)
        );

        
        coneSub = this->create_subscription<geometry_msgs::msg::PoseArray>(
            cfg.topics.input.cones, rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&Manager::conesCallback, this, std::placeholders::_1)
        );

        // coneSub = this->create_subscription<cat_msgs::msg::ConeArray>(
        //     cfg.topics.input.cones, rclcpp::QoS(rclcpp::KeepLast(1)),
        //     std::bind(&Manager::conesCallback, this, std::placeholders::_1)
        // );


        statePub = this->create_publisher<nav_msgs::msg::Odometry>(cfg.topics.output.state, 10);
        // conesPub = this->create_publisher<cat_msgs::msg::ConeArray>(cfg.topics.output.cones, 10);


        
    }


    void imuCallback(const sensor_msgs::msg::Imu::ConstSharedPtr &msg) {
        
        Imu imu = fromROS(msg);

        if (firstStamp < 0.)
            firstStamp = imu.stamp;

        if (not calibratedImu) {
            static int N(0);
            static Eigen::Vector3d gyroAvg(0., 0., 0.);
            static Eigen::Vector3d accelAvg(0., 0., 0.);
            Config &cfg = Config::getInstance();

            if ((imu.stamp - firstStamp) < cfg.cal.time) {
                gyroAvg  += imu.ang_vel;
                accelAvg += imu.lin_accel; 
                N++;
                return;
            } else {
                

                accelAvg /= N;
                gyroAvg /= N;
                std::cout << "Accel(x: " << accelAvg.x() << " y: " << accelAvg.y() << " z: " << accelAvg.z() << ")" <<  std::endl;
                std::cout << "GYRO(x: " << gyroAvg.x() << " y: " << gyroAvg.y() << " z: " << gyroAvg.z() << ")" << std::endl;

                
                if (cfg.cal.accel)
                    cfg.bias.accel = accelAvg;

                if (cfg.cal.gyro)
                    cfg.bias.gyro = gyroAvg;

                g.init();
                calibratedImu = true;
            }
            prevImu = imu;
            return;
        }

        double dt = imu.stamp - prevImu.stamp;
        if(dt < 0. ||   dt >= imu.stamp) {
            RCLCPP_ERROR(this->get_logger(), "WRONG IMU, DT < 0");
            return;
        }
        g.addImu(imu, dt);


        prevImu = imu;
    }

    void conesCallback(const geometry_msgs::msg::PoseArray::ConstSharedPtr &msg) {

        if (not calibratedImu) 
            return;

        Cones cones = fromROS(msg);

        g.addCones(cones);

        rclcpp::Time stamp = msg->header.stamp;
        statePub->publish(toROS(g, stamp));

    }

};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<Manager>();

    PROFC_INSTALL(node);
    
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}