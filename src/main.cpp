#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <as_lib/utils/Profiler.hpp>

#include "Config.hpp"
#include "ROSutils.hpp"
#include "Graph.hpp"



class Manager : public rclcpp::Node {

    Imu prevImu;
    double firstStamp = -1.;
    bool calibratedImu = false;

    Graph g;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub;
    // rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr coneSub;
    rclcpp::Subscription<cat_msgs::msg::ConeArray>::SharedPtr coneSub;


    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr statePub;
    // rclcpp::Publisher<cat_msgs::msg::ConeArray>::SharedPtr conesPub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr conesPub;

    // TF Broadcaster
    tf2_ros::TransformBroadcaster tfBr;


  public:

    Manager() : Node("g_slam",
                rclcpp::NodeOptions().
                allow_undeclared_parameters(true).
                automatically_declare_parameters_from_overrides(true)), tfBr(*this) {

        Config &cfg = Config::getInstance();
        fill_config(cfg, this);
    
        imuSub = this->create_subscription<sensor_msgs::msg::Imu>(
            cfg.topics.input.imu, rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&Manager::imuCallback, this, std::placeholders::_1)
        );

        
        // coneSub = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        //     cfg.topics.input.cones, rclcpp::QoS(rclcpp::KeepLast(1)),
        //     std::bind(&Manager::conesCallback, this, std::placeholders::_1)
        // );

        coneSub = this->create_subscription<cat_msgs::msg::ConeArray>(
            cfg.topics.input.cones, rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&Manager::conesCallback, this, std::placeholders::_1)
        );


        statePub = this->create_publisher<nav_msgs::msg::Odometry>(cfg.topics.output.state, 10);
        // conesPub = this->create_publisher<cat_msgs::msg::ConeArray>(cfg.topics.output.cones, 10);
        conesPub = this->create_publisher<visualization_msgs::msg::MarkerArray>(cfg.topics.output.cones, 10);

        // auto tfBr = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // geometry_msgs::msg::TransformStamped tfMsg;
        // tfMsg.header.stamp = this->get_clock()->now();
        // tfMsg.header.frame_id = "global";        // parent
        // tfMsg.child_frame_id = "lidar";       // child

        // Eigen::Vector3d T(cfg.lidar2baselink.translation());
        // tfMsg.transform.translation.x = T.x();
        // tfMsg.transform.translation.y = T.y();
        // tfMsg.transform.translation.z = T.z();

        // tfMsg.transform.rotation = tf2::toMsg(Eigen::Quaterniond(cfg.lidar2baselink.linear()));

        // tfBr->sendTransform(tfMsg);


        // tfMsg.header.frame_id = "global";
        // tfMsg.child_frame_id = "base_link";

        // tfMsg.transform.translation.x = 0.0;
        // tfMsg.transform.translation.y = 0.0;
        // tfMsg.transform.translation.z = 0.0;

        // tfMsg.transform.rotation.x = 0.0;
        // tfMsg.transform.rotation.y = 0.0;
        // tfMsg.transform.rotation.z = 0.0;
        // tfMsg.transform.rotation.w = 1.0;

        // tfBr->sendTransform(tfMsg);
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

                if (cfg.cal.gyro)
                    cfg.bias.gyro = gyroAvg;
                
                // if (cfg.cal.accel)
                //     cfg.bias.accel = accelAvg;
                if (cfg.cal.accel)
                    cfg.bias.accel = accelAvg - (cfg.imu2baselink.rotation() * Eigen::Vector3d::UnitZ() * cfg.bias.gravity);
                    std::cout << "Accel_cal(x: " << cfg.bias.accel.x() << " y: " << cfg.bias.accel.y() << " z: " << cfg.bias.accel.z() << ")" <<  std::endl;


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

        rclcpp::Time stamp = msg->header.stamp;
        statePub->publish(toROS(g, stamp));
        publishTFs(g, tfBr, stamp);
    }

    // void conesCallback(const visualization_msgs::msg::MarkerArray::ConstSharedPtr &msg) {
    void conesCallback(const cat_msgs::msg::ConeArray::ConstSharedPtr &msg) {
    

        if (not calibratedImu) 
            return;

        Cones cones = fromROS(msg);

        g.addCones(cones);

        rclcpp::Time stamp = msg->stamp;
        statePub->publish(toROS(g, stamp));
        conesPub->publish(toROS(g.cones(), stamp));
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