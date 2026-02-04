
#include "Config.hpp"
#include "ROSutils.hpp"

class Manager : public rclcpp::Node {


  public:

    Manager() : Node("g_slam",
                rclcpp::NodeOptions().
                allow_undeclared_parameters(true).
                automatically_declare_parameters_from_overrides(true)) {

        Config &cfg = Config::getInstance();
        fill_config(cfg, this);
        

    }
};

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);

    auto node = std::make_shared<Manager>();
}