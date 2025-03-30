#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <chrono>

using namespace std::chrono_literals;

class TargetPublisher : public rclcpp::Node {
public:
    TargetPublisher() : Node("target_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("get_point", 10);
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TargetPublisher::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Target Publisher has been started");
    }

private:
    void timer_callback() {
        auto message = geometry_msgs::msg::Point();
        
        // For testing, we'll publish different target positions
        static float x = 0.0;
        static float y = 0.0;
        static float z = 0.0;
        
        // Simulate target movement
        x += 0.1;
        if (x > 2.0) x = 0.0;
        
        message.x = x;
        message.y = y;
        message.z = z;
        
        RCLCPP_INFO(this->get_logger(), "Publishing target: x=%.2f, y=%.2f, z=%.2f",
            message.x, message.y, message.z);
            
        publisher_->publish(message);
    }
    
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TargetPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 