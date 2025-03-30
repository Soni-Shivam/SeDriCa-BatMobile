#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "integratedPackage/msg/control_instructions.hpp"
#include <vector>
#include <cmath>

using std::placeholders::_1;
float threshold = 0.6, theta =0 ,epsilonA = 0.1, epsilonB = 0.1, linear_speed =1 , angular_speed=0.5;
bool target_found=false ,moving=false;

struct Point{
    float x;
    float y;
    float z;
};

Point target,endpnt,camera,Controls_msg; 

void calcEndpoint(){
    float bufferdist= 0.6; //60 cm
    theta = atan((target.x+camera.x)/(target.z+camera.z)); // Confirm transformation
    endpnt.x= (target.x+camera.x)- bufferdist*sin(theta);
    endpnt.z= (target.z+camera.z)- bufferdist*cos(theta);
}

class CoordinateSubscriber : public rclcpp::Node {
public:
    CoordinateSubscriber()
    : Node("coordinate_subscriber"){
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "get_point", 10, std::bind(&CoordinateSubscriber::callback, this, _1));
    }

private:
    void callback(const geometry_msgs::msg::Point::SharedPtr msg) const{
        RCLCPP_INFO(this->get_logger(), "Received coordinates: x=%.2f, y=%.2f, z=%.2f",
        msg->x, msg->y, msg->z);
        target.x = msg->x;
        target.y = msg->y;
        target.z = msg->z;

        //target location is wrt camera
        if (target.z!=0){ // found target
            target_found=true;
        }
        else{ // search
            sendVelAndTwist(0, angular_speed);
        }
        if(target_found && !moving){
            calcEndpoint();
            if (abs(endpnt.z - threshold) > epsilonB) {
                moving = true;
                while(target.x > epsilonA) {
                    // ENSURE No infinite loops
                    sendVelAndTwist(0, angular_speed);
                }
                sendVelAndTwist(linear_speed, 0);
            }
        }
        if (moving) {
            if (abs(endpnt.z - threshold) <= epsilonB){
                sendVelAndTwist(0,0);
                moving = false;
            }
        }
    }
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
};

class ControlsPublisher : public rclcpp::Node{
public:
    ControlsPublisher() : Node("control_publisher") {
        control_publisher = this->create_publisher<integratedPackage::msg::ControlInstructions>(
            "Control_instruction", 10);
    }

    static int sendVelAndTwist(float linear_speed, float angular_speed) {
        auto control_msg = integratedPackage::msg::ControlInstructions();
        control_msg.linear_speed = linear_speed;
        control_msg.angular_speed = angular_speed;
        control_msg.shoot_cannon = false;  // Set to true when shooting is needed
        
        RCLCPP_INFO(rclcpp::get_logger("control_publisher"), 
            "Publishing control instructions: linear_speed=%.2f, angular_speed=%.2f, shoot_cannon=%d", 
            control_msg.linear_speed, control_msg.angular_speed, control_msg.shoot_cannon);
            
        control_publisher->publish(control_msg);
        return 0;
    }
private:
    static rclcpp::Publisher<integratedPackage::msg::ControlInstructions>::SharedPtr control_publisher;
};
// Initialize static member
rclcpp::Publisher<integratedPackage::msg::ControlInstructions>::SharedPtr ControlsPublisher::control_publisher;

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    auto coordinate_subscriber = std::make_shared<CoordinateSubscriber>();
    auto controls_publisher = std::make_shared<ControlsPublisher>();
    rclcpp::spin(coordinate_subscriber);
    rclcpp::shutdown();
    return 0;
}
