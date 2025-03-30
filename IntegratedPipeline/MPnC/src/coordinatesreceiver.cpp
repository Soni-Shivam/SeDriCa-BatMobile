#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <vector>
#include <cmath>

// PLEASE READ ALL COMMENTS AND COMPLETE THE CODE , IM TIRED OF FUXING ROS

using std::placeholders::_1;
float threshold = 0.6, theta =0 ,epsilonA = 0.1, epsilonB = 0.1, linear_speed, angular_speed;
bool target_found=false ,moving=false;


struct Point{
    float x;
    float y ;
    float z;

};

Point target,endpnt,camera,Controls_msg; 

void calculateendpnt(){
    float bufferdist= 0.6; //60 cm
    theta = atan((target.x+camera.x)/(target.z+camera.z)); // Confirm transformation
    endpnt.x= (target.x+camera.x)- bufferdist*sin(theta);
    endpnt.z= (target.z+camera.z)- bufferdist*cos(theta);
}

int sendVelAndTwist(float linear_speed, float angular_speed) {


    return 0;
    // 1) forward/backward/stop : +1/-1/0
    // 2) bot rotate angle : radian 
    // 3) cannon shoot boolean
    
    //these 3 numbers are bein sent via the below node as a Point from the geometry library (cuz im too lazy to define a new topic interface) 

    // TODO :  use this function as the logic to do all computations and store the the results as x,y,z coordinates of the Point Controls_msg



}


class CoordinateSubscriber : public rclcpp::Node
{
public:
    CoordinateSubscriber()
    : Node("coordinate_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "get_point", 10, std::bind(&CoordinateSubscriber::callback, this, _1));
    }

private:
    void callback(const geometry_msgs::msg::Point::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received coordinates: x=%.2f, y=%.2f, z=%.2f",
        msg->x, msg->y, msg->z);
        target.x = msg->x;
        target.y = msg->y;
        target.z = msg->z;

        //target location is wrt camera
        if(target.z!=0){ // found target
            target_found=true;
        }
        else{ // search
            sendVelAndTwist;
        }
        if(target_found && !moving){
            calculateendpnt();
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
    ControlsPublisher() : Node("control_publisher") , count_(0){
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("Control_instruction", 10);
        timer_ = this->create_wall_timer(
        500ms, std::bind(&ControlsPublisher::control_callback, this));
    }

    private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;

    void control_callback(){
        auto msg = geometry_msgs::msg::Point()

        msg.x = Controls_msg.x;
        msg.y =Controls_msg.y;
        msg.z = Controls_msg.z;
        RCLCPP_INFO(this->get_logger(), "Publishing instructions : move=%.2f, turn=%.2f, shoot=%.2f", msg.x, msg.y, msg.z);
        publisher_->publish(message);
        

    }

}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoordinateSubscriber>());

    rclcpp::shutdown();
    return 0;
}
