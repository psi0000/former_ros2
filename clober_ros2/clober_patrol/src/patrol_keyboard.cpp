#include "rclcpp/rclcpp.hpp"
#include "clober_msgs/srv/set_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <future>
#include <memory>
#include <chrono>
#include <string>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "nav_msgs/msg/odometry.hpp"
#include <unistd.h>

#include <termios.h>
#include <list>

using namespace std;
using namespace std::placeholders;

class PoseControllerNode : public rclcpp::Node {
public:  
    PoseControllerNode() : Node("patrol_keyboard_client") 
    {
        // Initialize goal_pose_ as a PoseStamped message
        goal_pose_ = std::make_shared<geometry_msgs::msg::PoseStamped>();

        // Create a publisher to publish goal poses
        goal_pose_publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);


        // Start keyboard input handling
        handleKeyboardInput();
    }

private:
    rclcpp::Client<clober_msgs::srv::SetPose>::SharedPtr set_goal_client_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_;

    void setDesiredPose(double x, double y, double z, double orientation_x, double orientation_y, double orientation_z, double orientation_w) {
        
        goal_pose_->header.frame_id = "map";
        goal_pose_->pose.position.x = x;
        goal_pose_->pose.position.y = y;
        goal_pose_->pose.position.z = z;
        goal_pose_->pose.orientation.x = orientation_x;
        goal_pose_->pose.orientation.y = orientation_y;
        goal_pose_->pose.orientation.z = orientation_z;
        goal_pose_->pose.orientation.w = orientation_w;

        RCLCPP_INFO(this->get_logger(), "Sending goal");
                
        RCLCPP_INFO(this->get_logger(), "Send goal_pose x : %f", goal_pose_->pose.position.x);
        RCLCPP_INFO(this->get_logger(), "Send goal_pose y : %f", goal_pose_->pose.position.y);
        RCLCPP_INFO(this->get_logger(), "Send goal_pose z : %f", goal_pose_->pose.orientation.z);
        RCLCPP_INFO(this->get_logger(), "Send goal_pose w : %f \n", goal_pose_->pose.orientation.w);
        
        goal_pose_publisher_->publish(*goal_pose_);
    }


    void handleKeyboardInput() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        char key;
        while (rclcpp::ok()) {
            cout << "where do you want to go? " << endl;
            std::cin >> key;
            cout << "key: " << key << endl;
            if (key == '0') {

                cout<<"Go to InitPose"<<endl;
                setDesiredPose(-0.0846604, -0.231, 0.0, 0.0, 0.0, 0.716745, 0.697335);

            }else if (key == '1') {

                cout<<"Go to 1"<<endl;  
                setDesiredPose(-2.09483, -1.89985, 0.0, 0.0, 0.0, 0.65727, 0.753655);

            }else if (key == '2') {

                cout<<"Go to 2"<<endl;  
                setDesiredPose(-1.78594, 4.41018, 0.0, 0.0, 0.0, 0.013575, 0.999908);

            }else if (key == '3') {

                cout<<"Go to 3"<<endl;  
                setDesiredPose(1.96068, 3.88317, 0.0, 0.0, 0.0, 0.0062032, 0.999981);

            }else if (key == '4') {

                cout<<"Go to 4"<<endl;  
                setDesiredPose(5.27684, 4.01497, 0.0, 0.0, 0.0, -0.727762, 0.68583);

            }else if (key == '5') {

                cout<<"Go to 5"<<endl;  
                setDesiredPose(4.74515, -0.807825, 0.0, 0.0, 0.0, -0.999258, 0.0385052);

            }else if (key == '6') {

                cout<<"Go to 6"<<endl;  
                setDesiredPose(2.01114, -0.908249, 0.0, 0.0, 0.0, -0.996714, 0.0810063);

            }else if(key == 'q'){

                rclcpp::shutdown();
                break;
                
            }else{
                cout<<"Not set_pose key"<<endl;
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseControllerNode>());
    rclcpp::shutdown();
    return 0;
}
