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
                setDesiredPose(0.084, -0.332, 0.0, 0.0, 0.0, -0.250347, 0.968156);

            }else if (key == '1') {

                cout<<"Go to 1"<<endl;  
                setDesiredPose(-0.690582, 2.98624, 0.0, 0.0, 0.0, 0.475004, 0.879984);

            }else if (key == '2') {

                cout<<"Go to 2"<<endl;  
                setDesiredPose(3.36312, 1.11576, 0.0, 0.0, 0.0, -0.855573, 0.517683);

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
