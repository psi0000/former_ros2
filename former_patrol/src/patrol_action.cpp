#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <future>
#include <memory>
#include <chrono>
#include <string>
#include <iostream>
#include <list>
using namespace std;

string id="0";
int X=0;
int Y=0;
int W=0;
list<int> lt;

void Point(list<int> i){
    
    switch (i.front())
    {
    case 1:
         
        cout<<"Go to 1"<<endl;
        id="map";
        X=3.01;
        Y=0.686;
        W=0.00178; 
        break;
    

    case 2:
        cout<<"Go to 2"<<endl;
        id="map";
        X=0.2; 
        Y=2.0;
        W=1.0; 
        break;
    

    case 3:
        cout<<"Go to 3"<<endl;
        id="map";
        X=0.0;
        Y=0.0;
        W=1.0; 
        break;
    
    default:
        cout<<"End"<<endl;
        rclcpp::shutdown();
        break;
    }

}


class MoveActionClient : public rclcpp::Node {
public:

    using MovetoGoal = nav2_msgs::action::NavigateToPose;
    using GoalHandleMovetoGoal = rclcpp_action::ClientGoalHandle<MovetoGoal>;

    MoveActionClient(list<int> i)
    : Node("patrol_action_client")
    {
        this->client_ptr_ = rclcpp_action::create_client<MovetoGoal>(
        this,
        "navigate_to_pose");
        this->sendGoal(i);
    }

    void sendGoal(list<int> i) {
        using namespace std::placeholders;
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }
        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        
                Point(i);
                goal_msg.pose.header.frame_id = id;
                goal_msg.pose.pose.position.x = X;  // Adjust coordinates as needed
                goal_msg.pose.pose.position.y = Y;
                goal_msg.pose.pose.orientation.w = W; // Quaternion orientation
                
                RCLCPP_INFO(this->get_logger(), "Sending goal");
                
                auto send_goal_options = rclcpp_action::Client<MovetoGoal>::SendGoalOptions();
                send_goal_options.goal_response_callback = std::bind(&MoveActionClient::goal_response_callback, this, _1);
                send_goal_options.result_callback = std::bind(&MoveActionClient::goalResultCallback, this, _1);
                this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
                
        
    }
    
private:
    rclcpp_action::Client<MovetoGoal>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void goal_response_callback(std::shared_future<GoalHandleMovetoGoal::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }


    void goalResultCallback(const GoalHandleMovetoGoal::WrappedResult & result) {
        // result = std::future.get();
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(), "Navigation succeeded");
            lt.pop_front();
            if(lt.size()!=0){
                this->sendGoal(lt);
            }else if(lt.size()==0){

                cout<<"End"<<endl;
                rclcpp::shutdown();
            }
            
        } else {
            RCLCPP_ERROR(get_logger(), "Navigation failed with result code: %d", static_cast<int>(result.code));
        }
    }


};
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    lt.clear();
    //std::cout<<argc<<std::endl;
    
    for(int i=1;i<argc;i++){
        lt.push_back(*argv[i]-48);
        // std::cout<<argv[i]<<std::endl;
        //std::cout<<lt.front()<<std::endl;
    }
    auto action_client=std::make_shared<MoveActionClient>(lt); 
    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}