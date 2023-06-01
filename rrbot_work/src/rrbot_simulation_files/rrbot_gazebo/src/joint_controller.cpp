#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <iostream>
#include "custom_interfaces/srv/set_joint_states.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class joint_control : public rclcpp::Node
{
public:
  joint_control() : Node("joint_controller_server"), count_(0)
  {service_ = this->create_service<custom_interfaces::srv::SetJointStates>("joint_control", std::bind(&joint_control::receive_reference_joint_position_from_service, this, _1));}

private:
  void receive_reference_joint_position_from_service(const std::shared_ptr<custom_interfaces::srv::SetJointStates::Request> request)
  {
    ref_pose = {request->rq1,request->rq2,request->rq3};
    subscriber_of_jointstates = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&joint_control::calculate_joint_efforts, this, _1));
    publisher_effort = create_publisher<std_msgs::msg::Float64MultiArray>("/forward_effort_controller/commands", 10);
    reference_value_publisher = create_publisher<std_msgs::msg::Float64MultiArray>("/reference_joint/commands", 10);
  }

  void calculate_joint_efforts(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::double_t position_of_joint[3] = {msg->position[0], msg->position[1], msg->position[2]};
    std::double_t velocity_of_joint[3] = {msg->velocity[0], msg->velocity[1], msg->velocity[2]};
    std::double_t error_caused[3] = {(-position_of_joint[0] + ref_pose[0]), (-position_of_joint[1] + ref_pose[1]), (-position_of_joint[2] + ref_pose[2])};
    std::vector<std::double_t> applying_joint_efforts = {0, 0, 9.8};

    applying_joint_efforts[0] = (gain_by_proportionality[0] * error_caused[0]) - (gain_by_derivative[0] * velocity_of_joint[0]);
    applying_joint_efforts[1] = (gain_by_proportionality[1] * error_caused[1]) - (gain_by_derivative[1] * velocity_of_joint[1]);
    applying_joint_efforts[2] = (gain_by_proportionality[2] * error_caused[2]) - (gain_by_derivative[2] * velocity_of_joint[2]);

    std_msgs::msg::Float64MultiArray message;
    message.data = applying_joint_efforts;
    std_msgs::msg::Float64MultiArray reference_joint_states;
    reference_joint_states.data = ref_pose;

    publisher_effort->publish(message);
    reference_value_publisher->publish(reference_joint_states);
  }

  rclcpp::Service<custom_interfaces::srv::SetJointStates>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_of_jointstates;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_effort;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr reference_value_publisher;

  size_t count_;
  std::vector<std::double_t> ref_pose;

  std::double_t gain_by_proportionality[3] = {60, 50, 90};
  std::double_t gain_by_derivative[3] = {30, 35, 35};

};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<joint_control>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}