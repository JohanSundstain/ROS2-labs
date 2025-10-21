#include "rclcpp/rclcpp.hpp"
#include "my_interface/srv/full_name_sum_service.hpp"

#include <memory>
#include <string>

void add(const std::shared_ptr<my_interface::srv::FullNameSumService::Request> request,
          std::shared_ptr<my_interface::srv::FullNameSumService::Response>      response)
{
  response->full_name = request->last_name + std::string{" "} + request->name + std::string{" "} + request->first_name;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nlast_name: %s\nname: %s\nfirst_name: %s",
                request->last_name.c_str(), request->name.c_str(), request->last_name.c_str());
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("concat_server");

  rclcpp::Service<my_interface::srv::FullNameSumService>::SharedPtr service =
    node->create_service<my_interface::srv::FullNameSumService>("concat", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to add two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}