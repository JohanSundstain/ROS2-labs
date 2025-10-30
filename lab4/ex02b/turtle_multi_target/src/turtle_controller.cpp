// src/turtles_manager.cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "current_target/msg/current_target.hpp"
#include "current_target/srv/current_target_service.hpp"

using namespace std::chrono_literals;

class TurtleController : public rclcpp::Node {
public:
	TurtleController() : Node("turtle_controller")
	{

		this->turtle2_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

		this->sub_target_ = this->create_subscription<current_target::msg::CurrentTarget>(
			"/current_target",
			10,
			std::bind(&TurtleController::target_listener, this, std::placeholders::_1)
		);
		
		// tf broadcaster
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
		this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

		// запускаем таймер для update (publish tf и управление)
		timer_ = this->create_wall_timer(50ms, std::bind(&TurtleController::on_timer, this));

		last_time_ = this->now();
	}

private:
	rclcpp::Time last_time_;

	// ROS сущности
	rclcpp::Subscription<current_target::msg::CurrentTarget>::SharedPtr sub_target_ {nullptr};
	current_target::msg::CurrentTarget::SharedPtr current_target_msg_ {nullptr};

	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle2_cmd_pub_;
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	void on_timer()
	{		
		if (this->current_target_msg_)
		{
			try
			{
				geometry_msgs::msg::Twist cmd;
				std::string target_name = this->current_target_msg_->target_name;

				auto transform = tf_buffer_->lookupTransform("turtle2", target_name, tf2::TimePointZero); // from, to
				double distance = std::hypot(transform.transform.translation.x, transform.transform.translation.y);
				double angle_to_target = std::atan2(transform.transform.translation.y, transform.transform.translation.x);

				cmd.linear.x = 1.4 * distance;
				cmd.angular.z = 1.1 * angle_to_target;

				turtle2_cmd_pub_->publish(cmd);			
			}
			catch (tf2::TransformException &ex)
			{
				RCLCPP_WARN(this->get_logger(), "%s", ex.what());
			}
		} 

  	}

	void target_listener(const current_target::msg::CurrentTarget::SharedPtr msg)
	{
		this->current_target_msg_ = msg;
	}
	
};

int main(int argc, char **argv) 
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TurtleController>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
