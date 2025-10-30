// src/turtles_manager.cpp
#include <chrono>
#include <cmath>
#include <memory>
#include <string>

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

using namespace std::chrono_literals;

class TimeTravel : public rclcpp::Node {
public:
	TimeTravel() : Node("time_travel")
	{
		// параметры
		this->declare_parameter<double>("delay", 0.3);
		this->delay_ = this->get_parameter("delay").as_double();

		this->turtle2_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
		// подписки на позы
		pose1_sub_ = this->create_subscription<turtlesim::msg::Pose>(
		"/turtle1/pose", 10,
		std::bind(&TimeTravel::pose1_cb, this, std::placeholders::_1));

		pose2_sub_ = this->create_subscription<turtlesim::msg::Pose>(
		"/turtle2/pose", 10,
		std::bind(&TimeTravel::pose2_cb, this, std::placeholders::_1));


		// tf broadcaster
		this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
		this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);


		// запускаем таймер для update (publish tf и управление)
		timer_ = this->create_wall_timer(100ms, std::bind(&TimeTravel::on_timer, this));
		last_time_ = this->now();
	}

private:
	rclcpp::Time last_time_;

	double delay_;
	// ROS сущности
	turtlesim::msg::Pose::SharedPtr pose1_{nullptr};
	turtlesim::msg::Pose::SharedPtr pose2_{nullptr};

	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose1_sub_;
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose2_sub_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle2_cmd_pub_;

		
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

	void publish_tf(const std::string &child_frame, const std::string &header, double x, double y, double theta) 
	{
		geometry_msgs::msg::TransformStamped t;
		t.header.stamp = this->now();
		t.header.frame_id = header;
		t.child_frame_id = child_frame;
		t.transform.translation.x = x;
		t.transform.translation.y = y;
		t.transform.translation.z = 0.0;
		tf2::Quaternion q;
		q.setRPY(0, 0, theta);
		t.transform.rotation.x = q.x();
		t.transform.rotation.y = q.y();
		t.transform.rotation.z = q.z();
		t.transform.rotation.w = q.w();
		tf_broadcaster_->sendTransform(t);
	}

	void on_timer() 
	{

		this->delay_ = this->get_parameter("delay").as_double();

		if (this->pose1_)
		{
			publish_tf("turtle1", "world", this->pose1_->x, this->pose1_->y, this->pose1_->theta);
		}

		if (this->pose2_)
		{
			publish_tf("turtle2", "world", this->pose2_->x, this->pose2_->y, this->pose2_->theta);
		}

		try
		{
			geometry_msgs::msg::Twist cmd;
			rclcpp::Time query_time = this->now() - rclcpp::Duration::from_seconds(this->delay_);
			auto transform = tf_buffer_->lookupTransform("turtle2", "turtle1", query_time); // from, to
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

	// callback area
	void pose1_cb(const turtlesim::msg::Pose::SharedPtr msg) 
	{
		pose1_ = msg;
	}

	void pose2_cb(const turtlesim::msg::Pose::SharedPtr msg)
	{
		pose2_ = msg;
	}

};

int main(int argc, char **argv) 
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TimeTravel>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
