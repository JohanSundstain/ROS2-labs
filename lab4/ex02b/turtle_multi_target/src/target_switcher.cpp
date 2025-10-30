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
#include "current_target/msg/current_target.hpp"
#include "current_target/msg/pressed_key.hpp"

using namespace std::chrono_literals;

class TurtleSwitcher : public rclcpp::Node {
public:
	TurtleSwitcher() : Node("turtle_switcher")
	{
		// параметры
		this->declare_parameter<double>("switch_threshold", 0.3);
		switch_threshold_ = this->get_parameter("switch_threshold").as_double();

		// подписки на позы
		pose1_sub_ = this->create_subscription<turtlesim::msg::Pose>(
		"/turtle1/pose", 10,
		std::bind(&TurtleSwitcher::pose1_cb, this, std::placeholders::_1));

		pose2_sub_ = this->create_subscription<turtlesim::msg::Pose>(
		"/turtle2/pose", 10,
		std::bind(&TurtleSwitcher::pose2_cb, this, std::placeholders::_1));

		pose3_sub_ = this->create_subscription<turtlesim::msg::Pose>(
		"/turtle3/pose", 10,
		std::bind(&TurtleSwitcher::pose3_cb, this, std::placeholders::_1));

		sub_key_ = this->create_subscription<current_target::msg::PressedKey>(
		"/pressed_key", 10,
		std::bind(&TurtleSwitcher::key_handler, this, std::placeholders::_1));

		// tf broadcaster
		this->tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
		this->tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
		this->tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_);

		this->pub_target_ = this->create_publisher<current_target::msg::CurrentTarget>("/current_target", 10);

		// запускаем таймер для update (publish tf и управление)
		timer_ = this->create_wall_timer(100ms, std::bind(&TurtleSwitcher::on_timer, this));

		last_time_ = this->now();
		carrot_radius_ = 1.0;
		current_target_ = "carrot1";

		pressed_key_ = std::make_shared<current_target::msg::PressedKey>();
		pressed_key_->key = "";

	}

private:
	// параметры
	double switch_threshold_;
	double carrot_radius_;
	std::string current_target_;

	rclcpp::Time last_time_;

	// ROS сущности

	rclcpp::Publisher<current_target::msg::CurrentTarget>::SharedPtr pub_target_;

	turtlesim::msg::Pose::SharedPtr pose1_{nullptr};
	turtlesim::msg::Pose::SharedPtr pose2_{nullptr};
	turtlesim::msg::Pose::SharedPtr pose3_{nullptr};
	current_target::msg::PressedKey::SharedPtr pressed_key_{nullptr};

	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose1_sub_;
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose2_sub_;
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose3_sub_;

	rclcpp::Subscription<current_target::msg::PressedKey>::SharedPtr sub_key_;
		
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

	void on_timer() {
		// вычисляем dt
		rclcpp::Time now = this->now();
		double x = now.seconds() * M_PI / 10;
	
		this->switch_threshold_ = this->get_parameter("switch_threshold").as_double();

		if (this->pose1_)
		{
			publish_tf("turtle1", "world", this->pose1_->x, this->pose1_->y, this->pose1_->theta);
		}

		if (this->pose2_)
		{
			publish_tf("turtle2", "world", this->pose2_->x, this->pose2_->y, this->pose2_->theta);
		}

		if (this->pose3_)
		{
			publish_tf("turtle3", "world", this->pose3_->x, this->pose3_->y, this->pose3_->theta);
		}

		publish_tf("static_target", "world", 8.0, 2.0, 0);

		publish_tf("carrot1", "turtle1", carrot_radius_ * sin(x), carrot_radius_ * cos(x), 0);
		publish_tf("carrot2", "turtle3", carrot_radius_ * sin(x), carrot_radius_ * cos(x), 0);

		try
		{
			auto transform = tf_buffer_->lookupTransform("turtle2", this->current_target_, tf2::TimePointZero); // from, to
			double distance = std::hypot(transform.transform.translation.x, transform.transform.translation.y);
			double angle_to_target = std::atan2(transform.transform.translation.y, transform.transform.translation.x);

			if (distance <= this->switch_threshold_)
			{
				next_target();
				return;
			}

			if (this->pressed_key_)
			{
				if (this->pressed_key_->key == "n")
				{
					next_target();
					this->pressed_key_->key == "";
					return;
				}
			}

			current_target::msg::CurrentTarget msg;
			msg.target_name = this->current_target_;
			msg.target_x = transform.transform.translation.x;
			msg.target_y = transform.transform.translation.y;
			msg.distance_to_target = distance;
			this->pub_target_->publish(msg);
		
		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_WARN(this->get_logger(), "%s", ex.what());
		}
  	}

	void next_target()
	{
		if (this->current_target_ == "carrot1")
		{
			this->current_target_ = "carrot2";
		}
		else if (this->current_target_ == "carrot2")
		{
			this->current_target_ = "static_target";
		}
		else if (this->current_target_ == "static_target")
		{
			this->current_target_ = "carrot1";
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

	void pose3_cb(const turtlesim::msg::Pose::SharedPtr msg) 
	{
		pose3_ = msg;
	}

	void key_handler(const current_target::msg::PressedKey::SharedPtr msg) 
	{
		this->pressed_key_ = msg;
	}

};

int main(int argc, char **argv) 
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<TurtleSwitcher>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
