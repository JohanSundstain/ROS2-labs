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

using namespace std::chrono_literals;

class TurtlesManager : public rclcpp::Node {
public:
  TurtlesManager()
  : Node("turtles_manager")
  {
    // параметры
    this->declare_parameter<double>("radius", 1.0);
    this->declare_parameter<int>("direction_of_rotation", 1);
    radius_ = this->get_parameter("radius").as_double();
    direction_ = this->get_parameter("direction_of_rotation").as_int();

    if (direction_ != 1 && direction_ != -1) {
      RCLCPP_WARN(this->get_logger(), "direction_of_rotation must be 1 or -1. Using 1.");
      direction_ = 1;
    }

    // подписки на позы
    pose1_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10,
      std::bind(&TurtlesManager::pose1_cb, this, std::placeholders::_1));

    pose2_sub_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle2/pose", 10,
      std::bind(&TurtlesManager::pose2_cb, this, std::placeholders::_1));

    // паблишер cmd_vel для turtle2
    turtle2_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);

    // tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // spawn client для turtle2
    spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");

    // запускаем таймер для update (publish tf и управление)
    timer_ = this->create_wall_timer(20ms, std::bind(&TurtlesManager::on_timer, this));

    // пытаемся создать turtle2 (если уже создана - сервис вернёт ошибку)
    spawn_turtle2_if_needed();
    last_time_ = this->now();
  }

private:
  // параметры
  double radius_;
  int direction_;

  // позы
  turtlesim::msg::Pose::SharedPtr pose1_{nullptr};
  turtlesim::msg::Pose::SharedPtr pose2_{nullptr};

  // угол для вращения морковки
  double carrot_angle_{0.0};
  rclcpp::Time last_time_;

  // ROS сущности
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose1_sub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose2_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle2_cmd_pub_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  void pose1_cb(const turtlesim::msg::Pose::SharedPtr msg) {
    pose1_ = msg;
  }

  void pose2_cb(const turtlesim::msg::Pose::SharedPtr msg) {
    pose2_ = msg;
  }

  void spawn_turtle2_if_needed() {
    // подождать availability сервиса
    if (!spawn_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "Spawn service not available yet.");
      return;
    }

    auto req = std::make_shared<turtlesim::srv::Spawn::Request>();
    req->x = 5.0;
    req->y = 5.0;
    req->theta = 0.0;
    req->name = "turtle2";

    auto result_future = spawn_client_->async_send_request(req,
      [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
        try {
          auto res = future.get();
          RCLCPP_INFO(this->get_logger(), "Spawned turtle: %s", res->name.c_str());
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->get_logger(), "Failed to spawn turtle2 (maybe exists): %s", e.what());
        }
      });
  }

  void publish_tf(const std::string &child_frame, double x, double y, double theta) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "world";
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
    double dt = (now - last_time_).seconds();
    if (dt <= 0.0) dt = 0.001;
    last_time_ = now;
	radius_ = this->get_parameter("radius").as_double();
    direction_ = this->get_parameter("direction_of_rotation").as_int();


    // публикуем tf для turtle1 и turtle2, если есть позы
    if (pose1_) {
      publish_tf("turtle1", pose1_->x, pose1_->y, pose1_->theta);
    }
    if (pose2_) {
      publish_tf("turtle2", pose2_->x, pose2_->y, pose2_->theta);
    }

    // обновляем угол морковки
    const double angular_speed = 0.8; // рад/с (можно сделать параметром)
    carrot_angle_ += direction_ * angular_speed * dt;

    // вычисляем положение морковки относительно turtle1
    double carrot_x = 0.0, carrot_y = 0.0, carrot_theta = 0.0;
    if (pose1_) {
      carrot_x = pose1_->x + radius_ * std::cos(carrot_angle_);
      carrot_y = pose1_->y + radius_ * std::sin(carrot_angle_);
      // ориентация морковки укажем как тангенциальную (дополнительно)
      carrot_theta = carrot_angle_ + M_PI_2; // так, чтобы "морковка" смотрела по касательной
    } else {
      // если нет pose1, ставим в центр
      carrot_x = radius_ * std::cos(carrot_angle_);
      carrot_y = radius_ * std::sin(carrot_angle_);
    }

    // публикуем tf carrot
    publish_tf("carrot", carrot_x, carrot_y, carrot_theta);

    // управление turtle2 чтобы следовать за carrot
    if (pose2_) {
      follow_carrot(carrot_x, carrot_y);
    }
  }

  void follow_carrot(double target_x, double target_y) {
    // простой P-контроллер
    double dx = target_x - pose2_->x;
    double dy = target_y - pose2_->y;
    double distance = std::hypot(dx, dy);
    double angle_to_target = std::atan2(dy, dx);
    double angle_err = normalize_angle(angle_to_target - pose2_->theta);

    double kp_lin = 1.0;
    double kp_ang = 6.0;

    geometry_msgs::msg::Twist cmd;
    // если большой угол — поворачиваем на месте
    if (std::fabs(angle_err) > 0.4) {
      cmd.linear.x = 0.0;
      cmd.angular.z = kp_ang * angle_err;
    } else {
      cmd.linear.x = std::min(kp_lin * distance, 2.0);
      cmd.angular.z = kp_ang * angle_err;
    }
    turtle2_cmd_pub_->publish(cmd);
  }

  double normalize_angle(double a) {
    while (a > M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurtlesManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
