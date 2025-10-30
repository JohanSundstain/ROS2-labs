#include <rclcpp/rclcpp.hpp>
#include "current_target/msg/pressed_key.hpp"
#include <termios.h>
#include <unistd.h>
#include <iostream>

using namespace std::chrono_literals;

class KeyListener : public rclcpp::Node {
public:
	KeyListener() : Node("key_listener")
	{
		pub_key_ = this->create_publisher<current_target::msg::PressedKey>("/pressed_key", 10);
		RCLCPP_INFO(this->get_logger(), "Keyboard listener started. Press 'n' to switch target.");
		configureTerminal();
		timer_ = this->create_wall_timer(50ms, std::bind(&KeyListener::checkKey, this));
	}

	~KeyListener() override {
		restoreTerminal();
	}

private:
	rclcpp::Publisher<current_target::msg::PressedKey>::SharedPtr pub_key_;
	rclcpp::TimerBase::SharedPtr timer_;

	struct termios cooked_, raw_;
	bool terminal_configured_ = false;

	void configureTerminal() {
		tcgetattr(STDIN_FILENO, &cooked_);
		raw_ = cooked_;
		raw_.c_lflag &= ~(ICANON | ECHO);  
		raw_.c_cc[VEOL] = 1;
		raw_.c_cc[VEOF] = 2;
		tcsetattr(STDIN_FILENO, TCSANOW, &raw_);
		terminal_configured_ = true;
	}

	void restoreTerminal() {
		if (terminal_configured_) {
			tcsetattr(STDIN_FILENO, TCSANOW, &cooked_);
		}
	}

	void checkKey() {
		fd_set set;
		struct timeval timeout;

		FD_ZERO(&set);
		FD_SET(STDIN_FILENO, &set);

		timeout.tv_sec = 0;
		timeout.tv_usec = 0;

		int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);

		if (rv > 0) {
			char c;
			if (read(STDIN_FILENO, &c, 1) < 0)
				return;

			current_target::msg::PressedKey msg;
			msg.key = std::string(1, c);
			pub_key_->publish(msg);
		}
	}
};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	auto node = std::make_shared<KeyListener>();
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
