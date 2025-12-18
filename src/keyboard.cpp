#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class KeyboardCmdVel : public rclcpp::Node {
  public:
    KeyboardCmdVel() : Node("keyboard_cmd_vel") {
        pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);

        timer_ = this->create_wall_timer(100ms, // 10 Hz
                                         std::bind(&KeyboardCmdVel::timerCallback, this));

        setNonBlocking();

        std::cout << "\nControl:\n"
                     "  w/s : forward/backward\n"
                     "  a/d : rotate left/right\n"
                     "  space: stop\n"
                     "  q    : quit\n\n";
    }

    ~KeyboardCmdVel() { restoreTerminal(); }

  private:
    geometry_msgs::msg::TwistStamped twist_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TwistStamped t_;

    termios orig_termios_;

    void timerCallback() {
        char c = readKey();
        if (c != 0) {
            processKey(c);
        }

        twist_ = t_; 
        
        if (twist_.twist.linear.x < 0.0f) {
            twist_.twist.angular.z = -twist_.twist.angular.z;
        }
        
        twist_.header.stamp = this->get_clock()->now();
        twist_.header.frame_id = "base_link";

        pub_->publish(twist_);

        print_status();
    }

    float round_one_decimal(float value) { return std::round(value * 10.0f) / 10.0f; }

    void print_status() { std::cout << "\rLinear.x: " << twist_.twist.linear.x << "   Angular.z: " << twist_.twist.angular.z << "     " << std::flush; }

    void processKey(char c) {
        geometry_msgs::msg::TwistStamped old = t_;

        switch (c) {
        case 'w':
            t_.twist.linear.x = round_one_decimal(std::min(t_.twist.linear.x + 0.1, 2.0));
            break;
        case 's':
            t_.twist.linear.x = round_one_decimal(std::max(t_.twist.linear.x - 0.1, -2.0));
            break;
        case 'a':
            t_.twist.angular.z = round_one_decimal(std::min(t_.twist.angular.z + 2, 2.0));
            break;
        case 'd':
            t_.twist.angular.z = round_one_decimal(std::max(t_.twist.angular.z - 2, -2.0));
            break;
        case ' ':
            t_ = geometry_msgs::msg::TwistStamped();
            break;
        case 'q':
            std::cout << "Quit.\n";
            rclcpp::shutdown();
            break;
        }
    }

    // ---- terminal handling -------------------------------------------------

    void setNonBlocking() {
        tcgetattr(STDIN_FILENO, &orig_termios_);

        termios raw = orig_termios_;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);

        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    void restoreTerminal() { tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_); }

    char readKey() {
        char c = 0;
        ssize_t n = read(STDIN_FILENO, &c, 1);
        if (n <= 0) {
            return 0;
        }
        char dump;
        while (read(STDIN_FILENO, &dump, 1) > 0) {
        }

        return c;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyboardCmdVel>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
