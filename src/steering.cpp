#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rccar_msgs/msg/twist_stamped_twist_timestamp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

using std::placeholders::_1;

class Steering : public rclcpp::Node {
  public:
    Steering() : Node("steering") {
        auto qos_sensor = rclcpp::SensorDataQoS();

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", qos_sensor, std::bind(&Steering::imu_callback, this, _1));

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 1, std::bind(&Steering::cmd_vel_callback, this, _1));

        pub_cmd_vel_imu_ = this->create_publisher<rccar_msgs::msg::TwistStampedTwistTimestamp>(
            "/cmd_vel_imu", 1);

        RCLCPP_INFO(this->get_logger(), "Steering node has been started.");
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<rccar_msgs::msg::TwistStampedTwistTimestamp>::SharedPtr pub_cmd_vel_imu_;

    sensor_msgs::msg::Imu::SharedPtr latest_imu_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) { latest_imu_ = msg; }

    void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        auto output_msg = rccar_msgs::msg::TwistStampedTwistTimestamp();
        output_msg.header = msg->header;
        output_msg.twist_0 = msg->twist;
        output_msg.twist_1 = msg->twist;

        if (latest_imu_) {
            output_msg.stamp.sec = 0;
            output_msg.stamp.nanosec = 0;
        } else {
            output_msg.stamp = latest_imu_->header.stamp;
        }

        pub_cmd_vel_imu_->publish(output_msg);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Steering>());
    rclcpp::shutdown();
    return 0;
}