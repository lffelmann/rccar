#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rccar_msgs/msg/rccar_corr1_time2.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class Bypass : public rclcpp::Node {
  public:
    Bypass() : Node("bypass") {
        auto qos_sensor = rclcpp::SensorDataQoS();

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", qos_sensor, std::bind(&Bypass::imu_callback, this, _1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos_sensor, std::bind(&Bypass::odom_callback, this, _1));

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 1, std::bind(&Bypass::cmd_vel_callback, this, _1));

        pub_cmd_vel_imu_ = this->create_publisher<rccar_msgs::msg::RccarCorr1Time2>(
            "/cmd_vel_corr", 1);

        RCLCPP_INFO(this->get_logger(), "Bypass node has been started.");
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
    rclcpp::Publisher<rccar_msgs::msg::RccarCorr1Time2>::SharedPtr pub_cmd_vel_imu_;

    sensor_msgs::msg::Imu::SharedPtr latest_imu_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) { latest_imu_ = msg; }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { latest_odom_ = msg; }

    void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        auto output_msg = rccar_msgs::msg::RccarCorr1Time2();
        output_msg.header = msg->header;
        output_msg.twist = msg->twist;
        output_msg.yaw_direction = 999.0f; // not used in this node

        if (!latest_imu_) {
            output_msg.timestamp_imu.sec = 0;
            output_msg.timestamp_imu.nanosec = 0;
        } else {
            output_msg.timestamp_imu = latest_imu_->header.stamp;
        }

        if (!latest_odom_) {
            output_msg.timestamp_odom.sec = 0;
            output_msg.timestamp_odom.nanosec = 0;
        } else {
            output_msg.timestamp_odom = latest_odom_->header.stamp;
        }

        pub_cmd_vel_imu_->publish(output_msg);
        RCLCPP_INFO(this->get_logger(), "Published to /cmd_vel_corr");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Bypass>());
    rclcpp::shutdown();
    return 0;
}
