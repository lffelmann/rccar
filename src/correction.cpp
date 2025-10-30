#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rccar_msgs/msg/twist_stamped_twist_timestamp_timestamp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include <math.h>
#include <deque>
#include <iostream>


template<typename T, size_t MAX_SIZE>
class FixedList {
private:
    std::deque<T> dq;

public:
    FixedList() = default;

    void push_front(const T& msg) {
        if (dq.size() >= MAX_SIZE) {
            dq.pop_back();
        }
        dq.push_front(msg);
    }
    
    const T& peek_by_index(size_t index) const {
        return dq.at(index);
    }

    bool is_full() const {
        return dq.size() >= MAX_SIZE;
    }
};

using std::placeholders::_1;

class Correction : public rclcpp::Node {
  public:
    Correction() : Node("correction") {
        auto qos_sensor = rclcpp::SensorDataQoS();

        sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", qos_sensor, std::bind(&Correction::imu_callback, this, _1));

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", qos_sensor, std::bind(&Correction::odom_callback, this, _1));

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 1, std::bind(&Correction::cmd_vel_callback, this, _1));

        sub_imu_latency_ = this->create_subscription<sensor_msgs::msg::TimeReference>(
            "/imu_latency", 1, std::bind(&Correction::imu_latency_callback, this, _1));

        pub_cmd_vel_imu_ = this->create_publisher<rccar_msgs::msg::TwistStampedTwistTimestampTimestamp>(
            "/cmd_vel_imu", 1);

        RCLCPP_INFO(this->get_logger(), "Correction node started with alpha = %.3f", alpha_);
        RCLCPP_INFO(this->get_logger(), "Correction node has been started.");
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr sub_imu_latency_;
    rclcpp::Publisher<rccar_msgs::msg::TwistStampedTwistTimestampTimestamp>::SharedPtr pub_cmd_vel_imu_;

    sensor_msgs::msg::Imu::SharedPtr latest_imu_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    sensor_msgs::msg::TimeReference::SharedPtr latest_imu_latency_;
    FixedList<geometry_msgs::msg::TwistStamped::SharedPtr, 100> cmd_vel_list_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) { latest_imu_ = msg; }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { latest_odom_ = msg; }
    void imu_latency_callback(const sensor_msgs::msg::TimeReference::SharedPtr msg) { latest_imu_latency_ = msg; }

    double time_ref_to_float(const sensor_msgs::msg::TimeReference::SharedPtr msg) {
        return msg->time_ref.sec + msg->time_ref.nanosec * 1e-9;
    }

    void cmd_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        auto output_msg = rccar_msgs::msg::TwistStampedTwistTimestampTimestamp();
        output_msg.header = msg->header;
        output_msg.twist_0 = msg->twist;

        double alpha = alpha_;

        if (!latest_imu_) {
            output_msg.stamp_0.sec = 0;
            output_msg.stamp_0.nanosec = 0;
        } else {
            output_msg.stamp_0 = latest_imu_->header.stamp;
        }

        if (!latest_odom_) {
            output_msg.stamp_1.sec = 0;
            output_msg.stamp_1.nanosec = 0;
        } else {
            output_msg.stamp_1 = latest_odom_->header.stamp;
        }

        if (!latest_imu_latency_ || !latest_imu_ || !cmd_vel_list_.is_full()) {
            output_msg.twist_1.angular.z = 999.9;
            cmd_vel_list_.push_front(msg);
            pub_cmd_vel_imu_->publish(output_msg);
            RCLCPP_INFO(this->get_logger(), "Published to /cmd_vel_imu without correction");
            return;
        }

        sensor_msgs::msg::Imu::SharedPtr latest_imu = latest_imu_;

        double delta_t_imu = time_ref_to_float(latest_imu_latency_);
        double delta_t_cmd = 0.1;
         int cmds_used = static_cast<int>(delta_t_imu / delta_t_cmd);
        double delta_t_calc = fmod(delta_t_imu, delta_t_cmd);

        double w = latest_imu->orientation.w;
        double x = latest_imu->orientation.x;
        double y = latest_imu->orientation.y;
        double z = latest_imu->orientation.z;

        double psi = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        
        for (int i = cmds_used; i>0; i--) {
            double psi_vel_cmd = cmd_vel_list_.peek_by_index(i)->twist.angular.z;
            psi += psi_vel_cmd * delta_t_calc;
            delta_t_calc = delta_t_cmd;
        }

        output_msg.twist_1.angular.z = psi;

        cmd_vel_list_.push_front(msg);
        pub_cmd_vel_imu_->publish(output_msg);
        RCLCPP_INFO(this->get_logger(), "Published to /cmd_vel_imu");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Correction>());
    rclcpp::shutdown();
    return 0;
}
