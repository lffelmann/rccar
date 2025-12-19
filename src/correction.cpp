#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rccar_msgs/msg/rccar1_corr2_time.hpp"
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

    size_t size() const {
        return dq.size();
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
            "/imu_latency", qos_sensor, std::bind(&Correction::imu_latency_callback, this, _1));

        pub_cmd_vel_corr_ = this->create_publisher<rccar_msgs::msg::Rccar1Corr2Time>(
            "/cmd_vel_corr", 1);

        RCLCPP_INFO(this->get_logger(), "Correction node has been started.");
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_cmd_vel_;
    rclcpp::Subscription<sensor_msgs::msg::TimeReference>::SharedPtr sub_imu_latency_;
    rclcpp::Publisher<rccar_msgs::msg::Rccar1Corr2Time>::SharedPtr pub_cmd_vel_corr_;

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
        auto output_msg = rccar_msgs::msg::Rccar1Corr2Time();
        output_msg.header = msg->header;
        output_msg.twist = msg->twist;

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

        if (!latest_imu_latency_) {
            RCLCPP_INFO(this->get_logger(), "No latest imu latency");
        }

        if (!latest_imu_) {
            RCLCPP_INFO(this->get_logger(), "No latest imu");
        }

        if (!cmd_vel_list_.is_full()) {
            RCLCPP_INFO(this->get_logger(), "cmd_vel_list_ not full yet Size: %zu", cmd_vel_list_.size());
        }

        if (!latest_imu_latency_ || !latest_imu_ || !cmd_vel_list_.is_full()) {
            output_msg.yaw_direction = 999.0;
            cmd_vel_list_.push_front(msg);
            pub_cmd_vel_corr_->publish(output_msg);
            RCLCPP_INFO(this->get_logger(), "Published to /cmd_vel_corr without correction");
            return;
        }

        cmd_vel_list_.push_front(msg);

        sensor_msgs::msg::Imu::SharedPtr latest_imu = latest_imu_;

        double delta_t_imu = time_ref_to_float(latest_imu_latency_);
        double delta_t_cmd = 0.1;
        int cmds_used = static_cast<int>(delta_t_imu / delta_t_cmd) + 1; // we want to predict where it is when the command is finished
        double delta_t_calc = fmod(delta_t_imu, delta_t_cmd);

        double w = latest_imu->orientation.w;
        double x = latest_imu->orientation.x;
        double y = latest_imu->orientation.y;
        double z = latest_imu->orientation.z;

        double psi = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
        
        for (int i = cmds_used; i>=0; i--) {
            if (cmd_vel_list_.peek_by_index(i)->twist.linear.x == 0.0) {
                psi = 999.0;
                break;
            }
            double psi_vel_cmd = cmd_vel_list_.peek_by_index(i)->twist.angular.z;
            psi += psi_vel_cmd * delta_t_calc;
            delta_t_calc = delta_t_cmd;
        }

        if (psi != 999.0) {
            while (psi > M_PI) {
                psi -= 2.0f * M_PI;
            }
            while (psi < -M_PI) {
                psi += 2.0f * M_PI;
            }
        }

        output_msg.yaw_direction = psi;
        pub_cmd_vel_corr_->publish(output_msg);
        RCLCPP_INFO(this->get_logger(), "Published to /cmd_vel_corr");
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Correction>());
    rclcpp::shutdown();
    return 0;
}
