#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <chrono>


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class LidarAvoidanceNode : public rclcpp::Node
{
public:
    LidarAvoidanceNode()
        : Node("lidar_avoidance_node")
    {
        scan_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/lidar",
                10,
                std::bind(
                    &LidarAvoidanceNode::scanCallback,
                    this,
                    _1));

        cmd_pub_ =
            this->create_publisher<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10);

        last_scan_time_ = this->now();

        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&LidarAvoidanceNode::watchdogCallback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "LiDAR avoidance node started.");
    }

private:
    static constexpr double STOP_DISTANCE = 1.0;
    static constexpr double RESUME_DISTANCE = 1.2;
    static constexpr double FORWARD_SPEED = 0.4;
    static constexpr double TURN_SPEED = 0.5;

    bool obstacle_detected_ = false;

    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    rclcpp::Time last_scan_time_;
    bool received_first_scan_ = false;

    rclcpp::Subscription<
        sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<
        geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    double sectorAverage(
        const sensor_msgs::msg::LaserScan::SharedPtr scan,
        double start_angle,
        double end_angle)
    {
        double sum = 0.0;
        int count = 0;

        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            const double angle =
                scan->angle_min +
                i * scan->angle_increment;

            if (angle < start_angle || angle > end_angle)
                continue;

            const double range = scan->ranges[i];

            if (std::isfinite(range) &&
                range >= scan->range_min &&
                range <= scan->range_max)
            {
                sum += range;
                ++count;
            }
        }

        if (count == 0)
            return 0.0;

        return sum / count;
    }

    void scanCallback(
        const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        last_scan_time_ = this->now();
        received_first_scan_ = true;
        
        double front_min =
            std::numeric_limits<double>::infinity();

        // Examine the forward sector: -30 to +30 degrees.
        for (size_t i = 0; i < scan->ranges.size(); ++i)
        {
            const double angle =
                scan->angle_min +
                i * scan->angle_increment;

            if (std::abs(angle) > M_PI / 6.0)
                continue;

            const double range = scan->ranges[i];

            if (std::isfinite(range) &&
                range >= scan->range_min &&
                range <= scan->range_max)
            {
                front_min = std::min(front_min, range);
            }
        }

        // Hysteresis prevents rapid switching near the threshold.
        if (!obstacle_detected_ &&
            front_min < STOP_DISTANCE)
        {
            obstacle_detected_ = true;
        }
        else if (obstacle_detected_ &&
                 front_min > RESUME_DISTANCE)
        {
            obstacle_detected_ = false;
        }

        geometry_msgs::msg::Twist cmd;

        if (obstacle_detected_)
        {
            const double left_space =
                sectorAverage(
                    scan, M_PI / 6.0, 5.0 * M_PI / 9.0);

            const double right_space =
                sectorAverage(
                    scan, -5.0 * M_PI / 9.0, -M_PI / 6.0);

            cmd.linear.x = 0.0;

            cmd.angular.z =
                (left_space > right_space)
                    ? TURN_SPEED
                    : -TURN_SPEED;

            RCLCPP_INFO(
                this->get_logger(),
                "Obstacle ahead. Turning %s.",
                cmd.angular.z > 0.0 ? "left" : "right");
        }
        else
        {
            cmd.linear.x = FORWARD_SPEED;
            cmd.angular.z = 0.0;

            RCLCPP_INFO(
                this->get_logger(),
                "Path clear. Moving forward.");
        }

        cmd_pub_->publish(cmd);
    }

    void watchdogCallback()
    {
        if (!received_first_scan_)
        {
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);

            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Waiting for the first LiDAR scan. Stopping robot.");

            return;
        }

        const double elapsed =
            (this->now() - last_scan_time_).seconds();

        if (elapsed > 1.0)
        {
            geometry_msgs::msg::Twist stop;
            cmd_pub_->publish(stop);

            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "LiDAR timeout! Publishing zero velocity.");
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<LidarAvoidanceNode>());

    rclcpp::shutdown();
    return 0;
}