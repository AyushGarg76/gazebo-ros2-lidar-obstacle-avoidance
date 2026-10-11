#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

#include <ignition/transport/Node.hh>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/msgs/twist.pb.h>

ignition::transport::Node node;
auto pub = node.Advertise<ignition::msgs::Twist>("/cmd_vel");

// Controller settings
constexpr double STOP_DISTANCE = 1.0;
constexpr double RESUME_DISTANCE = 1.2;

constexpr double FORWARD_SPEED = 0.4;
constexpr double TURN_SPEED = 0.5;

constexpr double FRONT_SECTOR_DEG = 30.0;

bool obstacleDetected = false;

double getMinimumRange(
    const ignition::msgs::LaserScan &_msg,
    double minAngle,
    double maxAngle)
{
    double minimum = std::numeric_limits<double>::infinity();

    for (int i = 0; i < _msg.ranges_size(); ++i)
    {
        const double angle =
            _msg.angle_min() + i * _msg.angle_step();

        // Only examine the requested angular sector.
        if (angle < minAngle || angle > maxAngle)
        {
            continue;
        }

        const double distance = _msg.ranges(i);

        if (std::isfinite(distance) &&
            distance >= _msg.range_min() &&
            distance <= _msg.range_max())
        {
            minimum = std::min(minimum, distance);
        }
    }

    return minimum;
}

double getAverageRange(
    const ignition::msgs::LaserScan &_msg,
    double minAngle,
    double maxAngle)
{
    double total = 0.0;
    int count = 0;

    for (int i = 0; i < _msg.ranges_size(); ++i)
    {
        const double angle =
            _msg.angle_min() + i * _msg.angle_step();

        if (angle < minAngle || angle > maxAngle)
        {
            continue;
        }

        const double distance = _msg.ranges(i);

        if (std::isfinite(distance) &&
            distance >= _msg.range_min() &&
            distance <= _msg.range_max())
        {
            total += distance;
            ++count;
        }
    }

    if (count == 0)
    {
        return 0.0;
    }

    return total / count;
}

void lidarCallback(const ignition::msgs::LaserScan &_msg)
{
    constexpr double DEG_TO_RAD = M_PI / 180.0;

    const double frontAngle =
        FRONT_SECTOR_DEG * DEG_TO_RAD;

    const double frontMinimum =
        getMinimumRange(_msg, -frontAngle, frontAngle);

    const double leftAverage =
        getAverageRange(_msg, 30.0 * DEG_TO_RAD,
                               100.0 * DEG_TO_RAD);

    const double rightAverage =
        getAverageRange(_msg, -100.0 * DEG_TO_RAD,
                               -30.0 * DEG_TO_RAD);

    // Apply hysteresis to avoid rapid switching.
    if (!obstacleDetected && frontMinimum < STOP_DISTANCE)
    {
        obstacleDetected = true;
    }
    else if (obstacleDetected &&
             frontMinimum > RESUME_DISTANCE)
    {
        obstacleDetected = false;
    }

    ignition::msgs::Twist command;

    if (!obstacleDetected)
    {
        command.mutable_linear()->set_x(FORWARD_SPEED);
        command.mutable_angular()->set_z(0.0);

        std::cout << "Path clear: moving forward"
                  << std::endl;
    }
    else
    {
        command.mutable_linear()->set_x(0.0);

        // Turn toward the side with more open space.
        if (leftAverage >= rightAverage)
        {
            command.mutable_angular()->set_z(TURN_SPEED);
            std::cout << "Obstacle ahead: turning left"
                      << std::endl;
        }
        else
        {
            command.mutable_angular()->set_z(-TURN_SPEED);
            std::cout << "Obstacle ahead: turning right"
                      << std::endl;
        }
    }

    pub.Publish(command);
}

int main()
{
    if (!pub)
    {
        std::cerr << "Failed to advertise /cmd_vel"
                  << std::endl;
        return 1;
    }

    if (!node.Subscribe("/lidar", lidarCallback))
    {
        std::cerr << "Failed to subscribe to /lidar"
                  << std::endl;
        return 1;
    }

    std::cout << "Improved LiDAR controller running."
              << std::endl;

    ignition::transport::waitForShutdown();

    return 0;
}