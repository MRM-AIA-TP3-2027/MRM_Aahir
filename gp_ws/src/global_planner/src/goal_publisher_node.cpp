/**
 * @file goal_publisher_node.cpp
 * @brief One-shot helper that publishes a NavSatFix goal to /goal_gps.
 *
 * Usage (via launch param or ros2 run):
 *   goal_latitude  / goal_longitude parameters set the target coordinate.
 *
 * The node publishes once (with a small delay to let subscribers connect)
 * then shuts itself down.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <chrono>

using namespace std::chrono_literals;

class GoalPublisher : public rclcpp::Node
{
public:
    GoalPublisher()
    : Node("goal_publisher"), published_(false)
    {
        declare_parameter("goal_latitude",  12.9726);   // ~100 m north of default
        declare_parameter("goal_longitude", 77.5956);

        goal_lat_ = get_parameter("goal_latitude").as_double();
        goal_lon_ = get_parameter("goal_longitude").as_double();

        pub_ = create_publisher<sensor_msgs::msg::NavSatFix>("/goal_gps", 10);

        // Small delay so subscribers have time to connect
        timer_ = create_wall_timer(1500ms, [this]() {
            if (published_) return;
            published_ = true;

            sensor_msgs::msg::NavSatFix msg;
            msg.header.stamp    = this->now();
            msg.header.frame_id = "map";
            msg.latitude        = goal_lat_;
            msg.longitude       = goal_lon_;
            msg.altitude        = 0.0;
            msg.status.status   = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

            pub_->publish(msg);
            RCLCPP_INFO(get_logger(),
                "Published goal GPS: (%.6f, %.6f)", goal_lat_, goal_lon_);

            // Shutdown after publishing
            rclcpp::shutdown();
        });
    }

private:
    double goal_lat_, goal_lon_;
    bool   published_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisher>());
    return 0;
}
