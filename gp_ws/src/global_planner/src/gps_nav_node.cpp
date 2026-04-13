/**
 * @file gps_nav_node.cpp
 * @brief Global Planner — GPS-only autonomous navigation (no IMU required).
 *
 * Heading is estimated from consecutive GPS positions (course-over-ground),
 * eliminating the need for a working IMU topic.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

#include <cmath>
#include <string>
#include <memory>
#include <chrono>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
class GeoUtils
{
public:
    static constexpr double EARTH_RADIUS_M = 6371000.0;

    static double haversineDistance(double lat1, double lon1,
                                    double lat2, double lon2)
    {
        const double dLat = toRad(lat2 - lat1);
        const double dLon = toRad(lon2 - lon1);
        const double a = std::sin(dLat/2)*std::sin(dLat/2)
                       + std::cos(toRad(lat1))*std::cos(toRad(lat2))
                       * std::sin(dLon/2)*std::sin(dLon/2);
        return EARTH_RADIUS_M * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0-a));
    }

    static double bearing(double lat1, double lon1,
                          double lat2, double lon2)
    {
        const double dLon = toRad(lon2 - lon1);
        const double y = std::sin(dLon)*std::cos(toRad(lat2));
        const double x = std::cos(toRad(lat1))*std::sin(toRad(lat2))
                       - std::sin(toRad(lat1))*std::cos(toRad(lat2))*std::cos(dLon);
        return wrapAngle(-(M_PI_2 - std::atan2(y, x)));
    }

    static double wrapAngle(double a)
    {
        while (a >  M_PI) a -= 2.0*M_PI;
        while (a < -M_PI) a += 2.0*M_PI;
        return a;
    }

    static double toRad(double d) { return d * M_PI / 180.0; }
    static double toDeg(double r) { return r * 180.0 / M_PI; }
};

// ─────────────────────────────────────────────────────────────────────────────
class PIDController
{
public:
    PIDController(double kp, double ki, double kd,
                  double out_min=-1e9, double out_max=1e9)
    : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max),
      integral_(0), prev_error_(0), first_(true) {}

    void reset() { integral_=0; prev_error_=0; first_=true; }

    double compute(double error, double dt)
    {
        if (dt <= 0.0) return 0.0;
        integral_ += error * dt;
        double deriv = first_ ? 0.0 : (error - prev_error_) / dt;
        first_ = false; prev_error_ = error;
        return std::clamp(kp_*error + ki_*integral_ + kd_*deriv, out_min_, out_max_);
    }

private:
    double kp_, ki_, kd_, out_min_, out_max_;
    double integral_, prev_error_;
    bool   first_;
};

// ─────────────────────────────────────────────────────────────────────────────
struct RoverState
{
    double latitude=0, longitude=0;
    bool   gps_valid=false;
    double yaw=0;
    bool   heading_valid=false;
    double prev_lat=0, prev_lon=0;
    bool   has_prev=false;
};

// ─────────────────────────────────────────────────────────────────────────────
class GlobalPlanner : public rclcpp::Node
{
public:
    enum class State { IDLE, NAVIGATING, REACHED };

    GlobalPlanner()
    : Node("global_planner"), state_(State::IDLE), last_time_(this->now())
    {
        declare_parameter("goal_latitude",    12.9726);
        declare_parameter("goal_longitude",   77.5956);
        declare_parameter("goal_tolerance_m",  1.0);
        declare_parameter("max_linear_vel",    2.0);
        declare_parameter("max_angular_vel",   2.0);
        declare_parameter("linear_kp",         0.8);
        declare_parameter("angular_kp",        1.5);
        declare_parameter("angular_ki",        0.0);
        declare_parameter("angular_kd",        0.05);
        declare_parameter("min_move_dist_m",   0.3);

        goal_lat_      = get_parameter("goal_latitude").as_double();
        goal_lon_      = get_parameter("goal_longitude").as_double();
        goal_tol_m_    = get_parameter("goal_tolerance_m").as_double();
        max_lin_vel_   = get_parameter("max_linear_vel").as_double();
        max_ang_vel_   = get_parameter("max_angular_vel").as_double();
        min_move_dist_ = get_parameter("min_move_dist_m").as_double();

        linear_pid_  = std::make_unique<PIDController>(
            get_parameter("linear_kp").as_double(), 0.0, 0.0, 0.0, max_lin_vel_);
        angular_pid_ = std::make_unique<PIDController>(
            get_parameter("angular_kp").as_double(),
            get_parameter("angular_ki").as_double(),
            get_parameter("angular_kd").as_double(),
            -max_ang_vel_, max_ang_vel_);

        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", 10,
            [this](sensor_msgs::msg::NavSatFix::SharedPtr msg){ gpsCallback(msg); });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            [this](sensor_msgs::msg::Imu::SharedPtr msg){ imuCallback(msg); });

        goal_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            "/goal_gps", 10,
            [this](sensor_msgs::msg::NavSatFix::SharedPtr msg){ goalCallback(msg); });

        cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        status_pub_  = create_publisher<std_msgs::msg::String>("/planner/status", 10);

        timer_ = create_wall_timer(100ms, [this](){ controlLoop(); });

        RCLCPP_INFO(get_logger(),
            "GlobalPlanner ready (GPS-only mode). Goal: (%.6f, %.6f) | Tol: %.1f m",
            goal_lat_, goal_lon_, goal_tol_m_);
        RCLCPP_INFO(get_logger(), "Waiting for /gps/fix ...");
    }

private:
    void gpsCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        const double new_lat = msg->latitude;
        const double new_lon = msg->longitude;

        if (rover_.has_prev) {
            double moved = GeoUtils::haversineDistance(
                rover_.prev_lat, rover_.prev_lon, new_lat, new_lon);
            if (moved >= min_move_dist_) {
                rover_.yaw = GeoUtils::bearing(
                    rover_.prev_lat, rover_.prev_lon, new_lat, new_lon);
                rover_.heading_valid = true;
                rover_.prev_lat = new_lat;
                rover_.prev_lon = new_lon;
            }
        } else {
            rover_.prev_lat = new_lat;
            rover_.prev_lon = new_lon;
            rover_.has_prev = true;
        }

        rover_.latitude  = new_lat;
        rover_.longitude = new_lon;
        rover_.gps_valid = true;
    }

    void imuCallback(sensor_msgs::msg::Imu::SharedPtr msg)
{
    double qx=msg->orientation.x, qy=msg->orientation.y;
    double qz=msg->orientation.z, qw=msg->orientation.w;
    
    // Change this line — subtract M_PI_2 to correct the 90° offset
    rover_.yaw = std::atan2(2.0*(qw*qz+qx*qy), 1.0-2.0*(qy*qy+qz*qz)) - M_PI_2;
    
    rover_.heading_valid = true;
    RCLCPP_INFO_ONCE(get_logger(), "IMU connected! Switching to IMU heading.");
}

    void goalCallback(sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        goal_lat_ = msg->latitude;
        goal_lon_ = msg->longitude;
        state_ = State::IDLE;
        linear_pid_->reset(); angular_pid_->reset();
        RCLCPP_INFO(get_logger(), "New goal: (%.6f, %.6f)", goal_lat_, goal_lon_);
    }

    void startNavigation()
    {
        state_ = State::NAVIGATING;
        last_time_ = this->now();
        linear_pid_->reset(); angular_pid_->reset();
        double dist = GeoUtils::haversineDistance(
            rover_.latitude, rover_.longitude, goal_lat_, goal_lon_);
        RCLCPP_INFO(get_logger(),
            ">>> Navigation STARTED! Distance to goal: %.2f m", dist);
    }

    void controlLoop()
    {
        auto now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt <= 0.0) return;

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
            "[STATE=%s] gps=%s heading=%s lat=%.6f lon=%.6f yaw=%.1fdeg",
            stateStr(),
            rover_.gps_valid ? "OK" : "WAITING",
            rover_.heading_valid ? "OK" : "NO",
            rover_.latitude, rover_.longitude,
            GeoUtils::toDeg(rover_.yaw));

        publishStatus();

        // Auto-start the moment GPS arrives
        if (state_ == State::IDLE && rover_.gps_valid) {
            startNavigation();
            return;
        }

        if (state_ != State::NAVIGATING) { publishStop(); return; }
        if (!rover_.gps_valid) { publishStop(); return; }

        double dist = GeoUtils::haversineDistance(
            rover_.latitude, rover_.longitude, goal_lat_, goal_lon_);
        double desired = GeoUtils::bearing(
            rover_.latitude, rover_.longitude, goal_lat_, goal_lon_);

        if (dist <= goal_tol_m_) {
            state_ = State::REACHED;
            publishStop();
            RCLCPP_INFO(get_logger(), "✅ GOAL REACHED! dist=%.3f m", dist);
            return;
        }

        double angular_vel, linear_vel;

        if (rover_.heading_valid) {
            double err = GeoUtils::wrapAngle(desired - rover_.yaw);
            angular_vel = angular_pid_->compute(err, dt);
            double factor = std::max(0.0, std::cos(err));
            linear_vel  = std::clamp(
                linear_pid_->compute(dist, dt) * factor, 0.0, max_lin_vel_);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[NAV] dist=%.2fm hdg_err=%.1fdeg lin=%.2f ang=%.2f",
                dist, GeoUtils::toDeg(err), linear_vel, angular_vel);
        } else {
            // No heading yet: spin in place toward goal bearing to generate movement
            double err = GeoUtils::wrapAngle(desired - rover_.yaw);
            angular_vel = std::copysign(
                std::min(0.4, max_ang_vel_), err);   // gentle spin
            linear_vel  = 0.15;                        // creep forward slowly

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[NAV-INIT] dist=%.2fm, building heading estimate...", dist);
        }

        geometry_msgs::msg::Twist twist;
        twist.linear.x  = linear_vel;
        twist.angular.z = angular_vel;
        cmd_vel_pub_->publish(twist);
    }

    void publishStop()
    {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
    }

    void publishStatus()
    {
        std_msgs::msg::String msg;
        msg.data = stateStr();
        status_pub_->publish(msg);
    }

    const char* stateStr() const {
        switch(state_) {
            case State::IDLE:       return "IDLE";
            case State::NAVIGATING: return "NAVIGATING";
            case State::REACHED:    return "REACHED";
        }
        return "UNKNOWN";
    }

    State      state_;
    RoverState rover_;
    double     goal_lat_, goal_lon_, goal_tol_m_;
    double     max_lin_vel_, max_ang_vel_, min_move_dist_;

    std::unique_ptr<PIDController> linear_pid_, angular_pid_;

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_, goal_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr      cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr          status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_time_;
};

// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlanner>());
    rclcpp::shutdown();
    return 0;
}
