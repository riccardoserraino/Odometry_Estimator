#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "bunker_msgs/msg/bunker_status.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "first_project/srv/reset.hpp"


using std::placeholders::_1;
using std::placeholders::_2;



class Odometer : public rclcpp::Node
{
public:
  Odometer() : Node("odometer"),
               x_(0.0), y_(0.0), theta_(0.0),
               initialized_(false)
  {
    subscription_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "/bunker_status", 10, 
      std::bind(&Odometer::odometry_callback, this, _1));

    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/project_odom", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    service_ = this->create_service<first_project::srv::Reset>(
      "reset_odom",
      std::bind(&Odometer::handle_request, this, _1, _2));

    last_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Odometer started.");
  }


private:
  // ── State
  double dt_;
  double x_, y_, theta_;
  double vx_, omega_;
  bool initialized_;
  rclcpp::Time last_time_;

  // Wheels
  double vl_, vr_;
  double rpm_front_left_, rpm_front_right_;
  double omega_front_left_, omega_front_right_;

  // Ground truth
  double vx_real_, omega_real_;

  // ── Configuration
  const double omega_threshold_ = 1e-4;
  const double wheel_base_ = 0.710; //0.635
  const double wheel_radius_ = 0.083; //0.127
  const double gr_ = 7.5;



  // ── ROS interfaces
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<first_project::srv::Reset>::SharedPtr service_;


  // HELPER FUNCTIONS
  void integrateExact(double v, double omega, double dt)
  {
    double theta_next = theta_ + omega * dt;

    x_ += (v / omega) * (sin(theta_next) - sin(theta_));
    y_ -= (v / omega) * (cos(theta_next) - cos(theta_));
    theta_ = theta_next;
  }

  void integrateRK2(double v, double omega, double dt)
  {
    double theta_mid = theta_ + 0.5 * omega * dt;

    x_ += v * cos(theta_mid) * dt;
    y_ += v * sin(theta_mid) * dt;
    theta_ += omega * dt;
  }

  // Adaptive function to choose integration method based on angular velocity
  void integrate(double v, double omega, double dt)
  {
    if (fabs(omega) > omega_threshold_)
      integrateExact(v, omega, dt);
    else
      integrateRK2(v, omega, dt);
  }


  // RESET SERVICE
  void handle_request(const std::shared_ptr<first_project::srv::Reset::Request> request,
                      std::shared_ptr<first_project::srv::Reset::Response> response)
  {
    if (request->reset == true) {
      RCLCPP_INFO(this->get_logger(), "Resetting odometry...");
      x_ = 0.0;
      y_ = 0.0;
      theta_ = 0.0;
      initialized_ = false;
    }
    response->success = true;
  }


  // ODOMETRY CALLBACK
  void odometry_callback(const bunker_msgs::msg::BunkerStatus & msg)
  {
    rclcpp::Time current_time = this->get_clock()->now();

    if (!initialized_) {
      last_time_ = current_time;
      initialized_ = true;
      return;
    }

    dt_ = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // Reject invalid dt
    if (dt_ <= 0.0 || dt_ > 1.0)
      return;

    // ── Read RPM
    rpm_front_right_ = msg.actuator_states[0].rpm;
    rpm_front_left_  = msg.actuator_states[1].rpm;

    // ── Motor - wheel angular velocity
    omega_front_left_  = (rpm_front_left_  * 2.0 * M_PI / 60.0) / gr_;
    omega_front_right_ = (rpm_front_right_ * 2.0 * M_PI / 60.0) / gr_;

    // ── Linear velocity of wheels
    vl_ = wheel_radius_ * omega_front_left_;
    vr_ = wheel_radius_ * omega_front_right_;

    // ── Robot velocity (estimation)
    vx_ = (vr_ + vl_) / 2.0;
    omega_ = (vr_ - vl_) / wheel_base_;

    // ── Ground truth
    vx_real_ = msg.linear_velocity;
    omega_real_ = msg.angular_velocity;

    // ── Integration
    integrate(vx_, omega_, dt_);

    // ── Orientation
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);

    // Publish odometry message
    nav_msgs::msg::Odometry odom;

    // ── Header (required by RViz2)
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link2";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = vx_;
    odom.twist.twist.angular.z = omega_;

    publisher_->publish(odom);

    // ── CLEAN LOGS

    // 1. Main state 
    RCLCPP_INFO(this->get_logger(),
      "POSE x=%.3f y=%.3f θ=%.3f | v=%.3f ω=%.3f | dt=%.3f",
      x_, y_, theta_, vx_, omega_, dt_);

    // 2. Estimation vs Ground Truth 
    RCLCPP_INFO(this->get_logger(),
      "VEL COMP | est: v=%.3f ω=%.3f | gt: v=%.3f ω=%.3f | err_v=%.3f err_w=%.3f",
      vx_, omega_,
      vx_real_, omega_real_,
      vx_ - vx_real_,
      omega_ - omega_real_);

    
    // Publish tf transform: odom -> base_link2
    geometry_msgs::msg::TransformStamped transform;

    // Set the header with timestamp and frame names
    transform.header.stamp = current_time;
    transform.header.frame_id = "odom";      // Parent frame
    transform.child_frame_id = "base_link2"; // Child frame

    // Set translation (x, y position)
    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;

    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();
    
    // Broadcast the transform
    tf_broadcaster_->sendTransform(transform);

  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Odometer>());
  rclcpp::shutdown();
  return 0;
}