#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "first_project/msg/tf_error_msg.hpp"

using namespace std::chrono_literals;

class TfError : public rclcpp::Node
{
public:
  TfError() : Node("tf_error")
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);
    timer_ = this->create_wall_timer(500ms, std::bind(&TfError::on_timer, this));

    publisher_ = this->create_publisher<first_project::msg::TfErrorMsg>("/tf_error_msg", 10);

    RCLCPP_INFO(this->get_logger(), "tf_error started.");
  }


private:
  std::shared_ptr<tf2_ros::Buffer>            tf_buffer_;   // stores transforms
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_; // feeds the buffer
  rclcpp::TimerBase::SharedPtr                timer_;       // triggers onTimer()
  rclcpp::Publisher<first_project::msg::TfErrorMsg>::SharedPtr publisher_;
  
  float x_prev    = 0.0;
  float y_prev    = 0.0;
  float dist_prev = 0.0;

  void on_timer()
  {
    geometry_msgs::msg::TransformStamped base_link;
    geometry_msgs::msg::TransformStamped base_link2;

    try {
      base_link  = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
      base_link2 = tf_buffer_->lookupTransform("odom", "base_link2", tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Lookup failed: %s", ex.what());
      return;
    }

    auto   ts    = base_link.header.stamp; 
    float x_bl  = base_link.transform.translation.x;
    float y_bl  = base_link.transform.translation.y;
    float x_bl2 = base_link2.transform.translation.x;
    float y_bl2 = base_link2.transform.translation.y;

    float distance = std::sqrt(std::pow(x_bl - x_bl2, 2) + std::pow(y_bl - y_bl2, 2));
    float trav_dist = dist_prev + std::sqrt(std::pow(x_bl2 - x_prev, 2) + std::pow(y_bl2 - y_prev, 2));
    
    x_prev = x_bl2;
    y_prev = y_bl2;
    dist_prev = trav_dist;

    first_project::msg::TfErrorMsg tf_err;
    tf_err.tf_error = distance;
    tf_err.time_from_start = ts.sec;
    tf_err.travelled_distance = dist_prev;
    
    publisher_->publish(tf_err);
  }
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfError>());
  rclcpp::shutdown();
  return 0;
}