// Roland Arsenault
// Center for Coastal and Ocean Mapping
// University of New Hampshire
// Copyright 2017, All rights reserved.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "marine_interfaces/msg/helm.hpp"
#include "marine_interfaces/msg/heartbeat.hpp"
#include "marine_autonomy/pid.h"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"


class ASVHelm: public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ASVHelm()
  :rclcpp_lifecycle::LifecycleNode("asv_helm")
  {
  }
  
  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override
  {
    throttle_publisher_ = create_publisher<std_msgs::msg::Float32>("throttle",1);
    rudder_publisher_ = create_publisher<std_msgs::msg::Float32>("rudder",1);
    status_publisher_ = create_publisher<marine_interfaces::msg::Heartbeat>("marine/status/helm",1);

    helm_subscription_ = create_subscription<marine_interfaces::msg::Helm>("helm", 1, std::bind(&ASVHelm::helmCallback, this, std::placeholders::_1));
    twist_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel", 10, std::bind(&ASVHelm::twistCallback, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>("odom", 5, std::bind(&ASVHelm::odometryCallback, this, std::placeholders::_1));

  
    have_commands_subscription_ = create_subscription<std_msgs::msg::Bool>("have_commands", 1, std::bind(&ASVHelm::haveCommandsCallback, this, std::placeholders::_1));
    return LifecycleNode::on_configure(state);
  }

private:
  void helmCallback(const marine_interfaces::msg::Helm& msg)
  {
    std_msgs::msg::Float32 throttle_msg;
    throttle_msg.data = msg.throttle;
    throttle_publisher_->publish(throttle_msg);
    std_msgs::msg::Float32 rudder_msg;
    rudder_msg.data = msg.rudder;
    rudder_publisher_->publish(rudder_msg);
  }

  void twistCallback(const geometry_msgs::msg::TwistStamped& msg)
  {
    std_msgs::msg::Float32 throttle_msg;
    throttle_msg.data = msg.twist.linear.x/max_speed_;
    if(rclcpp::Time(msg.header.stamp) - rclcpp::Time(latest_odometry_.header.stamp) < rclcpp::Duration::from_seconds(1.0))
    {
      if(!pid_)
        pid_ = std::make_shared<marine::PID>(shared_from_this());

      pid_->setPoint(msg.twist.linear.x);
      throttle_msg.data = pid_->update(latest_odometry_.twist.twist.linear.x, latest_odometry_.header.stamp);
    }
    else
    {
      auto clock = get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(get_logger(), *clock, 2000, "No recent odometry for use with throttle PID");
    }
    throttle_msg.data = std::max(-1.0f, std::min(1.0f, throttle_msg.data));
    std_msgs::msg::Float32 rudder_msg;
    rudder_msg.data = -msg.twist.angular.z/max_yaw_speed_;
    rudder_msg.data = std::max(-1.0f, std::min(1.0f, rudder_msg.data));
    
    float min_throttle = 0.5*std::abs(rudder_msg.data);
    if(msg.twist.linear.x > 0.0)
      throttle_msg.data = std::max(throttle_msg.data, min_throttle);
    else
      throttle_msg.data = std::min(throttle_msg.data,-min_throttle);

    throttle_publisher_->publish(throttle_msg);
    rudder_publisher_->publish(rudder_msg);
  }

  void odometryCallback(const nav_msgs::msg::Odometry &msg)
  {
    latest_odometry_ = msg;
  }

  void haveCommandsCallback(const std_msgs::msg::Bool& msg)
  {
    marine_interfaces::msg::Heartbeat hb;
    hb.header.stamp = get_clock()->now();
    marine_interfaces::msg::KeyValue kv;
    kv.key = "sim";
    kv.value = "asv_sim";
    hb.values.push_back(kv);
    kv.key = "have_commands";
    if(msg.data)
      kv.value = "true";
    else
      kv.value = "false";
    hb.values.push_back(kv);
    status_publisher_->publish(hb);
  }


  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rudder_publisher_;
  rclcpp::Publisher<marine_interfaces::msg::Heartbeat>::SharedPtr status_publisher_;

  rclcpp::Subscription<marine_interfaces::msg::Helm>::SharedPtr helm_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr have_commands_subscription_;


  nav_msgs::msg::Odometry latest_odometry_;
  std::shared_ptr<marine::PID> pid_;
  double max_speed_ = 2.75;
  double max_yaw_speed_ = 0.5;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ASVHelm>();
  rclcpp::spin(node->get_node_base_interface());

  return 0;
}
