#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include "marine_interfaces/msg/heartbeat.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"



class WAMVHelm: public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  WAMVHelm()
  :rclcpp_lifecycle::LifecycleNode("wamv_helm")
  {
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override
  {
    declare_parameter("max_thrust", max_thrust_);
    get_parameter("max_thrust", max_thrust_);
    declare_parameter("max_speed", max_speed_);
    get_parameter("max_speed", max_speed_);
    declare_parameter("max_yaw_speed", max_yaw_speed_);
    get_parameter("max_yaw_speed", max_yaw_speed_);

    left_thrust_publisher_ = create_publisher<std_msgs::msg::Float64>("thrusters/left/thrust",1);
    right_thrust_publisher_ = create_publisher<std_msgs::msg::Float64>("thrusters/right/thrust",1);
    left_position_publisher_ = create_publisher<std_msgs::msg::Float64>("thrusters/left/pos",1);
    right_position_publisher_ = create_publisher<std_msgs::msg::Float64>("thrusters/right/pos",1);

    status_publisher_ = create_publisher<marine_interfaces::msg::Heartbeat>("marine_autonomy/status/helm",1);

    twist_subscription_ = create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel", 10, std::bind(&WAMVHelm::twist_callback, this, std::placeholders::_1));
    joint_states_subscription_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", 5, std::bind(&WAMVHelm::joint_states_callback, this, std::placeholders::_1));

    standby_subscription_ = create_subscription<std_msgs::msg::Bool>("piloting_mode/standby/active", 5, std::bind(&WAMVHelm::standby_callback, this, std::placeholders::_1));

    return LifecycleNode::on_configure(state);
  }

private:

  void twist_callback(const geometry_msgs::msg::TwistStamped& msg)
  {
    double linear = msg.twist.linear.x/max_speed_;
    double angular = msg.twist.angular.z/max_yaw_speed_;

    linear = std::clamp(linear, -1.0, 1.0);
    angular = std::clamp(angular, -1.0, 1.0);

    double scale = std::max(std::abs(linear) + std::abs(angular), 1.0);

    double left_thrust = (linear - angular) / scale;
    double right_thrust = (linear + angular) / scale;

    std_msgs::msg::Float64 left_thrust_msg;
    left_thrust_msg.data = left_thrust * max_thrust_;
    left_thrust_publisher_->publish(left_thrust_msg);

    std_msgs::msg::Float64 right_thrust_msg;
    right_thrust_msg.data = right_thrust * max_thrust_;
    right_thrust_publisher_->publish(right_thrust_msg);

    std_msgs::msg::Float64 left_position_msg;
    left_position_msg.data = 0.0;
    left_position_publisher_->publish(left_position_msg);

    std_msgs::msg::Float64 right_position_msg;
    right_position_msg.data = 0.0;
    right_position_publisher_->publish(right_position_msg);
  }

  void joint_states_callback(const sensor_msgs::msg::JointState& msg)
  {
    if(last_status_time_.nanoseconds() == 0 || rclcpp::Time(msg.header.stamp) - last_status_time_ >= status_interval_)
    {
      marine_interfaces::msg::Heartbeat status_msg;
      status_msg.header = msg.header;
      for(std::size_t i = 0; i < msg.name.size(); ++i)
      {
        if(msg.name[i] == "wamv/left_chassis_engine_joint")
        {
          marine_interfaces::msg::KeyValue kv;
          kv.key = "left_angle";
          kv.value = std::to_string(msg.position[i]);
          status_msg.values.push_back(kv);
        }
        else if(msg.name[i] == "wamv/right_chassis_engine_joint")
        {
          marine_interfaces::msg::KeyValue kv;
          kv.key = "right_angle";
          kv.value = std::to_string(msg.position[i]);
          status_msg.values.push_back(kv);
        }
        else if(msg.name[i] == "wamv/left_engine_propeller_joint")
        {
          marine_interfaces::msg::KeyValue kv;
          kv.key = "left_speed";
          kv.value = std::to_string(msg.velocity[i]);
          status_msg.values.push_back(kv);
        }
        else if(msg.name[i] == "wamv/right_engine_propeller_joint")
        {
          marine_interfaces::msg::KeyValue kv;
          kv.key = "right_speed";
          kv.value = std::to_string(msg.velocity[i]);
          status_msg.values.push_back(kv);
        }
      }
      status_publisher_->publish(status_msg);
      last_status_time_ = msg.header.stamp;
    }
  }

  void standby_callback(const std_msgs::msg::Bool& msg)
  {
    if(msg.data)
    {
      // Publish zero thrusts when entering standby
      std_msgs::msg::Float64 left_thrust_msg;
      left_thrust_msg.data = 0.0;
      left_thrust_publisher_->publish(left_thrust_msg);

      std_msgs::msg::Float64 right_thrust_msg;
      right_thrust_msg.data = 0.0;
      right_thrust_publisher_->publish(right_thrust_msg);

      std_msgs::msg::Float64 left_position_msg;
      left_position_msg.data = 0.0;
      left_position_publisher_->publish(left_position_msg);

      std_msgs::msg::Float64 right_position_msg;
      right_position_msg.data = 0.0;
      right_position_publisher_->publish(right_position_msg);
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_thrust_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_position_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_position_publisher_;

  rclcpp::Publisher<marine_interfaces::msg::Heartbeat>::SharedPtr status_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr standby_subscription_;

  rclcpp::Time last_status_time_;
  rclcpp::Duration status_interval_ = rclcpp::Duration::from_seconds(0.5);

  double max_thrust_ = 1000.0;
  double max_speed_ = 3.3;
  double max_yaw_speed_ = 0.74;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WAMVHelm>();
  rclcpp::spin(node->get_node_base_interface());

  return 0;
}
