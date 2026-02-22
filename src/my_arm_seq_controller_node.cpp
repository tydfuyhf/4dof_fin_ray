// my_arm_seq_controller_node.cpp
// - holds initial pose until first target arrives
// - after receiving first /desired_joint_states, waits start_delay_sec
// - then moves ONE joint at a time in joint_order_ toward target

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <unordered_map>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

class MyArmSeqController : public rclcpp::Node {
public:
  MyArmSeqController() : Node("my_arm_seq_controller_node") {
    rate_hz_ = this->declare_parameter<double>("rate_hz", 50.0);
    joint_speed_rps_ = this->declare_parameter<double>("joint_speed_rps", 0.6);
    joint_tol_rad_ = this->declare_parameter<double>("joint_tol_rad", 0.01);
    start_delay_sec_ = this->declare_parameter<double>("start_delay_sec", 5.0);

    // Initial pose to hold before starting (URDF "default" visually is usually all zeros)
    initial_q_ = this->declare_parameter<std::vector<double>>(
      "initial_q",
      std::vector<double>{0.0, 0.0, 0.0, 0.0}
    );

    joint_order_ = this->declare_parameter<std::vector<std::string>>(
      "joint_order",
      std::vector<std::string>{
        "joint_to_horn_case",
        "joint_to_arm_link_1",
        "joint_to_arm_link_2",
        "joint_to_gripper_link2"
      }
    );

    if (initial_q_.size() != joint_order_.size()) {
      RCLCPP_WARN(get_logger(),
                  "initial_q size(%zu) != joint_order size(%zu). Using zeros.",
                  initial_q_.size(), joint_order_.size());
      initial_q_.assign(joint_order_.size(), 0.0);
    }

    pub_js_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    sub_desired_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/desired_joint_states", rclcpp::QoS(10),
      std::bind(&MyArmSeqController::onDesired, this, std::placeholders::_1)
    );

    // State init
    current_q_ = initial_q_;                // hold this until start
    target_q_  = initial_q_;
    have_target_ = false;
    started_ = false;
    active_index_ = 0;

    auto period = std::chrono::duration<double>(1.0 / std::max(1.0, rate_hz_));
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MyArmSeqController::onTick, this)
    );

    RCLCPP_INFO(get_logger(),
      "Seq controller ready. Hold initial pose. Start %.2fs after first target.",
      start_delay_sec_);
  }

private:
  void onDesired(const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::unordered_map<std::string, double> m;
    m.reserve(msg->name.size());
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i < msg->position.size()) m[msg->name[i]] = msg->position[i];
    }

    std::vector<double> new_target(joint_order_.size(), 0.0);
    for (size_t i = 0; i < joint_order_.size(); ++i) {
      auto it = m.find(joint_order_[i]);
      if (it == m.end()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Desired missing joint %s", joint_order_[i].c_str());
        return;
      }
      new_target[i] = it->second;
    }

    target_q_ = new_target;

    if (!have_target_) {
      have_target_ = true;
      first_target_time_ = this->now();     // arm delay timer
      started_ = false;
      active_index_ = 0;
      // NOTE: current_q_ stays at initial_q_ so you will SEE motion after delay.
      RCLCPP_INFO(get_logger(), "First target received. Will start in %.2fs", start_delay_sec_);
    }
  }

  static double stepToward(double cur, double tgt, double max_step) {
    const double e = tgt - cur;
    if (std::fabs(e) <= max_step) return tgt;
    return cur + (e > 0.0 ? max_step : -max_step);
  }

  void publishCurrent_() {
    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.name = joint_order_;
    js.position = current_q_;
    pub_js_->publish(js);
  }

  void onTick() {
    // Always publish something so robot_state_publisher updates smoothly
    if (!have_target_) {
      publishCurrent_(); // hold initial pose forever until target arrives
      return;
    }

    // Wait delay after first target
    const double elapsed = (this->now() - first_target_time_).seconds();
    if (!started_) {
      if (elapsed < start_delay_sec_) {
        publishCurrent_(); // still holding initial pose
        return;
      }
      started_ = true;
      RCLCPP_INFO(get_logger(), "Starting sequential motion now.");
    }

    const double dt = 1.0 / std::max(1.0, rate_hz_);
    const double max_step = joint_speed_rps_ * dt;

    // Move only active joint
    const size_t i = active_index_;
    const double err = target_q_[i] - current_q_[i];

    if (std::fabs(err) <= joint_tol_rad_) {
      active_index_ = (active_index_ + 1) % joint_order_.size();
    } else {
      current_q_[i] = stepToward(current_q_[i], target_q_[i], max_step);
    }

    publishCurrent_();
  }

private:
  // Params
  double rate_hz_{50.0};
  double joint_speed_rps_{0.6};
  double joint_tol_rad_{0.01};
  double start_delay_sec_{5.0};

  std::vector<std::string> joint_order_;
  std::vector<double> initial_q_;

  // State
  std::vector<double> current_q_;
  std::vector<double> target_q_;
  bool have_target_{false};
  bool started_{false};
  size_t active_index_{0};
  rclcpp::Time first_target_time_{0, 0, RCL_ROS_TIME};

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_js_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_desired_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyArmSeqController>());
  rclcpp::shutdown();
  return 0;
}