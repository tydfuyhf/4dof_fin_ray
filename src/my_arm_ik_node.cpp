#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>

struct Segment {
  Eigen::Vector3d origin_xyz;
  Eigen::Vector3d origin_rpy;
  bool is_revolute;
  Eigen::Vector3d axis_local;
  std::string joint_name;
  double lower, upper;
};

static Eigen::Matrix3d rotX(double a){
  double c=cos(a),s=sin(a);
  Eigen::Matrix3d R; R<<1,0,0,0,c,-s,0,s,c; return R;
}
static Eigen::Matrix3d rotY(double a){
  double c=cos(a),s=sin(a);
  Eigen::Matrix3d R; R<<c,0,s,0,1,0,-s,0,c; return R;
}
static Eigen::Matrix3d rotZ(double a){
  double c=cos(a),s=sin(a);
  Eigen::Matrix3d R; R<<c,-s,0,s,c,0,0,0,1; return R;
}
static Eigen::Matrix3d rpyToRot(const Eigen::Vector3d& rpy){
  return rotZ(rpy.z()) * rotY(rpy.y()) * rotX(rpy.x());
}
static Eigen::Matrix3d rotAxisAngle(const Eigen::Vector3d& axis_unit,double angle){
  return Eigen::AngleAxisd(angle,axis_unit).toRotationMatrix();
}
static Eigen::Matrix4d makeT(const Eigen::Matrix3d& R,const Eigen::Vector3d& p){
  Eigen::Matrix4d T=Eigen::Matrix4d::Identity();
  T.block<3,3>(0,0)=R; T.block<3,1>(0,3)=p; return T;
}

class MyArmIkNode : public rclcpp::Node {
public:
  MyArmIkNode():Node("my_arm_ik_node"){

    rate_hz_ = declare_parameter<double>("rate_hz",30.0);
    ik_iters_per_tick_ = declare_parameter<int>("ik_iters_per_tick",20);
    lambda_ = declare_parameter<double>("dls_lambda",0.02);
    step_gain_ = declare_parameter<double>("step_gain",0.7);
    tol_m_ = declare_parameter<double>("pos_tolerance_m",0.001);

    buildChain();

    pub_desired_ = create_publisher<sensor_msgs::msg::JointState>(
      "/desired_joint_states",10);

    sub_target_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/tip_target",10,
      std::bind(&MyArmIkNode::onTarget,this,std::placeholders::_1));

    auto period = std::chrono::duration<double>(1.0/std::max(1.0,rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MyArmIkNode::onTick,this));

    q_seed_.assign(joint_names_.size(),0.0);

    RCLCPP_INFO(get_logger(),"IK node ready. tip = gripper_base origin. Waiting for /tip_target...");
  }

private:

  void buildChain(){

    segments_.clear();
    joint_names_.clear();
    lower_.clear();
    upper_.clear();

    // base_link -> joint_without_idler (fixed)
    segments_.push_back({{0.028,-0.0005,0},{0,0,0},false,{0,0,0},"",0,0});

    // joint_without_idler -> horn_case (continuous)
    segments_.push_back({{0.0198,0.0265,0.02745},{0,0,0},true,{0,0,1},
      "joint_to_horn_case",-1e9,1e9});

    // horn_case -> joint_idler (fixed)
    segments_.push_back({{-0.020276,-0.0275,-0.02735},{0,0,0},false,{0,0,0},"",0,0});

    // joint_idler -> robot_arm_link_1 (revolute)
    segments_.push_back({{0.020276,0.03845,0.062239},{0,1.5708,1.5708},true,{0,0,1},
      "joint_to_arm_link_1",-1.0472,1.0472});

    // robot_arm_link_1 -> joint_idler_2 (fixed)
    segments_.push_back({{-0.0691,-0.106735,-0.039},{-1.5708,3.1416,0},false,{0,0,0},"",0,0});

    // joint_idler_2 -> robot_arm_link_2 (revolute)
    segments_.push_back({{-0.001696,0.017235,0.129},{1.5708,1.5708,0},true,{0,0,1},
      "joint_to_arm_link_2",-1.0472,1.0472});

    // robot_arm_link_2 -> joint_idler_3 (fixed)
    segments_.push_back({{0.062,0.0687,0.018},{-1.5708,0,1.5708},false,{0,0,0},"",0,0});

    // joint_idler_3 -> gripper_link2 (revolute)
    segments_.push_back({{-0.0694,0.019,0.1296},{1.5708,3.1416,0},true,{0,0,1},
      "joint_to_gripper_link2",-1.0472,1.0472});

    // gripper_link2 -> joint_without_idler2 (fixed)  [NEW]
    segments_.push_back({{-0.017,-0.0352,-0.0115},{1.5708,-1.5708,0},false,{0,0,0},"",0,0});

    // joint_without_idler2 -> gripper_base (fixed)   [NEW]
    segments_.push_back({{0.002,-0.0119,0.007},{-3.1416,0,0},false,{0,0,0},"",0,0});

    // collect actuated joints
    for(const auto& seg:segments_){
      if(seg.is_revolute){
        joint_names_.push_back(seg.joint_name);
        lower_.push_back(seg.lower);
        upper_.push_back(seg.upper);
      }
    }
  }

  void onTarget(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
    desired_target_.x()=msg->pose.position.x;
    desired_target_.y()=msg->pose.position.y;
    desired_target_.z()=msg->pose.position.z;
    have_target_=true;
  }

  void computeFkJac(const std::vector<double>& q,
                    Eigen::Vector3d& p_tip,
                    Eigen::MatrixXd& J){

    int N=q.size();
    J=Eigen::MatrixXd::Zero(3,N);
    Eigen::Matrix4d T=Eigen::Matrix4d::Identity();

    std::vector<Eigen::Vector3d> p_joint,z_axis;
    p_joint.reserve(N);
    z_axis.reserve(N);

    int qi=0;
    for(const auto& seg:segments_){

      // apply fixed transform (origin xyz/rpy)
      T=T*makeT(rpyToRot(seg.origin_rpy),seg.origin_xyz);

      if(seg.is_revolute){
        // joint axis and origin in base frame BEFORE joint rotation
        Eigen::Matrix3d R=T.block<3,3>(0,0);
        Eigen::Vector3d axis=seg.axis_local.normalized();
        p_joint.push_back(T.block<3,1>(0,3));
        z_axis.push_back(R*axis);

        // apply joint rotation
        T=T*makeT(rotAxisAngle(axis,q[qi]),Eigen::Vector3d::Zero());
        qi++;
      }
    }

    // Tip is end of chain: now gripper_base origin
    p_tip=T.block<3,1>(0,3);

    // Jacobian for position-only IK
    for(int i=0;i<N;i++)
      J.col(i)=z_axis[i].cross(p_tip-p_joint[i]);
  }

  void onTick(){

    if(!have_target_) return;

    std::vector<double> q=q_seed_;

    for(int iter=0;iter<ik_iters_per_tick_;iter++){

      Eigen::Vector3d p_tip;
      Eigen::MatrixXd J;
      computeFkJac(q,p_tip,J);

      Eigen::Vector3d e=desired_target_-p_tip;
      if(e.norm()<tol_m_) break;

      Eigen::Matrix3d A=J*J.transpose();
      A+=lambda_*lambda_*Eigen::Matrix3d::Identity();
      Eigen::Vector3d y=A.ldlt().solve(e);
      Eigen::VectorXd dq=J.transpose()*y;

      dq*=step_gain_;

      for(int i=0;i<dq.size();i++){
        q[i]+=dq[i];
        if(std::isfinite(lower_[i])&&std::isfinite(upper_[i]))
          q[i]=std::clamp(q[i],lower_[i],upper_[i]);
      }
    }

    q_seed_=q;

    sensor_msgs::msg::JointState out;
    out.header.stamp=now();
    out.name=joint_names_;
    out.position=q;
    pub_desired_->publish(out);
  }

private:
  double rate_hz_{30.0},lambda_{0.02},step_gain_{0.7},tol_m_{0.001};
  int ik_iters_per_tick_{20};

  std::vector<Segment> segments_;
  std::vector<std::string> joint_names_;
  std::vector<double> lower_,upper_;
  std::vector<double> q_seed_;

  Eigen::Vector3d desired_target_{0,0,0};
  bool have_target_{false};

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_desired_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_target_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc,char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<MyArmIkNode>());
  rclcpp::shutdown();
  return 0;
}
