// Cartesian impedance controller for the FR3 (paper Eq. 7, simplified).
//
//   tau = J^T [ K e - D xdot ] + N (k_n (q_n - q) + tau_lim - d_n qdot) + tau_g
//
// e = (p_d - p, rotation vector of R_d R^T) is the 6-D pose error of the
// gripper centre, xdot = J qdot its twist, N = I - J^T (J J^T + l^2 I)^-1 J
// the null-space projector of the redundant seventh joint and tau_g an
// optional gravity term from KDL. tau_lim pushes a joint away from its limit
// once it comes within `joint_limit_margin`: it is the negative gradient of
// V_lim = 1/2 k_lim sum_i max(0, margin - dist_i)^2. The set-point (p_d, R_d) arrives on
// ~/target_pose; on activation it starts at the current pose, so switching
// to this controller does not move the arm.
//
// Stability (Lyapunov): with tau_g exact and a fixed set-point, take
//   V = 1/2 qdot^T M(q) qdot + 1/2 e^T K e  >= 0.
// Using the skew symmetry of Mdot - 2C, the robot dynamics give
//   Vdot = -xdot^T D xdot - qdot^T N d_n qdot + xdot^T F_env <= xdot^T F_env,
// so the arm alone is stable, and in contact with a passive mechanism
// (drawer, door: friction and damping only dissipate) the closed loop stays
// passive. See README section 5.

#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/string.hpp>

namespace sc_control
{

class CartesianImpedanceController : public controller_interface::ControllerInterface
{
public:
  using Vector6d = Eigen::Matrix<double, 6, 1>;
  using Matrix6d = Eigen::Matrix<double, 6, 6>;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool build_chain(const std::string & urdf);
  void read_state();
  void forward_kinematics(Eigen::Vector3d & p, Eigen::Quaterniond & r, Eigen::Matrix<double, 6, 7> & J);
  void load_gains();

  // parameters
  std::vector<std::string> joints_;
  std::string base_link_, tip_link_;
  Vector6d stiffness_, damping_;
  double nullspace_stiffness_{0.0}, nullspace_damping_{0.0}, pinv_damping_{0.0};
  double limit_margin_{0.0}, limit_stiffness_{0.0};
  Eigen::Matrix<double, 7, 1> q_min_, q_max_;
  double max_trans_error_{0.0}, max_rot_error_{0.0}, max_torque_rate_{0.0};
  std::array<double, 7> torque_limit_{};
  bool gravity_compensation_{false};

  // kinematics
  std::string urdf_;
  bool chain_ready_{false};
  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_;
  std::unique_ptr<KDL::ChainDynParam> dyn_;
  KDL::JntArray q_kdl_, g_kdl_;

  // state
  Eigen::Matrix<double, 7, 1> q_, dq_, q_null_, tau_prev_;
  struct Target
  {
    Eigen::Vector3d p;
    Eigen::Quaterniond r;
  };
  realtime_tools::RealtimeBuffer<std::shared_ptr<Target>> target_buffer_;
  Target target_;
  unsigned int cycle_{0};

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr description_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<std::string>> description_buffer_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>> pose_pub_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>> twist_pub_;
};

}  // namespace sc_control
