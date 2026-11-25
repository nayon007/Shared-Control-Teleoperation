#include "sc_control/cartesian_impedance_controller.hpp"

#include <algorithm>
#include <cmath>

#include <kdl_parser/kdl_parser.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <urdf/model.h>
#include <rclcpp/qos.hpp>

namespace sc_control
{

using controller_interface::CallbackReturn;
using controller_interface::InterfaceConfiguration;
using controller_interface::interface_configuration_type;

namespace
{
Eigen::Matrix<double, 6, 1> to6(const std::vector<double> & v)
{
  Eigen::Matrix<double, 6, 1> out;
  for (int i = 0; i < 6; ++i) {out(i) = v.at(i);}
  return out;
}
}  // namespace

CallbackReturn CartesianImpedanceController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("base_link", "fr3_link0");
    auto_declare<std::string>("tip_link", "fr3_hand_tcp");
    auto_declare<std::string>("robot_description", "");
    // [N/m, N/m, N/m, Nm/rad, Nm/rad, Nm/rad], base frame
    auto_declare<std::vector<double>>("stiffness", {600, 600, 600, 30, 30, 30});
    // [Ns/m ..., Nms/rad ...]
    auto_declare<std::vector<double>>("damping", {50, 50, 50, 2, 2, 2});
    auto_declare<double>("nullspace_stiffness", 2.0);
    auto_declare<double>("nullspace_damping", 1.0);
    auto_declare<double>("pinv_damping", 0.05);
    auto_declare<double>("joint_limit_margin", 0.3);      // [rad]
    auto_declare<double>("joint_limit_stiffness", 30.0);  // [Nm/rad]
    auto_declare<double>("max_trans_error", 0.10);
    auto_declare<double>("max_rot_error", 0.3);
    auto_declare<double>("max_torque_rate", 1.0);   // [Nm per cycle], FR3 accepts 1 Nm/ms
    auto_declare<std::vector<double>>("torque_limit", {87, 87, 87, 87, 12, 12, 12});
    auto_declare<bool>("gravity_compensation", false);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter declaration failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration CartesianImpedanceController::command_interface_configuration() const
{
  InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {cfg.names.push_back(j + "/effort");}
  return cfg;
}

InterfaceConfiguration CartesianImpedanceController::state_interface_configuration() const
{
  InterfaceConfiguration cfg{interface_configuration_type::INDIVIDUAL, {}};
  for (const auto & j : joints_) {cfg.names.push_back(j + "/position");}
  for (const auto & j : joints_) {cfg.names.push_back(j + "/velocity");}
  return cfg;
}

void CartesianImpedanceController::load_gains()
{
  auto node = get_node();
  stiffness_ = to6(node->get_parameter("stiffness").as_double_array());
  damping_ = to6(node->get_parameter("damping").as_double_array());
  nullspace_stiffness_ = node->get_parameter("nullspace_stiffness").as_double();
  nullspace_damping_ = node->get_parameter("nullspace_damping").as_double();
  pinv_damping_ = node->get_parameter("pinv_damping").as_double();
  limit_margin_ = node->get_parameter("joint_limit_margin").as_double();
  limit_stiffness_ = node->get_parameter("joint_limit_stiffness").as_double();
  max_trans_error_ = node->get_parameter("max_trans_error").as_double();
  max_rot_error_ = node->get_parameter("max_rot_error").as_double();
  max_torque_rate_ = node->get_parameter("max_torque_rate").as_double();
  auto lim = node->get_parameter("torque_limit").as_double_array();
  for (size_t i = 0; i < 7; ++i) {torque_limit_[i] = lim.at(i);}
  gravity_compensation_ = node->get_parameter("gravity_compensation").as_bool();
}

CallbackReturn CartesianImpedanceController::on_configure(const rclcpp_lifecycle::State &)
{
  auto node = get_node();
  joints_ = node->get_parameter("joints").as_string_array();
  if (joints_.size() != 7) {
    RCLCPP_ERROR(node->get_logger(), "'joints' must list the 7 arm joints");
    return CallbackReturn::ERROR;
  }
  base_link_ = node->get_parameter("base_link").as_string();
  tip_link_ = node->get_parameter("tip_link").as_string();
  try {
    load_gains();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "bad gain parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }

  // The URDF comes from the parameter if set, otherwise from the latched
  // /robot_description topic (Humble does not hand it to controllers).
  urdf_ = node->get_parameter("robot_description").as_string();
  chain_ready_ = !urdf_.empty() && build_chain(urdf_);
  if (!chain_ready_) {
    description_sub_ = node->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::QoS(1).transient_local(),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        description_buffer_.writeFromNonRT(std::make_shared<std::string>(msg->data));
      });
  }

  target_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/target_pose", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      auto t = std::make_shared<Target>();
      t->p = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
      t->r = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                                msg->pose.orientation.y, msg->pose.orientation.z).normalized();
      target_buffer_.writeFromNonRT(t);
    });

  pose_pub_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>>(
    node->create_publisher<geometry_msgs::msg::PoseStamped>("~/current_pose", 10));
  twist_pub_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::TwistStamped>>(
    node->create_publisher<geometry_msgs::msg::TwistStamped>("~/current_twist", 10));
  RCLCPP_INFO(node->get_logger(), "configured: K = [%g %g %g | %g %g %g], gravity term %s",
    stiffness_(0), stiffness_(1), stiffness_(2), stiffness_(3), stiffness_(4), stiffness_(5),
    gravity_compensation_ ? "on" : "off");
  return CallbackReturn::SUCCESS;
}

bool CartesianImpedanceController::build_chain(const std::string & urdf)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromString(urdf, tree) || !tree.getChain(base_link_, tip_link_, chain_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "no KDL chain %s -> %s in the URDF",
      base_link_.c_str(), tip_link_.c_str());
    return false;
  }
  if (chain_.getNrOfJoints() != 7) {
    RCLCPP_ERROR(get_node()->get_logger(), "chain has %u joints, expected 7", chain_.getNrOfJoints());
    return false;
  }
  urdf::Model model;
  if (!model.initString(urdf)) {
    RCLCPP_ERROR(get_node()->get_logger(), "could not parse the URDF");
    return false;
  }
  for (size_t i = 0; i < 7; ++i) {
    auto j = model.getJoint(joints_[i]);
    if (!j || !j->limits) {
      RCLCPP_ERROR(get_node()->get_logger(), "no limits for %s", joints_[i].c_str());
      return false;
    }
    q_min_(i) = j->limits->lower;
    q_max_(i) = j->limits->upper;
  }
  fk_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
  jac_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
  dyn_ = std::make_unique<KDL::ChainDynParam>(chain_, KDL::Vector(0.0, 0.0, -9.81));
  q_kdl_.resize(7);
  g_kdl_.resize(7);
  return true;
}

void CartesianImpedanceController::read_state()
{
  for (size_t i = 0; i < 7; ++i) {
    q_(i) = state_interfaces_[i].get_value();
    dq_(i) = state_interfaces_[7 + i].get_value();
  }
}

void CartesianImpedanceController::forward_kinematics(
  Eigen::Vector3d & p, Eigen::Quaterniond & r, Eigen::Matrix<double, 6, 7> & J)
{
  for (size_t i = 0; i < 7; ++i) {q_kdl_(i) = q_(i);}
  KDL::Frame f;
  fk_->JntToCart(q_kdl_, f);
  p = Eigen::Vector3d(f.p.x(), f.p.y(), f.p.z());
  double x, y, z, w;
  f.M.GetQuaternion(x, y, z, w);
  r = Eigen::Quaterniond(w, x, y, z);
  KDL::Jacobian jac(7);
  jac_->JntToJac(q_kdl_, jac);
  J = jac.data;
}

CallbackReturn CartesianImpedanceController::on_activate(const rclcpp_lifecycle::State &)
{
  if (!chain_ready_) {
    auto d = description_buffer_.readFromNonRT();
    chain_ready_ = d && *d && build_chain(**d);
    if (!chain_ready_) {
      RCLCPP_ERROR(get_node()->get_logger(), "robot_description not received yet");
      return CallbackReturn::ERROR;
    }
  }
  load_gains();        // allows `ros2 param set` between deactivate/activate
  read_state();
  Eigen::Matrix<double, 6, 7> J;
  forward_kinematics(target_.p, target_.r, J);
  target_buffer_.writeFromNonRT(std::make_shared<Target>(target_));
  q_null_ = q_;
  tau_prev_.setZero();
  for (size_t i = 0; i < 7; ++i) {tau_prev_(i) = command_interfaces_[i].get_value();}
  for (size_t i = 0; i < 7; ++i) {
    if (!std::isfinite(tau_prev_(i))) {tau_prev_(i) = 0.0;}
  }
  RCLCPP_INFO(get_node()->get_logger(), "active, holding p = (%.3f, %.3f, %.3f)",
    target_.p.x(), target_.p.y(), target_.p.z());
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianImpedanceController::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & c : command_interfaces_) {c.set_value(0.0);}
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type CartesianImpedanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  auto t = target_buffer_.readFromRT();
  if (t && *t) {target_ = **t;}

  read_state();
  Eigen::Vector3d p;
  Eigen::Quaterniond r;
  Eigen::Matrix<double, 6, 7> J;
  forward_kinematics(p, r, J);

  // 6-D pose error, both parts saturated so the spring force stays bounded
  Vector6d e;
  Eigen::Vector3d ep = target_.p - p;
  if (ep.norm() > max_trans_error_) {ep *= max_trans_error_ / ep.norm();}
  Eigen::Quaterniond qe = target_.r * r.inverse();
  if (qe.w() < 0.0) {qe.coeffs() *= -1.0;}
  Eigen::AngleAxisd aa(qe);
  Eigen::Vector3d eo = aa.angle() * aa.axis();
  if (eo.norm() > max_rot_error_) {eo *= max_rot_error_ / eo.norm();}
  e << ep, eo;

  const Vector6d xdot = J * dq_;
  const Vector6d F = stiffness_.cwiseProduct(e) - damping_.cwiseProduct(xdot);
  Eigen::Matrix<double, 7, 1> tau = J.transpose() * F;

  // redundancy: keep the elbow near its start configuration, without
  // producing force at the gripper
  const Eigen::Matrix<double, 7, 7> N = Eigen::Matrix<double, 7, 7>::Identity() -
    J.transpose() * (J * J.transpose() + pinv_damping_ * pinv_damping_ * Matrix6d::Identity())
    .ldlt().solve(J);
  Eigen::Matrix<double, 7, 1> tau_lim = Eigen::Matrix<double, 7, 1>::Zero();
  for (int i = 0; i < 7; ++i) {
    const double lo = q_(i) - q_min_(i), hi = q_max_(i) - q_(i);
    if (lo < limit_margin_) {tau_lim(i) += limit_stiffness_ * (limit_margin_ - lo);}
    if (hi < limit_margin_) {tau_lim(i) -= limit_stiffness_ * (limit_margin_ - hi);}
  }
  tau += N * (nullspace_stiffness_ * (q_null_ - q_) + tau_lim - nullspace_damping_ * dq_);

  if (gravity_compensation_) {
    dyn_->JntToGravity(q_kdl_, g_kdl_);
    tau += g_kdl_.data;
  }

  for (size_t i = 0; i < 7; ++i) {
    double v = std::clamp(tau(i), tau_prev_(i) - max_torque_rate_, tau_prev_(i) + max_torque_rate_);
    v = std::clamp(v, -torque_limit_[i], torque_limit_[i]);
    if (!std::isfinite(v)) {v = 0.0;}
    command_interfaces_[i].set_value(v);
    tau_prev_(i) = v;
  }

  if (++cycle_ % 10 == 0) {   // 100 Hz at a 1 kHz controller rate
    if (pose_pub_->trylock()) {
      auto & m = pose_pub_->msg_;
      m.header.stamp = time;
      m.header.frame_id = base_link_;
      m.pose.position.x = p.x();
      m.pose.position.y = p.y();
      m.pose.position.z = p.z();
      m.pose.orientation.w = r.w();
      m.pose.orientation.x = r.x();
      m.pose.orientation.y = r.y();
      m.pose.orientation.z = r.z();
      pose_pub_->unlockAndPublish();
    }
    if (twist_pub_->trylock()) {
      auto & m = twist_pub_->msg_;
      m.header.stamp = time;
      m.header.frame_id = base_link_;
      m.twist.linear.x = xdot(0);
      m.twist.linear.y = xdot(1);
      m.twist.linear.z = xdot(2);
      m.twist.angular.x = xdot(3);
      m.twist.angular.y = xdot(4);
      m.twist.angular.z = xdot(5);
      twist_pub_->unlockAndPublish();
    }
  }
  return controller_interface::return_type::OK;
}

}  // namespace sc_control

PLUGINLIB_EXPORT_CLASS(sc_control::CartesianImpedanceController, controller_interface::ControllerInterface)
