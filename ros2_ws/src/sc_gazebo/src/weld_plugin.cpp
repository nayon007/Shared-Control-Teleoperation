// Gripper-handle weld for the shared-control project (paper Sec. 3.2,
// "welded grasp"). A Gazebo Classic world plugin that, on request, creates a
// fixed joint between a robot link (the FR3 hand; Gazebo lumps link8, the
// hand and the TCP frame into fr3_link7 because they hang off fixed joints) and
// a mechanism link (mech_moving), and publishes the wrench that joint
// transmits: F = force applied by the robot to the handle, in the world
// frame, which is the base frame of the FR3.
//
// ROS interface (relative to the node, i.e. /sc_weld/... with the plugin name
// sc_weld and <ros><namespace>/</namespace>):
//   ~/attach   std_srvs/Trigger   weld at the current relative pose
//   ~/detach   std_srvs/Trigger   remove the weld
//   ~/handle_wrench  geometry_msgs/WrenchStamped  force [N] on the handle and
//                    torque [Nm] about the mech_moving origin, world frame
//   ~/attached std_msgs/Bool (transient local)
//
// SDF parameters: robot_model, robot_link, target_model, target_link,
// publish_rate [Hz], break_force [N] (0 = never; otherwise the weld opens
// when |F| exceeds it, a simple stand-in for losing the handle).

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace sc_gazebo
{

class WeldPlugin : public gazebo::WorldPlugin
{
public:
  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    node_ = gazebo_ros::Node::Get(sdf);

    auto param = [&sdf](const std::string & key, const std::string & def) {
        return sdf->HasElement(key) ? sdf->Get<std::string>(key) : def;
      };
    robot_model_ = param("robot_model", "fr3");
    robot_link_ = param("robot_link", "fr3_link7");
    target_model_ = param("target_model", "mechanism");
    target_link_ = param("target_link", "mech_moving");
    const double rate = sdf->HasElement("publish_rate") ? sdf->Get<double>("publish_rate") : 500.0;
    publish_period_ = rate > 0.0 ? 1.0 / rate : 0.0;
    break_force_ = sdf->HasElement("break_force") ? sdf->Get<double>("break_force") : 0.0;

    wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
      "~/handle_wrench", rclcpp::SensorDataQoS());
    attached_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      "~/attached", rclcpp::QoS(1).transient_local());
    PublishAttached(false);

    using Trigger = std_srvs::srv::Trigger;
    attach_srv_ = node_->create_service<Trigger>(
      "~/attach", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        Request(Pending::kAttach, res);
      });
    detach_srv_ = node_->create_service<Trigger>(
      "~/detach", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        Request(Pending::kDetach, res);
      });

    // Joints may only be created or removed on the physics thread, so the
    // services queue a request that the next world update executes.
    update_begin_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&WeldPlugin::OnUpdateBegin, this));
    update_end_ = gazebo::event::Events::ConnectWorldUpdateEnd(
      std::bind(&WeldPlugin::OnUpdateEnd, this));

    RCLCPP_INFO(node_->get_logger(), "weld ready: %s::%s <-> %s::%s (call ~/attach)",
      robot_model_.c_str(), robot_link_.c_str(), target_model_.c_str(), target_link_.c_str());
  }

private:
  enum class Pending { kNone, kAttach, kDetach };

  void Request(Pending what, std_srvs::srv::Trigger::Response::SharedPtr res)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    pending_ = what;
    done_ = false;
    // Physics must be running for the request to execute.
    if (!cv_.wait_for(lock, std::chrono::seconds(5), [this] {return done_;})) {
      pending_ = Pending::kNone;
      res->success = false;
      res->message = "timed out (is the simulation paused?)";
      return;
    }
    res->success = result_ok_;
    res->message = result_msg_;
  }

  void Finish(bool ok, const std::string & msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_ = Pending::kNone;
      result_ok_ = ok;
      result_msg_ = msg;
      done_ = true;
    }
    cv_.notify_all();
    if (ok) {
      RCLCPP_INFO(node_->get_logger(), "%s", msg.c_str());
    } else {
      RCLCPP_WARN(node_->get_logger(), "%s", msg.c_str());
    }
  }

  void OnUpdateBegin()
  {
    Pending what;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      what = pending_;
    }
    if (what == Pending::kAttach) {
      std::string msg;
      const bool ok = Attach(msg);
      Finish(ok, msg);
    } else if (what == Pending::kDetach) {
      const bool was = static_cast<bool>(joint_);
      Detach();
      Finish(true, was ? "weld removed" : "not attached");
    }
  }

  bool Attach(std::string & msg)
  {
    if (joint_) {
      msg = "already attached";
      return true;
    }
    auto robot = world_->ModelByName(robot_model_);
    auto target = world_->ModelByName(target_model_);
    if (!robot || !target) {
      msg = "model not found: " + (robot ? target_model_ : robot_model_);
      return false;
    }
    parent_ = robot->GetLink(robot_link_);
    child_ = target->GetLink(target_link_);
    if (!parent_ || !child_) {
      msg = "link not found: " + (parent_ ? target_link_ : robot_link_);
      return false;
    }
    // Fixed joint at the current relative pose of the two links.
    joint_ = world_->Physics()->CreateJoint("fixed", robot);
    joint_->SetName("sc_weld_joint");
    joint_->Attach(parent_, child_);
    joint_->Load(parent_, child_, ignition::math::Pose3d());
    joint_->SetModel(robot);
    joint_->SetProvideFeedback(true);
    joint_->Init();
    PublishAttached(true);
    const auto rel = child_->WorldPose() - parent_->WorldPose();
    msg = "welded " + robot_link_ + " to " + target_link_ + " (offset " +
      std::to_string(rel.Pos().Length()) + " m)";
    return true;
  }

  void Detach()
  {
    if (!joint_) {
      return;
    }
    joint_->Detach();
    joint_->Fini();
    joint_.reset();
    PublishAttached(false);
  }

  void OnUpdateEnd()
  {
    if (!joint_) {
      return;
    }
    const double t = world_->SimTime().Double();
    if (publish_period_ > 0.0 && t - last_publish_ < publish_period_ - 1e-9) {
      return;
    }
    last_publish_ = t;

    // body2 = child = mech_moving. Gazebo returns body2Force/Torque as the
    // wrench the CHILD applies ON THE JOINT, in the child link frame, about
    // the joint anchor (= child origin, the joint was loaded at identity).
    // The robot's wrench on the handle is its negative, rotated to world.
    // Verified in sim: a +10 N opening push on the drawer gives
    // body2Force = (+9.7..10, ...) along the child x axis = opening direction.
    const gazebo::physics::JointWrench w = joint_->GetForceTorque(0u);
    const auto R = child_->WorldPose().Rot();
    const auto f = -R.RotateVector(w.body2Force);
    const auto tq = -R.RotateVector(w.body2Torque);

    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp.sec = static_cast<int32_t>(world_->SimTime().sec);
    msg.header.stamp.nanosec = static_cast<uint32_t>(world_->SimTime().nsec);
    msg.header.frame_id = "world";
    msg.wrench.force.x = f.X();
    msg.wrench.force.y = f.Y();
    msg.wrench.force.z = f.Z();
    msg.wrench.torque.x = tq.X();
    msg.wrench.torque.y = tq.Y();
    msg.wrench.torque.z = tq.Z();
    wrench_pub_->publish(msg);

    if (break_force_ > 0.0 && f.Length() > break_force_) {
      RCLCPP_WARN(node_->get_logger(), "|F| = %.1f N > break_force %.1f N: handle lost",
        f.Length(), break_force_);
      Detach();
    }
  }

  void PublishAttached(bool attached)
  {
    std_msgs::msg::Bool msg;
    msg.data = attached;
    attached_pub_->publish(msg);
  }

  gazebo::physics::WorldPtr world_;
  gazebo_ros::Node::SharedPtr node_;
  std::string robot_model_, robot_link_, target_model_, target_link_;
  double publish_period_{0.002}, last_publish_{-1.0}, break_force_{0.0};

  gazebo::physics::JointPtr joint_;
  gazebo::physics::LinkPtr parent_, child_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr attached_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attach_srv_, detach_srv_;
  gazebo::event::ConnectionPtr update_begin_, update_end_;

  std::mutex mutex_;
  std::condition_variable cv_;
  Pending pending_{Pending::kNone};
  bool done_{false}, result_ok_{false};
  std::string result_msg_;
};

GZ_REGISTER_WORLD_PLUGIN(WeldPlugin)

}  // namespace sc_gazebo
