// planner_bt.cpp — Behavior Tree (sotto-step 2)
// Un nodo ROS2 reale + albero di una sola Action: OpenGripper.
// Apre il gripper via follow_joint_trajectory (stesso meccanismo del planner
// lineare). Richiede la sim attiva (server /fr3_gripper/follow_joint_trajectory).
// planner_node resta intatto: questo e' un eseguibile separato.

#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

// ---- Nodo ROS2 che ospita gli action client (condiviso coi nodi BT) ----
class BtRosNode : public rclcpp::Node
{
public:
  BtRosNode() : Node("cube_planner_bt")
  {
    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/fr3_gripper/follow_joint_trajectory");
    RCLCPP_INFO(this->get_logger(), "BtRosNode avviato");
  }

  // Apre/chiude il gripper a una posizione (event-driven, con timeout di sicurezza).
  bool moveGripper(double position, const std::string & label)
  {
    if (!gripper_client_->wait_for_action_server(3s)) {
      RCLCPP_WARN(this->get_logger(), "Gripper server non disponibile (%s)", label.c_str());
      return false;
    }
    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = {"fr3_finger_joint1", "fr3_finger_joint2"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position, position};
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);
    goal.trajectory.points.push_back(point);

    RCLCPP_INFO(this->get_logger(), "Gripper %s (%.3f)...", label.c_str(), position);
    auto send_future = gripper_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), send_future, 5s)
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Gripper %s: goal non inviato", label.c_str());
      return false;
    }
    auto goal_handle = send_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(), "Gripper %s: goal rifiutato", label.c_str());
      return false;
    }
    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future, 10s)
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "Gripper %s: timeout risultato", label.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Gripper %s done", label.c_str());
    return true;
  }

private:
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
};

// ---- Azione BT: apre il gripper ----
class OpenGripper : public BT::SyncActionNode
{
public:
  OpenGripper(const std::string & name, const BT::NodeConfig & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override
  {
    // recupera il nodo ROS dalla blackboard
    auto ros = config().blackboard->get<std::shared_ptr<BtRosNode>>("ros_node");
    bool ok = ros->moveGripper(0.06, "open");
    return ok ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

static const char* xml_tree = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Sequence name="root_sequence">
      <OpenGripper/>
    </Sequence>
  </BehaviorTree>
</root>
)";

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto ros_node = std::make_shared<BtRosNode>();

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<OpenGripper>("OpenGripper");

  auto tree = factory.createTreeFromText(xml_tree);
  // passa il nodo ROS a tutti i nodi BT via blackboard
  tree.rootBlackboard()->set("ros_node", ros_node);

  RCLCPP_INFO(ros_node->get_logger(), "[BT] Tick dell'albero...");
  BT::NodeStatus result = tree.tickWhileRunning();
  RCLCPP_INFO(ros_node->get_logger(), "[BT] Risultato: %s", BT::toStr(result).c_str());

  rclcpp::shutdown();
  return 0;
}
