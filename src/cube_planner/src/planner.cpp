#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"

#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

#include "shape_msgs/msg/solid_primitive.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

class CubePlanner : public rclcpp::Node
{
public:
  explicit CubePlanner(const rclcpp::NodeOptions & options)
  : Node("cube_planner", options),
    cube_received_(false)
  {
    this->declare_parameter("target_x", 0.5);
    this->declare_parameter("target_y", -0.3);
    this->declare_parameter("target_z", 0.02);

    // Offset di movimento (relativi a cz del cubo o tz del target)
    this->declare_parameter("pregrasp_offset", 0.12);
    this->declare_parameter("grasp_offset", -0.045);
    this->declare_parameter("lift_offset", 0.35);
    this->declare_parameter("transport_offset", 0.45);
    this->declare_parameter("place_offset", 0.06);
    this->declare_parameter("retreat_offset", 0.30);

    // Gripper
    this->declare_parameter("gripper_open", 0.06);
    this->declare_parameter("gripper_close", 0.0);

    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/fr3_gripper/follow_joint_trajectory");

    cube_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cube_pose", 10,
      std::bind(&CubePlanner::cubeCallback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      5s, std::bind(&CubePlanner::runTask, this));

    RCLCPP_INFO(this->get_logger(), "CubePlanner avviato, attendo 5s e /cube_pose...");
  }

private:

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cube_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  geometry_msgs::msg::PoseStamped cube_pose_;
  bool cube_received_;

  void cubeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    cube_pose_ = *msg;
    cube_received_ = true;
  }

  geometry_msgs::msg::Pose makePose(double x, double y, double z)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = 1.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = 0.0;
    pose.orientation.w = 0.0;
    return pose;
  }

  void controlGripper(double position, const std::string & label)
  {
    if (!gripper_client_->wait_for_action_server(3s)) {
      RCLCPP_WARN(this->get_logger(), "Gripper server non disponibile, salto %s", label.c_str());
      return;
    }

    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = {"fr3_finger_joint1", "fr3_finger_joint2"};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position, position};
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);
    goal.trajectory.points.push_back(point);

    RCLCPP_INFO(this->get_logger(), "Gripper %s (%.3f)...", label.c_str(), position);

    auto send_future = gripper_client_->async_send_goal(goal);
    if (send_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Gripper %s: goal non inviato in tempo", label.c_str());
      return;
    }
    auto goal_handle = send_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(), "Gripper %s: goal rifiutato dal server", label.c_str());
      return;
    }

    auto result_future = gripper_client_->async_get_result(goal_handle);
    if (result_future.wait_for(10s) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Gripper %s: timeout sul risultato", label.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Gripper %s done", label.c_str());
  }

  void openGripper()  { controlGripper(this->get_parameter("gripper_open").as_double(),  "open"); }
  void closeGripper() { controlGripper(this->get_parameter("gripper_close").as_double(), "close"); }

  void setupScene(
    moveit::planning_interface::PlanningSceneInterface & psi,
    double cx, double cy, double cz)
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    // Ostacolo — 0.5x0.2x0.40 @ (0.5, 0.0, 0.15)
    moveit_msgs::msg::CollisionObject obstacle;
    obstacle.id = "obstacle";
    obstacle.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive obs_prim;
    obs_prim.type = obs_prim.BOX;
    obs_prim.dimensions = {0.5, 0.2, 0.40};
    geometry_msgs::msg::Pose obs_pose;
    obs_pose.position.x = 0.5;
    obs_pose.position.y = 0.0;
    obs_pose.position.z = 0.15;
    obs_pose.orientation.w = 1.0;
    obstacle.primitives.push_back(obs_prim);
    obstacle.primitive_poses.push_back(obs_pose);
    obstacle.operation = obstacle.ADD;
    objects.push_back(obstacle);

    // Tavolo
    moveit_msgs::msg::CollisionObject table;
    table.id = "table";
    table.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive table_prim;
    table_prim.type = table_prim.BOX;
    table_prim.dimensions = {1.2, 1.2, 0.02};
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.5;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.01;
    table_pose.orientation.w = 1.0;
    table.primitives.push_back(table_prim);
    table.primitive_poses.push_back(table_pose);
    table.operation = table.ADD;
    objects.push_back(table);

    // Cubo
    moveit_msgs::msg::CollisionObject cube;
    cube.id = "cube";
    cube.header.frame_id = "world";
    shape_msgs::msg::SolidPrimitive cube_prim;
    cube_prim.type = cube_prim.BOX;
    cube_prim.dimensions = {0.04, 0.04, 0.04};
    geometry_msgs::msg::Pose cube_pose_msg;
    cube_pose_msg.position.x = cx;
    cube_pose_msg.position.y = cy;
    cube_pose_msg.position.z = cz;
    cube_pose_msg.orientation.w = 1.0;
    cube.primitives.push_back(cube_prim);
    cube.primitive_poses.push_back(cube_pose_msg);
    cube.operation = cube.ADD;
    objects.push_back(cube);

    psi.applyCollisionObjects(objects);
    RCLCPP_INFO(this->get_logger(), "Planning scene configurata");
    rclcpp::sleep_for(500ms);
  }

  bool movePose(
    moveit::planning_interface::MoveGroupInterface & mg,
    const geometry_msgs::msg::Pose & target,
    const std::string & label)
  {
    mg.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mg.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Piano [%s] FALLITO", label.c_str());
      return false;
    }
    if (mg.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Esecuzione [%s] FALLITA", label.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "[%s] OK", label.c_str());
    return true;
  }

  // cartesianMove con lista di waypoints
  bool cartesianMove(
    moveit::planning_interface::MoveGroupInterface & mg,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    const std::string & label)
  {
    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = mg.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    if (fraction < 0.9) {
      RCLCPP_ERROR(this->get_logger(),
        "Cartesian [%s] FALLITO: %.0f%% — provo con movePose", label.c_str(), fraction * 100.0);
      return movePose(mg, waypoints.back(), label + "_fallback");
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    if (mg.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Esecuzione cartesian [%s] FALLITA", label.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Cartesian [%s] OK (%.0f%%)", label.c_str(), fraction * 100.0);
    return true;
  }

  // overload singolo waypoint
  bool cartesianMove(
    moveit::planning_interface::MoveGroupInterface & mg,
    const geometry_msgs::msg::Pose & target,
    const std::string & label)
  {
    return cartesianMove(mg, std::vector<geometry_msgs::msg::Pose>{target}, label);
  }

  void runTask()
  {
    timer_->cancel();

    if (!cube_received_) {
      RCLCPP_WARN(this->get_logger(), "Cubo non ancora rilevato, riprovo tra 2s...");
      timer_ = this->create_wall_timer(2s, std::bind(&CubePlanner::runTask, this));
      return;
    }

    moveit::planning_interface::MoveGroupInterface move_group(shared_from_this(), "fr3_arm");
    moveit::planning_interface::PlanningSceneInterface psi;

    move_group.setPlanningTime(15.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setPoseReferenceFrame("world");
    move_group.allowReplanning(true);
    move_group.setGoalPositionTolerance(0.01);
    move_group.setGoalOrientationTolerance(0.01);

    double cx = cube_pose_.pose.position.x;
    double cy = cube_pose_.pose.position.y;
    double cz = cube_pose_.pose.position.z;
    double tx = this->get_parameter("target_x").as_double();
    double ty = this->get_parameter("target_y").as_double();
    double tz = this->get_parameter("target_z").as_double();

    double pregrasp_off  = this->get_parameter("pregrasp_offset").as_double();
    double grasp_off     = this->get_parameter("grasp_offset").as_double();
    double lift_off      = this->get_parameter("lift_offset").as_double();
    double transport_off = this->get_parameter("transport_offset").as_double();
    double place_off     = this->get_parameter("place_offset").as_double();
    double retreat_off   = this->get_parameter("retreat_offset").as_double();

    RCLCPP_INFO(this->get_logger(), "Cubo: x=%.3f y=%.3f z=%.3f", cx, cy, cz);
    RCLCPP_INFO(this->get_logger(), "Target: x=%.3f y=%.3f z=%.3f", tx, ty, tz);

    // Configura scena
    setupScene(psi, cx, cy, cz);
    rclcpp::sleep_for(1s);

    // Step 1 — apri gripper
    RCLCPP_INFO(this->get_logger(), "Step 1: Open gripper...");
    openGripper();

    // Step 2 — pre-grasp (movePose — piano sicuro)
    RCLCPP_INFO(this->get_logger(), "Step 2: Pre-grasp...");
    if (!movePose(move_group, makePose(cx, cy, cz + pregrasp_off), "pre-grasp")) return;

    // Step 3 — approach con 2 waypoints: allineati XY poi scendi dritto
    RCLCPP_INFO(this->get_logger(), "Step 3: Approach...");
    std::vector<geometry_msgs::msg::Pose> approach_waypoints = {
      makePose(cx, cy, cz + pregrasp_off),  // allineati XY stessa quota pre-grasp
      makePose(cx, cy, cz + grasp_off)   // scendi dritto verticale
    };

    if (!cartesianMove(move_group, approach_waypoints, "approach")) return;

    // Step 4 — grasp
    RCLCPP_INFO(this->get_logger(), "Step 4: Grasp...");
    closeGripper();
    std::vector<std::string> touch_links = {"fr3_hand", "fr3_leftfinger", "fr3_rightfinger"};
    if (!move_group.attachObject("cube", "fr3_hand", touch_links)) {
      RCLCPP_ERROR(this->get_logger(), "Attach del cubo FALLITO");
    }
    rclcpp::sleep_for(1s);

    // Step 5 — lift (movePose — sali)
    RCLCPP_INFO(this->get_logger(), "Step 5: Lift...");
    if (!movePose(move_group, makePose(cx, cy, cz + lift_off), "lift")) return;

    // Step 6 — transport sopra target (movePose — piano sicuro)
    RCLCPP_INFO(this->get_logger(), "Step 6: Transport...");
    if (!movePose(move_group, makePose(tx, ty, tz + transport_off), "transport")) return;

    // Step 7 — place (movePose — evita il 360°)
    RCLCPP_INFO(this->get_logger(), "Step 7: Place...");
    if (!movePose(move_group, makePose(tx, ty, tz + place_off), "place")) return;

    // Step 8 — release
    RCLCPP_INFO(this->get_logger(), "Step 8: Release...");
    move_group.detachObject("cube");
    openGripper();
    rclcpp::sleep_for(1s);

    // Step 9 — retreat (movePose — piano sicuro)
    RCLCPP_INFO(this->get_logger(), "Step 9: Retreat...");
    movePose(move_group, makePose(tx, ty, tz + retreat_off), "retreat");

    RCLCPP_INFO(this->get_logger(), "TASK COMPLETATO!");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<CubePlanner>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
