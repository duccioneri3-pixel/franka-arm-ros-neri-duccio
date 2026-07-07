#include <memory>
#include <vector>
#include <chrono>
#include <mutex>
#include <thread>

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
#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using Grasp = franka_msgs::action::Grasp;
using Move  = franka_msgs::action::Move;

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

    this->declare_parameter("pregrasp_offset", 0.12);
    this->declare_parameter("grasp_offset", -0.035);
    this->declare_parameter("lift_offset", 0.35);
    this->declare_parameter("transport_offset", 0.45);
    this->declare_parameter("place_offset", 0.06);
    this->declare_parameter("retreat_offset", 0.30);

    this->declare_parameter("planning_retries", 3);  // tentativi di pianificazione per ogni movePose
    this->declare_parameter("release_lift_offset", 0.15);

    // --- Parametri di scena (configurabili sim/reale) ---
    // Default identici alla scena di simulazione. Per l'hardware reale,
    // sovrascrivere via file YAML con le misure del laboratorio.
    this->declare_parameter("obstacle_dimensions", std::vector<double>{0.5, 0.2, 0.40});
    this->declare_parameter("obstacle_position",   std::vector<double>{0.5, 0.0, 0.15});
    this->declare_parameter("table_dimensions",    std::vector<double>{1.2, 1.2, 0.02});
    this->declare_parameter("table_position",      std::vector<double>{0.5, 0.0, -0.01});
    this->declare_parameter("cube_size", 0.04);

    this->declare_parameter("gripper_open", 0.06);
    this->declare_parameter("gripper_close", 0.0);

    // --- Gripper reale (Franka): usato solo se use_real_gripper=true ---
    this->declare_parameter("use_real_gripper", false);
    this->declare_parameter("real_open_width", 0.08);      // m, apertura Move
    this->declare_parameter("grasp_width", 0.04);          // m, larghezza cubo
    this->declare_parameter("gripper_speed", 0.1);         // m/s
    this->declare_parameter("gripper_force", 20.0);        // N
    this->declare_parameter("grasp_epsilon_inner", 0.005); // m
    this->declare_parameter("grasp_epsilon_outer", 0.005); // m
    this->declare_parameter("move_action_name", "/fr3_gripper/move");
    this->declare_parameter("grasp_action_name", "/fr3_gripper/grasp");

    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/fr3_gripper/follow_joint_trajectory");

    move_client_ = rclcpp_action::create_client<Move>(
      this, this->get_parameter("move_action_name").as_string());
    grasp_client_ = rclcpp_action::create_client<Grasp>(
      this, this->get_parameter("grasp_action_name").as_string());

    cube_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cube_pose", 10,
      std::bind(&CubePlanner::cubeCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "CubePlanner avviato, attendo /cube_pose...");
  }

  // Aspetta la prima posa del cubo (bloccante, con timeout di sicurezza).
  // Chiamato dal main thread, mentre l'executor gira nel suo thread.
  bool waitForCube(std::chrono::seconds timeout = 120s)
  {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lock(cube_mutex_);
        if (cube_received_) {
          return true;
        }
      }
      if (std::chrono::steady_clock::now() - start > timeout) {
        RCLCPP_ERROR(this->get_logger(), "Timeout: nessuna posa cubo ricevuta");
        return false;
      }
      rate.sleep();
    }
    return false;
  }

  void executePickPlace()
  {
    // move_group costruito UNA volta, qui, con l'executor gia' in spinning
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

    // copia thread-safe della posa cubo
    geometry_msgs::msg::PoseStamped cube;
    {
      std::lock_guard<std::mutex> lock(cube_mutex_);
      cube = cube_pose_;
    }
    double cx = cube.pose.position.x;
    double cy = cube.pose.position.y;
    double cz = cube.pose.position.z;
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

    setupScene(psi, cx, cy, cz);
    rclcpp::sleep_for(1s);

    RCLCPP_INFO(this->get_logger(), "Step 1: Open gripper...");
    openGripper();

    RCLCPP_INFO(this->get_logger(), "Step 2: Pre-grasp...");
    if (!movePose(move_group, makePose(cx, cy, cz + pregrasp_off), "pre-grasp")) return;

    RCLCPP_INFO(this->get_logger(), "Step 3: Approach...");
    std::vector<geometry_msgs::msg::Pose> approach_waypoints = {
      makePose(cx, cy, cz + pregrasp_off),
      makePose(cx, cy, cz + grasp_off)
    };
    if (!cartesianMove(move_group, approach_waypoints, "approach")) return;

    RCLCPP_INFO(this->get_logger(), "Step 4: Grasp...");
    closeGripper();
    std::vector<std::string> touch_links = {"fr3_hand", "fr3_leftfinger", "fr3_rightfinger"};
    if (!move_group.attachObject("cube", "fr3_hand", touch_links)) {
      RCLCPP_ERROR(this->get_logger(), "Attach del cubo FALLITO");
    }
    rclcpp::sleep_for(1s);

    RCLCPP_INFO(this->get_logger(), "Step 5: Lift...");
    if (!movePose(move_group, makePose(cx, cy, cz + lift_off), "lift")) return;

    RCLCPP_INFO(this->get_logger(), "Step 6: Transport...");
    if (!movePose(move_group, makePose(tx, ty, tz + transport_off), "transport")) return;

    RCLCPP_INFO(this->get_logger(), "Step 7: Place...");
    if (!movePose(move_group, makePose(tx, ty, tz + place_off), "place")) return;

    RCLCPP_INFO(this->get_logger(), "Step 8: Release...");
    move_group.detachObject("cube");
    openGripper();
    rclcpp::sleep_for(1s);

    double release_lift_off = this->get_parameter("release_lift_offset").as_double();
    RCLCPP_INFO(this->get_logger(), "Step 8.5: Post-release lift (esce dalle dita)...");
    if (!movePose(move_group, makePose(tx, ty, tz + place_off + release_lift_off), "post-release-lift")) {
      RCLCPP_WARN(this->get_logger(), "Post-release lift fallito, provo retreat comunque");
    }

    RCLCPP_INFO(this->get_logger(), "Step 9: Retreat...");
    movePose(move_group, makePose(tx, ty, tz + retreat_off), "retreat");

    RCLCPP_INFO(this->get_logger(), "TASK COMPLETATO!");
  }

private:

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cube_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  rclcpp_action::Client<Move>::SharedPtr  move_client_;
  rclcpp_action::Client<Grasp>::SharedPtr grasp_client_;
  geometry_msgs::msg::PoseStamped cube_pose_;
  bool cube_received_;
  std::mutex cube_mutex_;

  void cubeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cube_mutex_);
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

  void openGripper()
  {
    if (this->get_parameter("use_real_gripper").as_bool()) {
      realMove(this->get_parameter("real_open_width").as_double(), "open");
    } else {
      controlGripper(this->get_parameter("gripper_open").as_double(), "open");
    }
  }

  void closeGripper()
  {
    if (this->get_parameter("use_real_gripper").as_bool()) {
      realGrasp(this->get_parameter("grasp_width").as_double(), "close");
    } else {
      controlGripper(this->get_parameter("gripper_close").as_double(), "close");
    }
  }

  // --- Percorso REALE: Move (open) e Grasp in forza (close) ---
  // Stesso pattern event-driven di controlGripper: attende il server, invia il
  // goal, attende handle e result, con timeout di sicurezza (non attese fisse).
  void realMove(double width, const std::string & label)
  {
    if (!move_client_->wait_for_action_server(3s)) {
      RCLCPP_WARN(this->get_logger(), "[REALE] Move server non disponibile, salto %s", label.c_str());
      return;
    }
    auto goal = Move::Goal();
    goal.width = width;
    goal.speed = this->get_parameter("gripper_speed").as_double();
    RCLCPP_INFO(this->get_logger(), "[REALE] Move %s (width=%.3f)...", label.c_str(), width);
    auto send_future = move_client_->async_send_goal(goal);
    if (send_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "[REALE] Move %s: goal non inviato in tempo", label.c_str());
      return;
    }
    auto goal_handle = send_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(), "[REALE] Move %s: goal rifiutato", label.c_str());
      return;
    }
    auto result_future = move_client_->async_get_result(goal_handle);
    if (result_future.wait_for(10s) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "[REALE] Move %s: timeout sul risultato", label.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[REALE] Move %s done", label.c_str());
  }

  void realGrasp(double width, const std::string & label)
  {
    if (!grasp_client_->wait_for_action_server(3s)) {
      RCLCPP_WARN(this->get_logger(), "[REALE] Grasp server non disponibile, salto %s", label.c_str());
      return;
    }
    auto goal = Grasp::Goal();
    goal.width = width;
    goal.speed = this->get_parameter("gripper_speed").as_double();
    goal.force = this->get_parameter("gripper_force").as_double();
    goal.epsilon.inner = this->get_parameter("grasp_epsilon_inner").as_double();
    goal.epsilon.outer = this->get_parameter("grasp_epsilon_outer").as_double();
    RCLCPP_INFO(this->get_logger(),
      "[REALE] Grasp %s (width=%.3f force=%.1fN)...", label.c_str(), width, goal.force);
    auto send_future = grasp_client_->async_send_goal(goal);
    if (send_future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "[REALE] Grasp %s: goal non inviato in tempo", label.c_str());
      return;
    }
    auto goal_handle = send_future.get();
    if (!goal_handle) {
      RCLCPP_WARN(this->get_logger(), "[REALE] Grasp %s: goal rifiutato", label.c_str());
      return;
    }
    auto result_future = grasp_client_->async_get_result(goal_handle);
    if (result_future.wait_for(10s) != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "[REALE] Grasp %s: timeout sul risultato", label.c_str());
      return;
    }
    RCLCPP_INFO(this->get_logger(), "[REALE] Grasp %s done", label.c_str());
  }

  void setupScene(
    moveit::planning_interface::PlanningSceneInterface & psi,
    double cx, double cy, double cz)
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    // Helper per aggiungere un box alla scena.
    auto addBox = [&](const std::string & id, double sx, double sy, double sz,
                      double px, double py, double pz) {
      moveit_msgs::msg::CollisionObject o;
      o.id = id;
      o.header.frame_id = "world";
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = prim.BOX;
      prim.dimensions = {sx, sy, sz};
      geometry_msgs::msg::Pose ps;
      ps.position.x = px; ps.position.y = py; ps.position.z = pz;
      ps.orientation.w = 1.0;
      o.primitives.push_back(prim);
      o.primitive_poses.push_back(ps);
      o.operation = o.ADD;
      objects.push_back(o);
    };

    // Ostacolo e tavolo: descritti da parametri (adattabili al laboratorio reale).
    auto od = this->get_parameter("obstacle_dimensions").as_double_array();
    auto op = this->get_parameter("obstacle_position").as_double_array();
    auto td = this->get_parameter("table_dimensions").as_double_array();
    auto tp = this->get_parameter("table_position").as_double_array();
    double cs = this->get_parameter("cube_size").as_double();

    addBox("obstacle", od[0], od[1], od[2], op[0], op[1], op[2]);
    addBox("table",    td[0], td[1], td[2], tp[0], tp[1], tp[2]);
    // Cubo: dimensione da parametro, posizione dal detector.
    addBox("cube",     cs, cs, cs, cx, cy, cz);

    psi.applyCollisionObjects(objects);
    RCLCPP_INFO(this->get_logger(), "Planning scene configurata");
    rclcpp::sleep_for(500ms);
  }

  bool movePose(
    moveit::planning_interface::MoveGroupInterface & mg,
    const geometry_msgs::msg::Pose & target,
    const std::string & label)
  {
    int retries = static_cast<int>(this->get_parameter("planning_retries").as_int());
    if (retries < 1) retries = 1;

    mg.setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    bool planned = false;
    for (int attempt = 1; attempt <= retries; ++attempt) {
      if (mg.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        if (attempt > 1) {
          RCLCPP_INFO(this->get_logger(),
            "Piano [%s] riuscito al tentativo %d/%d", label.c_str(), attempt, retries);
        }
        break;
      }
      RCLCPP_WARN(this->get_logger(),
        "Piano [%s] fallito (tentativo %d/%d)", label.c_str(), attempt, retries);
    }

    if (!planned) {
      RCLCPP_ERROR(this->get_logger(),
        "Piano [%s] FALLITO dopo %d tentativi", label.c_str(), retries);
      return false;
    }

    if (mg.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(this->get_logger(), "Esecuzione [%s] FALLITA", label.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "[%s] OK", label.c_str());
    return true;
  }

  bool cartesianMove(
    moveit::planning_interface::MoveGroupInterface & mg,
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    const std::string & label)
  {
    // Assestamento + re-seed dello start state prima del path cartesiano:
    // evita "start point deviates from current robot state" quando il
    // movimento precedente ha un moto residuo. Rilevante sul reale (inerzia).
    rclcpp::sleep_for(300ms);
    mg.setStartStateToCurrentState();

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

  bool cartesianMove(
    moveit::planning_interface::MoveGroupInterface & mg,
    const geometry_msgs::msg::Pose & target,
    const std::string & label)
  {
    return cartesianMove(mg, std::vector<geometry_msgs::msg::Pose>{target}, label);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<CubePlanner>(options);

  // Executor in un thread SEPARATO: non blocca il main, processa i callback
  // (inclusi i future del gripper) mentre il task gira.
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  // Il task gira nel main thread, in modo lineare.
  if (node->waitForCube()) {
    node->executePickPlace();
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
