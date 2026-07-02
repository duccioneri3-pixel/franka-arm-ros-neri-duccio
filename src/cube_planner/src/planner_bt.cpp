// planner_bt.cpp — Behavior Tree (sotto-step 3a)
// BtRosNode completo: move_group MoveIt, /cube_pose, scene, gripper, movimenti
// con retry. Nodi BT sottili che chiamano i metodi del BtRosNode.
// Tappa 3a: logica completa, ma main ticka ancora solo OpenGripper (verifica build).
// planner_node resta intatto.

#include <memory>
#include <vector>
#include <chrono>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using moveit::planning_interface::MoveGroupInterface;
using moveit::planning_interface::PlanningSceneInterface;

// =========================================================================
//  BtRosNode: ospita MoveIt, la posa del cubo e gli action client.
//  Espone metodi di alto livello che i nodi BT chiamano.
// =========================================================================
class BtRosNode : public rclcpp::Node
{
public:
  BtRosNode() : Node("cube_planner_bt"), cube_received_(false)
  {
    // parametri (stessi default del planner lineare)
    declare_parameter("target_x", 0.5);
    declare_parameter("target_y", -0.3);
    declare_parameter("target_z", 0.02);
    declare_parameter("pregrasp_offset", 0.12);
    declare_parameter("grasp_offset", -0.015);  // alzato da -0.035: -0.035 portava
    // il TCP a ~5mm dal tavolo, zona di collisione dita/tavolo -> OMPL non
    // campionava stati validi per il goal. A -0.015 il TCP e' a ~2.5cm dal
    // tavolo, le dita circondano comunque il cubo (lato 4cm).
    declare_parameter("lift_offset", 0.35);
    declare_parameter("transport_offset", 0.45);
    declare_parameter("place_offset", 0.06);
    declare_parameter("retreat_offset", 0.30);
    declare_parameter("release_lift_offset", 0.15);
    declare_parameter("gripper_open", 0.06);
    declare_parameter("gripper_close", 0.0);
    declare_parameter("planning_retries", 3);

    // --- Parametri di scena (configurabili sim/reale) ---
    // Default identici alla scena di simulazione. Per l'hardware reale,
    // sovrascrivere via file YAML con le misure del laboratorio.
    declare_parameter("obstacle_dimensions", std::vector<double>{0.5, 0.2, 0.40});
    declare_parameter("obstacle_position",   std::vector<double>{0.5, 0.0, 0.15});
    declare_parameter("table_dimensions",    std::vector<double>{1.2, 1.2, 0.02});
    declare_parameter("table_position",      std::vector<double>{0.5, 0.0, -0.01});
    declare_parameter("cube_size", 0.04);

    // Posa di riposo (ready) come parametro: 7 angoli giunto in radianti.
    // Default = posa di ready standard FR3. Sul reale puo' essere ridefinita
    // (es. posa piu' conservativa per il laboratorio) senza ricompilare.
    declare_parameter("home_joint_positions",
      std::vector<double>{0.0, -0.785398, 0.0, -2.356194, 0.0, 1.570796, 0.785398});

    // Verifica della presa: dopo la chiusura, la larghezza del gripper deve
    // essere vicina a cube_size (dita ferme sull'oggetto). Se vicina a 0, il
    // gripper si e' chiuso a vuoto -> presa fallita. Tolleranza configurabile.
    declare_parameter("grasp_check_tolerance", 0.015);  // m, scarto ammesso da cube_size
    declare_parameter("grasp_check_enabled", true);     // disattivabile se serve

    // Soglia minima di completamento del path cartesiano del LIFT per accettarlo
    // senza cadere su OMPL. Il lift e' verticale: anche un completamento parziale
    // alza il cubo dal tavolo ed e' utile. Piu' bassa della soglia generale (0.9)
    // perche' qui un 80% e' comunque un sollevamento valido, ed evita il fallback
    // OMPL che su questo movimento fallisce spesso (goal tree non campionabile).
    declare_parameter("lift_cartesian_min_fraction", 0.8);

    gripper_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
      this, "/fr3_gripper/follow_joint_trajectory");
    cube_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/cube_pose", 10,
      std::bind(&BtRosNode::cubeCallback, this, std::placeholders::_1));

    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&BtRosNode::jointStateCallback, this, std::placeholders::_1));
    detector_enable_pub_ = create_publisher<std_msgs::msg::Bool>("/detector_enable", 10);

    RCLCPP_INFO(get_logger(), "BtRosNode avviato");
  }

  // move_group va costruito dopo shared_from_this, con executor in spinning.
  void initMoveGroup()
  {
    move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(), "fr3_arm");
    move_group_->setPlanningTime(15.0);
    move_group_->setNumPlanningAttempts(10);
    move_group_->setMaxVelocityScalingFactor(0.2);
    move_group_->setMaxAccelerationScalingFactor(0.2);
    move_group_->setPoseReferenceFrame("world");
    move_group_->allowReplanning(true);
    move_group_->setGoalPositionTolerance(0.01);
    move_group_->setGoalOrientationTolerance(0.01);
  }

  bool waitForCube(std::chrono::seconds timeout = 120s)
  {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
      { std::lock_guard<std::mutex> lock(cube_mutex_);
        if (cube_received_) return true; }
      if (std::chrono::steady_clock::now() - start > timeout) {
        RCLCPP_ERROR(get_logger(), "Timeout: nessuna posa cubo"); return false;
      }
      rate.sleep();
    }
    return false;
  }

  geometry_msgs::msg::PoseStamped cubePose()
  {
    std::lock_guard<std::mutex> lock(cube_mutex_);
    return cube_pose_;
  }

  // Larghezza di apertura del gripper = somma dei due semi-spostamenti delle dita.
  // Letta da /joint_states cercando i giunti PER NOME (l'ordine nel messaggio
  // non e' garantito). Ritorna -1 se non ancora disponibile.
  double gripperWidth()
  {
    std::lock_guard<std::mutex> lock(js_mutex_);
    if (!js_received_) return -1.0;
    return finger1_ + finger2_;
  }

  double p(const std::string & name) { return get_parameter(name).as_double(); }

  geometry_msgs::msg::Pose makePose(double x, double y, double z)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x; pose.position.y = y; pose.position.z = z;
    pose.orientation.x = 1.0; pose.orientation.y = 0.0;
    pose.orientation.z = 0.0; pose.orientation.w = 0.0;
    return pose;
  }

  void setupScene()
  {
    auto cube = cubePose();
    double cx = cube.pose.position.x, cy = cube.pose.position.y, cz = cube.pose.position.z;
    std::vector<moveit_msgs::msg::CollisionObject> objects;

    auto addBox = [&](const std::string & id, double sx, double sy, double sz,
                      double px, double py, double pz) {
      moveit_msgs::msg::CollisionObject o;
      o.id = id; o.header.frame_id = "world";
      shape_msgs::msg::SolidPrimitive prim;
      prim.type = prim.BOX; prim.dimensions = {sx, sy, sz};
      geometry_msgs::msg::Pose ps;
      ps.position.x = px; ps.position.y = py; ps.position.z = pz; ps.orientation.w = 1.0;
      o.primitives.push_back(prim); o.primitive_poses.push_back(ps); o.operation = o.ADD;
      objects.push_back(o);
    };

    // Ostacolo e tavolo: descritti da parametri (adattabili al laboratorio reale).
    auto od = get_parameter("obstacle_dimensions").as_double_array();
    auto op = get_parameter("obstacle_position").as_double_array();
    auto td = get_parameter("table_dimensions").as_double_array();
    auto tp = get_parameter("table_position").as_double_array();
    double cs = get_parameter("cube_size").as_double();

    addBox("obstacle", od[0], od[1], od[2], op[0], op[1], op[2]);
    addBox("table",    td[0], td[1], td[2], tp[0], tp[1], tp[2]);
    // Cubo: dimensione da parametro, posizione dal detector (/cube_pose).
    addBox("cube",     cs, cs, cs, cx, cy, cz);

    psi_.applyCollisionObjects(objects);
    RCLCPP_INFO(get_logger(), "Planning scene configurata");
    rclcpp::sleep_for(500ms);
  }

  // movePose con retry (stesso meccanismo validato nel planner lineare)
  bool movePose(const geometry_msgs::msg::Pose & target, const std::string & label)
  {
    int retries = static_cast<int>(get_parameter("planning_retries").as_int());
    if (retries < 1) retries = 1;
    move_group_->setPoseTarget(target);
    MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= retries; ++attempt) {
      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        planned = true;
        if (attempt > 1)
          RCLCPP_INFO(get_logger(), "Piano [%s] riuscito al tentativo %d/%d",
                      label.c_str(), attempt, retries);
        break;
      }
      RCLCPP_WARN(get_logger(), "Piano [%s] fallito (tentativo %d/%d)",
                  label.c_str(), attempt, retries);
    }
    if (!planned) {
      RCLCPP_ERROR(get_logger(), "Piano [%s] FALLITO dopo %d tentativi", label.c_str(), retries);
      return false;
    }
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Esecuzione [%s] FALLITA", label.c_str()); return false;
    }
    RCLCPP_INFO(get_logger(), "[%s] OK", label.c_str());
    return true;
  }

  bool cartesianMove(const std::vector<geometry_msgs::msg::Pose> & waypoints,
                     const std::string & label, double min_fraction = 0.9)
  {
    // Assestamento: il movimento precedente puo' avere un moto residuo. Il
    // path cartesiano e' rigido sul punto di partenza (tolleranza 0.01 rad in
    // esecuzione), quindi attendiamo che il braccio si fermi e ri-ancoriamo
    // lo start state allo stato CORRENTE prima di calcolare. Evita l'errore
    // "start point deviates from current robot state". Rilevante sul reale,
    // dove l'inerzia rende l'assestamento piu' lento.
    rclcpp::sleep_for(300ms);
    move_group_->setStartStateToCurrentState();

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_->computeCartesianPath(waypoints, 0.01, 0.0, trajectory);
    if (fraction < min_fraction) {
      RCLCPP_ERROR(get_logger(), "Cartesian [%s] %.0f%% (<%.0f%%) — fallback movePose",
                   label.c_str(), fraction * 100.0, min_fraction * 100.0);
      return movePose(waypoints.back(), label + "_fallback");
    }
    MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Esecuzione cartesian [%s] FALLITA", label.c_str()); return false;
    }
    RCLCPP_INFO(get_logger(), "Cartesian [%s] OK (%.0f%%)", label.c_str(), fraction * 100.0);
    return true;
  }
// Abilita/disabilita il detector via topic. Dopo il grasp lo spegniamo:
  // il cubo e' tra le dita, cercarlo a terra e' inutile e riempie i log.
  void setDetector(bool on)
  {
    std_msgs::msg::Bool m; m.data = on;
    detector_enable_pub_->publish(m);
    RCLCPP_INFO(get_logger(), "[detector] %s", on ? "abilitato" : "disabilitato");
  }
  bool attachCube()
  {
    std::vector<std::string> touch = {"fr3_hand", "fr3_leftfinger", "fr3_rightfinger"};
    if (!move_group_->attachObject("cube", "fr3_hand", touch)) {
      RCLCPP_ERROR(get_logger(), "Attach FALLITO"); return false;
    }
    setDetector(false);  // cubo afferrato: spegni la detection
    rclcpp::sleep_for(1s); return true;
  }

  bool detachCube()
  {
    // Stacca il cubo dal gripper E lo rimuove dalla planning scene.
    // Il solo detachObject scollega ma lascia il cubo-oggetto nella posizione
    // di presa (tra le dita): i movimenti successivi (post-release-lift,
    // retreat, GoHome) lo troverebbero in collisione con fr3_hand e non
    // potrebbero pianificare. Rimuovendolo, la scena resta pulita dopo il
    // rilascio. (Il cubo reale e' gia' posato sul target; nel modello di
    // planning non serve piu'.)
    move_group_->detachObject("cube");
    rclcpp::sleep_for(300ms);
    psi_.removeCollisionObjects({"cube"});
    rclcpp::sleep_for(300ms);
    return true;
  }

  // Verifica sensoriale della presa: confronta la larghezza misurata del
  // gripper con la dimensione attesa del cubo. Larghezza ~ cube_size -> dita
  // ferme sull'oggetto (presa OK); larghezza ~ 0 -> chiuso a vuoto (fallita).
  // NB: in simulazione la larghezza segue il comando di posizione piu' che la
  // presenza fisica del cubo; la logica e' corretta e pronta per il reale.
  bool checkGrasp()
  {
    if (!get_parameter("grasp_check_enabled").as_bool()) {
      RCLCPP_INFO(get_logger(), "[grasp-check] disabilitato, salto la verifica");
      return true;
    }
    double width = gripperWidth();
    if (width < 0.0) {
      RCLCPP_WARN(get_logger(), "[grasp-check] larghezza non disponibile (/joint_states), salto");
      return true;  // non blocchiamo se il dato manca
    }
    double expected = get_parameter("cube_size").as_double();
    double tol = get_parameter("grasp_check_tolerance").as_double();
    RCLCPP_INFO(get_logger(),
      "[grasp-check] larghezza=%.4f m, attesa~%.4f m (tol %.4f)", width, expected, tol);
    if (std::abs(width - expected) <= tol) {
      RCLCPP_INFO(get_logger(), "[grasp-check] presa VERIFICATA");
      return true;
    }
    RCLCPP_ERROR(get_logger(),
      "[grasp-check] presa NON valida (larghezza %.4f lontana da %.4f) -> fallimento",
      width, expected);
    return false;
  }

  bool moveGripper(double position, const std::string & label)
  {
    if (!gripper_client_->wait_for_action_server(3s)) {
      RCLCPP_WARN(get_logger(), "Gripper server non disponibile (%s)", label.c_str());
      return false;
    }
    auto goal = FollowJointTrajectory::Goal();
    goal.trajectory.joint_names = {"fr3_finger_joint1", "fr3_finger_joint2"};
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = {position, position};
    point.time_from_start = rclcpp::Duration::from_seconds(2.0);
    goal.trajectory.points.push_back(point);
    RCLCPP_INFO(get_logger(), "Gripper %s (%.3f)...", label.c_str(), position);
    // L'executor gira gia' in un thread separato: NON spinnare qui (il nodo
    // e' gia' nell'executor). Aspettiamo solo i future, che l'executor avanza.
    auto sf = gripper_client_->async_send_goal(goal);
    if (sf.wait_for(5s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Gripper %s: goal non inviato in tempo", label.c_str());
      return false;
    }
    auto gh = sf.get();
    if (!gh) { RCLCPP_WARN(get_logger(), "Gripper %s rifiutato", label.c_str()); return false; }
    auto rf = gripper_client_->async_get_result(gh);
    if (rf.wait_for(10s) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Gripper %s: timeout risultato", label.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Gripper %s done", label.c_str());
    return true;
  }

  // Porta il braccio alla posa di ready nota (giunti espliciti, no IK).
  // Usata come recovery: stato sicuro e deterministico dopo un fallimento.
  bool goHome()
  {
    RCLCPP_INFO(get_logger(), "[RECOVERY] GoHome: ritorno alla posa di ready...");
    std::vector<double> ready = get_parameter("home_joint_positions").as_double_array();
    move_group_->setJointValueTarget(ready);
    int retries = static_cast<int>(get_parameter("planning_retries").as_int());
    if (retries < 1) retries = 1;
    MoveGroupInterface::Plan plan;
    bool planned = false;
    for (int attempt = 1; attempt <= retries; ++attempt) {
      if (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS) { planned = true; break; }
      RCLCPP_WARN(get_logger(), "[RECOVERY] GoHome piano fallito (tentativo %d/%d)", attempt, retries);
    }
    if (!planned) { RCLCPP_ERROR(get_logger(), "[RECOVERY] GoHome: pianificazione fallita"); return false; }
    if (move_group_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "[RECOVERY] GoHome: esecuzione fallita"); return false;
    }
    RCLCPP_INFO(get_logger(), "[RECOVERY] GoHome OK - robot in posa sicura");
    return true;
  }

private:
  void cubeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(cube_mutex_);
    cube_pose_ = *msg; cube_received_ = true;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(js_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == "fr3_finger_joint1") finger1_ = msg->position[i];
      else if (msg->name[i] == "fr3_finger_joint2") finger2_ = msg->position[i];
    }
    js_received_ = true;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cube_sub_;
  rclcpp_action::Client<FollowJointTrajectory>::SharedPtr gripper_client_;
  std::shared_ptr<MoveGroupInterface> move_group_;
  PlanningSceneInterface psi_;
  geometry_msgs::msg::PoseStamped cube_pose_;
  bool cube_received_;
  std::mutex cube_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr detector_enable_pub_;
  double finger1_ = 0.0, finger2_ = 0.0;
  bool js_received_ = false;
  std::mutex js_mutex_;
};

// =========================================================================
//  Nodi BT (sottili: chiamano i metodi del BtRosNode via blackboard)
// =========================================================================

// helper: recupera il BtRosNode dalla blackboard
static std::shared_ptr<BtRosNode> rosOf(const BT::TreeNode & node)
{
  return node.config().blackboard->get<std::shared_ptr<BtRosNode>>("ros_node");
}

#define BT_OK(cond) ((cond) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE)

class SetupScene : public BT::SyncActionNode {
public:
  SetupScene(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { rosOf(*this)->setupScene(); return BT::NodeStatus::SUCCESS; }
};

class OpenGripper : public BT::SyncActionNode {
public:
  OpenGripper(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto r = rosOf(*this); return BT_OK(r->moveGripper(r->p("gripper_open"), "open"));
  }
};

class CloseGripper : public BT::SyncActionNode {
public:
  CloseGripper(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto r = rosOf(*this); return BT_OK(r->moveGripper(r->p("gripper_close"), "close"));
  }
};

class PreGrasp : public BT::SyncActionNode {
public:
  PreGrasp(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto r = rosOf(*this); auto cube = r->cubePose();
    double cx = cube.pose.position.x, cy = cube.pose.position.y, cz = cube.pose.position.z;
    return BT_OK(r->movePose(r->makePose(cx, cy, cz + r->p("pregrasp_offset")), "pre-grasp"));
  }
};

class Approach : public BT::SyncActionNode {
public:
  Approach(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto r = rosOf(*this); auto cube = r->cubePose();
    double cx = cube.pose.position.x, cy = cube.pose.position.y, cz = cube.pose.position.z;
    std::vector<geometry_msgs::msg::Pose> wp = {
      r->makePose(cx, cy, cz + r->p("pregrasp_offset")),
      r->makePose(cx, cy, cz + r->p("grasp_offset"))
    };
    return BT_OK(r->cartesianMove(wp, "approach"));
  }
};

class AttachCube : public BT::SyncActionNode {
public:
  AttachCube(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { return BT_OK(rosOf(*this)->attachCube()); }
};

class CheckGrasp : public BT::SyncActionNode {
public:
  CheckGrasp(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { return BT_OK(rosOf(*this)->checkGrasp()); }
};

class DetachCube : public BT::SyncActionNode {
public:
  DetachCube(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { return BT_OK(rosOf(*this)->detachCube()); }
};

class Lift : public BT::SyncActionNode {
public:
  Lift(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto r = rosOf(*this); auto cube = r->cubePose();
    double cx = cube.pose.position.x, cy = cube.pose.position.y, cz = cube.pose.position.z;
    // Lift come movimento CARTESIANO verticale (deterministico, no RRTConnect):
    // dalla posa di grasp sale dritto. cartesianMove ha gia' fallback su movePose
    // se il percorso rettilineo non e' valido.
    std::vector<geometry_msgs::msg::Pose> wp = {
      r->makePose(cx, cy, cz + r->p("grasp_offset")),
      r->makePose(cx, cy, cz + r->p("lift_offset"))
    };
    // Soglia abbassata: un lift verticale parziale alza comunque il cubo,
    // meglio che cadere su OMPL (che qui fallisce spesso).
    return BT_OK(r->cartesianMove(wp, "lift", r->p("lift_cartesian_min_fraction")));
  }
};

class Transport : public BT::SyncActionNode {
public:
  Transport(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto r = rosOf(*this);
    return BT_OK(r->movePose(
      r->makePose(r->p("target_x"), r->p("target_y"), r->p("target_z") + r->p("transport_offset")),
      "transport"));
  }
};

class Place : public BT::SyncActionNode {
public:
  Place(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto r = rosOf(*this);
    return BT_OK(r->movePose(
      r->makePose(r->p("target_x"), r->p("target_y"), r->p("target_z") + r->p("place_offset")),
      "place"));
  }
};

class ReleaseLift : public BT::SyncActionNode {
public:
  ReleaseLift(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto r = rosOf(*this);
    return BT_OK(r->movePose(
      r->makePose(r->p("target_x"), r->p("target_y"),
                  r->p("target_z") + r->p("place_offset") + r->p("release_lift_offset")),
      "post-release-lift"));
  }
};

class Retreat : public BT::SyncActionNode {
public:
  Retreat(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override {
    auto r = rosOf(*this);
    return BT_OK(r->movePose(
      r->makePose(r->p("target_x"), r->p("target_y"), r->p("target_z") + r->p("retreat_offset")),
      "retreat"));
  }
};

class GoHome : public BT::SyncActionNode {
public:
  GoHome(const std::string & n, const BT::NodeConfig & c) : BT::SyncActionNode(n, c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override { return BT_OK(rosOf(*this)->goHome()); }
};

// Albero: l'intera sequenza pick-and-place, leggibile a colpo d'occhio.
static const char* xml_tree = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <Fallback name="task_with_recovery">
      <Sequence name="pick_and_place">
        <SetupScene/>
        <OpenGripper/>
        <PreGrasp/>
        <Approach/>
        <CloseGripper/>
        <AttachCube/>
        <CheckGrasp/>
        <Lift/>
        <Transport/>
        <Place/>
        <DetachCube/>
        <OpenGripper/>
        <ReleaseLift/>
        <Retreat/>
        <GoHome/>
      </Sequence>
      <Sequence name="recovery">
        <DetachCube/>
        <OpenGripper/>
        <GoHome/>
      </Sequence>
    </Fallback>
  </BehaviorTree>
</root>
)";

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto ros_node = std::make_shared<BtRosNode>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ros_node);
  std::thread spinner([&executor]() { executor.spin(); });

  ros_node->initMoveGroup();

  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SetupScene>("SetupScene");
  factory.registerNodeType<OpenGripper>("OpenGripper");
  factory.registerNodeType<CloseGripper>("CloseGripper");
  factory.registerNodeType<PreGrasp>("PreGrasp");
  factory.registerNodeType<Approach>("Approach");
  factory.registerNodeType<AttachCube>("AttachCube");
  factory.registerNodeType<CheckGrasp>("CheckGrasp");
  factory.registerNodeType<DetachCube>("DetachCube");
  factory.registerNodeType<Lift>("Lift");
  factory.registerNodeType<Transport>("Transport");
  factory.registerNodeType<Place>("Place");
  factory.registerNodeType<ReleaseLift>("ReleaseLift");
  factory.registerNodeType<Retreat>("Retreat");
  factory.registerNodeType<GoHome>("GoHome");

  auto tree = factory.createTreeFromText(xml_tree);
  tree.rootBlackboard()->set("ros_node", ros_node);

  RCLCPP_INFO(ros_node->get_logger(), "[BT] In attesa della posa del cubo...");
  if (!ros_node->waitForCube()) {
    RCLCPP_ERROR(ros_node->get_logger(), "Nessuna posa cubo, esco.");
    executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
  }

  RCLCPP_INFO(ros_node->get_logger(), "[BT] Tick dell'albero pick-and-place...");
  BT::NodeStatus result = tree.tickWhileRunning();
  RCLCPP_INFO(ros_node->get_logger(), "[BT] TASK COMPLETATO! Risultato: %s",
              BT::toStr(result).c_str());

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
