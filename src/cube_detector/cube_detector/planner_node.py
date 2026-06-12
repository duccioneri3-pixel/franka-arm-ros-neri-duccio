import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    CollisionObject,
    PlanningScene,
)
from shape_msgs.msg import SolidPrimitive
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class CubePlanner(Node):
    def __init__(self):
        super().__init__('cube_planner')

        self.declare_parameter('target_x', 0.5)
        self.declare_parameter('target_y', -0.3)
        self.declare_parameter('target_z', 0.01)
        self.declare_parameter('obstacle_x', 0.5)
        self.declare_parameter('obstacle_y', 0.0)
        self.declare_parameter('obstacle_z', 0.15)

        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/fr3_gripper/follow_joint_trajectory')

        self.planning_scene_pub = self.create_publisher(
            PlanningScene, '/planning_scene', 10)

        self.cube_pose = None
        self.task_done = False

        self.sub = self.create_subscription(
            PoseStamped, '/cube_pose', self.cube_pose_cb, 10)

        self.get_logger().info('Cube planner started...')
        self.timer = self.create_timer(2.0, self.check_and_run)

    def cube_pose_cb(self, msg):
        self.cube_pose = msg

    def check_and_run(self):
        if self.task_done:
            return
        if not self.move_client.server_is_ready():
            self.get_logger().info('Waiting for MoveIt...', throttle_duration_sec=5.0)
            return
        if self.cube_pose is None:
            self.get_logger().info('Waiting for /cube_pose...', throttle_duration_sec=5.0)
            return

        self.get_logger().info('Ready! Starting task...')
        self.task_done = True
        self.timer.cancel()
        self.remove_obstacle_from_scene()
        time.sleep(0.5)
        self.add_obstacle_to_scene()
        time.sleep(1.0)
        self.run_task()

    def remove_obstacle_from_scene(self):
        obstacle = CollisionObject()
        obstacle.id = 'obstacle'
        obstacle.header.frame_id = 'world'
        obstacle.header.stamp = self.get_clock().now().to_msg()
        obstacle.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obstacle)
        self.planning_scene_pub.publish(scene)
        self.get_logger().info('Obstacle removed from planning scene')

    def add_obstacle_to_scene(self):
        ox = self.get_parameter('obstacle_x').value
        oy = self.get_parameter('obstacle_y').value
        oz = self.get_parameter('obstacle_z').value

        obstacle = CollisionObject()
        obstacle.id = 'obstacle'
        obstacle.header.frame_id = 'world'
        obstacle.header.stamp = self.get_clock().now().to_msg()
        obstacle.operation = CollisionObject.ADD

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.08, 0.08, 0.20]

        pose = Pose()
        pose.position.x = ox
        pose.position.y = oy
        pose.position.z =  0.10
        pose.orientation.w = 1.0

        obstacle.primitives.append(box)
        obstacle.primitive_poses.append(pose)

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obstacle)
        self.planning_scene_pub.publish(scene)
        self.get_logger().info(f'Obstacle added at x={ox} y={oy} z={oz}')

    def move_to_pose(self, target_pose):
        goal = MoveGroup.Goal()
        goal.request.group_name = 'fr3_arm'
        goal.request.num_planning_attempts = 20
        goal.request.allowed_planning_time = 20.0
        goal.request.max_velocity_scaling_factor = 0.1
        goal.request.max_acceleration_scaling_factor = 0.1

        pos_constraint = PositionConstraint()
        pos_constraint.header = target_pose.header
        pos_constraint.link_name = 'fr3_hand_tcp'

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.04]

        bv = BoundingVolume()
        bv.primitives.append(primitive)
        bv.primitive_poses.append(target_pose.pose)
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0

        # ori_constraint = OrientationConstraint()
        # ori_constraint.header = target_pose.header
        # ori_constraint.link_name = 'fr3_hand_tcp'
        # ori_constraint.orientation = target_pose.pose.orientation
        # ori_constraint.absolute_x_axis_tolerance = 0.5
        # ori_constraint.absolute_y_axis_tolerance = 0.5
        #ori_constraint.absolute_z_axis_tolerance = 3.14
        #ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        #constraints.orientation_constraints.append(ori_constraint)
        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(
            f'Planning to '
            f'x={target_pose.pose.position.x:.3f} '
            f'y={target_pose.pose.position.y:.3f} '
            f'z={target_pose.pose.position.z:.3f}')

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result

        if result.error_code.val == 1:
            self.get_logger().info('Motion succeeded!')
            return True
        else:
            self.get_logger().error(f'Motion failed: error code {result.error_code.val}')
            return False

    def control_gripper(self, open=True):
        position = 0.04 if open else 0.0

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [
            'fr3_finger_joint1', 'fr3_finger_joint2']

        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points.append(point)

        self.get_logger().info(f'Gripper {"opening" if open else "closing"}...')

        if not self.gripper_client.server_is_ready():
            self.get_logger().warn('Gripper not ready, skipping')
            return False

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if not future.done():
            self.get_logger().warn('Gripper timeout, continuing anyway')
            return True

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Gripper goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
        self.get_logger().info('Gripper done!')
        return True

    def make_pose(self, x, y, z, ox=0.9239, oy=0.3827, oz=0.0, ow=0.0):
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = ox
        pose.pose.orientation.y = oy
        pose.pose.orientation.z = oz
        pose.pose.orientation.w = ow
        return pose

    def run_task(self):
        cx = self.cube_pose.pose.position.x
        cy = self.cube_pose.pose.position.y
        cz = self.cube_pose.pose.position.z
        tx = self.get_parameter('target_x').value
        ty = self.get_parameter('target_y').value
        tz = self.get_parameter('target_z').value

        self.get_logger().info(f'Cube at x={cx:.3f} y={cy:.3f} z={cz:.3f}')
        self.get_logger().info(f'Target at x={tx:.3f} y={ty:.3f} z={tz:.3f}')

        # Step 1 — apri gripper
        self.get_logger().info('Step 1: Opening gripper...')
        self.control_gripper(open=True)
        time.sleep(2.5)

        # Step 2 — pre-grasp sopra il cubo
        self.get_logger().info('Step 2: Pre-grasp...')
        if not self.move_to_pose(self.make_pose(cx, cy, cz + 0.15)):
            self.get_logger().error('Pre-grasp failed')
            return

        # Step 3 — scendi al cubo
        self.get_logger().info('Step 3: Approaching cube...')
        if not self.move_to_pose(self.make_pose(cx, cy, cz + 0.12)):
            self.get_logger().error('Approach failed')
            return

        # Step 4 — chiudi gripper
        self.get_logger().info('Step 4: Grasping...')
        self.control_gripper(open=False)
        time.sleep(2.5)

        # Step 5 — solleva alto per superare ostacolo
        self.get_logger().info('Step 5: Lifting...')
        if not self.move_to_pose(self.make_pose(cx, cy, cz + 0.55)):
            self.get_logger().error('Lift failed')
            return

        # Step 6 — trasporta evitando ostacolo
        self.get_logger().info('Step 6: Transporting...')
        if not self.move_to_pose(self.make_pose(tx, ty, tz + 0.55)):
            self.get_logger().error('Transport failed')
            return

        # Step 7 — deposita
        self.get_logger().info('Step 7: Placing...')
        if not self.move_to_pose(self.make_pose(tx, ty, tz + 0.05)):
            self.get_logger().error('Place failed')
            return

        # Step 8 — apri gripper
        self.get_logger().info('Step 8: Releasing...')
        self.control_gripper(open=True)
        time.sleep(0.5)

        # Step 9 — ritira
        self.get_logger().info('Step 9: Retreating...')
        self.move_to_pose(self.make_pose(tx, ty, tz + 0.3))

        self.get_logger().info('Task complete!')


def main(args=None):
    rclpy.init(args=args)
    node = CubePlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
