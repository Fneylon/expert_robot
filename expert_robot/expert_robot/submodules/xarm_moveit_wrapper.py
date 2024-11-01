"""

Create the wrapper for the MoveIt package.

# Subscribers:
+ joint_states (sensor_msgs/msg/JointState) -- subscribes to the topic
                                to get current joint states of the robots

# Clients:
+ GetPositionIK ( moveit_msgs/srv/) -- client to the service that calculates
                                        the IK for the robot

"""

import numpy as np
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import CollisionObject
from geometry_msgs.msg import Pose, Point
from sensor_msgs.msg import JointState
from moveit_msgs.srv import (
    GetPositionIK,
    GetPlanningScene,
    ApplyPlanningScene,
    GetCartesianPath,
)
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from .populate_msg import PopulateMsgs
from enum import Enum, auto
from moveit_msgs.msg import ObjectColor
from std_msgs.msg import ColorRGBA
import pymesh as pm
import time as t
# from franka_msgs.action import Grasp, Move

from scipy.spatial.transform import Rotation as R


class XARM(Enum):
    """

    Current state of the system.

    Determines what the main timer function should be doing on each iteration.

    """
    DONE = auto()
    INITALIZE = auto()
    WAITING = auto()
    PLANNING = auto()
    EXECUTING = auto()


class XarmWrapper:
    """
    This class is a wrapper for the MoveIt! planning scene and motion planning.

    It is used to plan and execute trajectories
    for the xArm7 manipulator.

    """
    def __init__(self, node: Node):
        # Inherits the node that will command the timing and planing of the MoveGroup
        self.node = node

        # Initialize Clients
        self.initialize_clients()

        # Initialize the robot type: 
        self.robot_type = 'xarm7'

        # Initialize the Populate Msgs Class
        self.pop_msgs = PopulateMsgs(node = self.node, group_name = "xarm7") # TODO: Finalize the MoveGroup Name

        # Initialize the move action: 
        self.action_node = ActionClient(self.node, MoveGroup, "move_action")

        # Initialize the Joint Names: 
        # self.joint_names = [
        #     "joint1",
        #     "joint2",
        #     "joint3",
        #     "joint4",
        #     "joint5",
        #     "joint6",
        #     "joint7",
        #     "drive_joint",
        #     "left_finger_joint",
        #     'left_inner_knuckle_joint', 
        #     'right_inner_knuckle_joint', 
        #     'right_outer_knuckle_joint', 
        #     'right_finger_joint'
        # ]

        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7"
            # "drive_joint",
            # "left_finger_joint",
            # 'left_inner_knuckle_joint', 
            # 'right_inner_knuckle_joint', 
            # 'right_outer_knuckle_joint', 
            # 'right_finger_joint'
        ]

        # Initialize Gripper Joint Names: 
        # self.ee_joint_names = [
        #     "xarm_gripper_base_link",
        #     "link_tcp", 
        #     "right_finger",
        #     "right_inner_knuckle",
        #     "right_outer_knuckle",
        #     "left_finger",
        #     "left_inner_knuckle",
        #     "left_outer_knuckle"
        # ]

        self.ee_joint_names = [
            "drive_joint"
        ]

        # Initialize Subscriptions: 
        self.initialize_subscriptions()

        # Initialize the Reference Frame: 
        self.frame_id = "link_base"

        # Initialize State Machine: 
        self.state = XARM.INITALIZE

        # Initialize the Execute Trajectory ActionClient: 
        self.execute_action = ActionClient(
            self.node, ExecuteTrajectory, "execute_trajectory"
        )


    def cb_joint_state(self, msg: JointState):
        """
        Call back function for joint states subscriber.

        Args:
            msg (JointState): message type from joint states subscriber.

        Returns
        -------
            joint_states (list): list of joint states.

        """
        self.joint_states = list(msg.position)
        return self.joint_states

    def future_pos_orien_callback(self, future_po):
        """Show whether the goal is rejected or accepted."""
        new_robot_state = future_po.result().solution
        print(f"new_robot_state.joint_state: {new_robot_state.joint_state}")
        new_joint_states = new_robot_state.joint_state
        joint_angles = np.array(new_joint_states.position) # NOTE: original
        # print(f"joint_angles: {joint_angles} ")

        # for x/y/z
        joint_angles = np.array([0.00174533, 0.280998, -0.00174533, 0.572468, 0.00174533, -1.279326, 0.00349066])
        # joint_angles = np.array([-0.5466344952583313 , 0.5771662592887878 , 0.0140452207997441, 
        #                           1.868735671043396, -1.1835880279541016, -0.5742381811141968, 1.1125375032424927])

        # for orientation
        # joint_angles = np.array([-0.0001253, 0.114768, -0.0032516, 1.047079, -0.00107456, 0.9244891, 0.00390636])

        goal_constraints = self.pop_msgs.set_Constraints(self.joint_names, joint_angles)

        motion_plan = self.pop_msgs.set_MotionPlanMsgs(
            self.frame_id, self.joint_names, self.joint_states, list(joint_angles)
        )

        # TODO: FIX!!!! 
        motion_plan.request.goal_constraints = goal_constraints
        future_po = self.action_node.send_goal_async(motion_plan)

        future_po.add_done_callback(self.goal_path_cb)

    def plan_to_joint_states(self, end_joint_states):

        goal_constraints = self.pop_msgs.set_Constraints(self.joint_names, end_joint_states)

        motion_plan = self.pop_msgs.set_MotionPlanMsgs(
            self.frame_id, self.joint_names, self.joint_states, list(end_joint_states)
        )

        # TODO: FIX!!!! 
        motion_plan.request.goal_constraints = goal_constraints
        future_po = self.action_node.send_goal_async(motion_plan)

        future_po.add_done_callback(self.goal_path_cb)


    def future_position_callback(self, future):
        """Show whether the goal is rejected or accepted."""
        new_robot_state = future.result().solution

        new_joint_states = new_robot_state.joint_state
        joint_angles = np.array(new_joint_states.position)

        goal_constraints = self.pop_msgs.set_Constraints(self.joint_names, joint_angles)

        motion_plan = self.pop_msgs.set_MotionPlanMsgs(
            self.frame_id, self.joint_names, new_joint_states.position
        )

        motion_plan.request.goal_constraints = goal_constraints
        future = self.action_node.send_goal_async(motion_plan)

        future.add_done_callback(self.goal_path_cb)

    def future_cartesian_cb(self, future):
        """Execute RobotState msg and Robot Trajectory msg."""
        robot_traj = future.result().solution
        frac = future.result().fraction
        print("fraction", frac)

        traj_motion = ExecuteTrajectory.Goal()
        traj_motion.trajectory = robot_traj

        if frac is not None or frac != 1.0:
            print(f"Fraction: {frac}")
        if traj_motion:
            self.node.get_logger().info("\n\tNOTE: Path found in MoveitWrapper")
            future = self.execute_action.send_goal_async(traj_motion)
            future.add_done_callback(self.future_execute_callback)
            self.state = XARM.EXECUTING

        else:
            self.node.get_logger().info("Path not found in MoveitWrapper")
            self.state = XARM.WAITING

    def goal_path_cb(self, future):
        """
        Call back function for the goal position path.

        Args:
            motion_plan (MotionPlan): message type from goal position path.

        Returns
        -------
            None

        """
        result = future.result()
        _position_result_future = result.get_result_async()
        _position_result_future.add_done_callback(self.get_result_cb)


    def get_result_cb(self, motion_plan_future):
        """
        Get path to the desired position and then we can execute it.

        And IK calculation has finished being computed.

        """
        # self.node.get_logger().info("In get result cb")
        self.path = motion_plan_future.result().result.planned_trajectory

        # print(f"self.path {self.path}")

        if self.path:
            self.robot_msg = ExecuteTrajectory.Goal()
            self.robot_msg.trajectory = self.path

            future = self.execute_action.send_goal_async(self.robot_msg)
            future.add_done_callback(self.future_execute_callback)
            self.state = XARM.EXECUTING

        else:
            self.node.get_logger().info("Path not found in Xarm Wrapper")
            self.state = XARM.WAITING

    def future_execute_callback(self, future):
        """Send the result for execution."""
        result = future.result()
        future2 = result.get_result_async()
        future2.add_done_callback(self.get_execute_result_cb)

    def get_execute_result_cb(self, future):
        """Change the state to done."""
        self.state = XARM.DONE

    def plan_path_cartesian(self, waypoints):
        """Cartesian path by specifying a list of waypoints for the end-effector to go through."""
        frame_id = "base_link"
        start_state = [0.30724, 0.00054, 0.59104]

        cartesian_msgs_request = self.pop_msgs.set_GetCartesianPositionRqt(
            self.robot_type, frame_id, start_state, self.ee_joint_names, waypoints
        )
        self.cartesian_future = self.cartesian_srv_client.call_async(
            cartesian_msgs_request
        )

        self.cartesian_future.add_done_callback(self.future_cartesian_cb)

    def plan_path_to_position(self, positions):
        """
        Plan the path to the desired position.

        Orientation of the ee is unspecified.

        Args:
            positions (list): list of x, y, z coordinates.

        Returns
        -------
            ik_calculations_future (future): future object of
                                            the IK calculations.

        """
        self.node.get_logger().info("In Plan Path to Position")
        x, y, z = positions
        point = Point(x=x, y=y, z=z)

        pose_stamped = self.pop_msgs.set_PoseStamp_position(point)

        robot_state = self.pop_msgs.set_RobotState(
            frame_id=self.frame_id,
            joint_names=self.joint_names,
            position=self.joint_states,
        )
        avoid_collisions = True
        timeout = self.pop_msgs.set_Duration(time=5)

        IK_request = self.pop_msgs.set_IKRequest(
            robot_state, avoid_collisions, pose_stamped, timeout
        )

        self.ik_calculations_position_future = self.ik_client.call_async(IK_request)

        self.ik_calculations_position_future.add_done_callback(
            self.future_position_callback
        )

    def plan_path_to_orientation(self, orientation):
        """
        Plan the path to the desired orientation.

        Position of the ee is unspecified.

        Args:
            orientation (list): list of x, y, z, w coordinates.

        Returns
        -------
            ik_calculations_future (future): future object of
                                            the IK calculations.

        """
        pose_stamped = self.pop_msgs.set_PoseStamp_orientation(orientation)

        robot_state = self.pop_msgs.set_RobotState(
            frame_id=self.frame_id,
            joint_names=self.joint_names,
            position=self.joint_states,
        )
        avoid_collisions = True
        timeout = self.pop_msgs.set_Duration(time=5)

        IK_request = self.pop_msgs.set_IKRequest(
            robot_state, avoid_collisions, pose_stamped, timeout
        )

        ik_calculations_future = self.ik_client.call_async(IK_request)

        return ik_calculations_future
    
    def plan_path_to_position_orientation(self, positions, orientation):
        """
        Plan the path to the desired position and orientation.

        Args:
            positions (list): list of x, y, z coordinates.
            orientation (list): list of x, y, z, w coordinates.

        Returns
        -------
            ik_calculations_future (future): future object of
                                            the IK calculations.

        """
        x, y, z = positions
        print("hello x, y, z in plan path", x, y, z)
        point = Point(x=x, y=y, z=z)
        pose_stamped = self.pop_msgs.set_PoseStamp(point, orientation)

        robot_state = self.pop_msgs.set_RobotState(
            frame_id=self.frame_id,
            joint_names=self.joint_names,
            position=self.joint_states, 
        )
        avoid_collisions = True
        timeout = self.pop_msgs.set_Duration(time=5)

        IK_request = self.pop_msgs.set_IKRequest(
            robot_state, avoid_collisions, pose_stamped, timeout
        )

        ik_calculations_future = self.ik_client.call_async(IK_request)

        print("Done Calling Planning async")

        ik_calculations_future.add_done_callback(self.future_pos_orien_callback)


    def add_collision_objects(self, id_list, positions, primitive_properties):
        """
        Add a box to the planning scene.

        Args:
            position (list): list of x, y, z coordinates.
            side_length (float): length of the box.

        Returns
        -------
            None

        """

        self.collision_objects = []

        for (id, position, property) in zip(id_list, positions, primitive_properties):

            print(f'\n\t (!) id: {id}')

            if id[:3] == "box":
                # BOX CASE
                self.box_collision = CollisionObject()
                self.box_collision.header.frame_id = "link_base"
                self.box_collision.header.stamp = self.node.get_clock().now().to_msg()
                self.box_collision.id = id #"box"

                self.box = SolidPrimitive()
                self.box.type = property['type']

                self.box.dimensions = property['side_length']
                self.box_collision.primitives.append(self.box)

                self.box_pose = Pose()
                self.box_pose.position.x = position[0]
                self.box_pose.position.y = position[1]
                self.box_pose.position.z = position[2]

                self.box_pose.orientation.x = 0.0
                self.box_pose.orientation.y = 0.0
                self.box_pose.orientation.z = 0.0
                self.box_pose.orientation.w = 1.0

                self.box_collision.pose = self.box_pose
                self.box_collision.operation = CollisionObject.ADD

                self.collision_objects.append(self.box_collision)

            elif id[:4] == 'mesh':
                #MESHES
                self.mesh_collision = CollisionObject()
                self.mesh_collision.header.frame_id = "link_base"
                self.mesh_collision.header.stamp = self.node.get_clock().now().to_msg()
                self.mesh_collision.id = id


                path_load = "/home/r01_ros2_ws/src/map_comparison/pfields/pfields/mesh/"
                fname = property['filename']
                fpath = path_load + fname + '.obj'

                print(f'\n fpath: {fpath}')

                mesh = pm.meshio.load_mesh(fpath)
                vtxs = mesh.vertices.tolist()
                faces = mesh.faces.tolist()

                print(f'\n NOTE: mesh loaded!')

                ## reorient and translate the loaded cage mesh to match the experimental setup
                rot_val = 1*np.radians(108)
                r = R.from_euler('xyz', [0, 0, rot_val]) #  deg rotation

                mesh_scale = property['scale']
                # 110 cm
                largest_z = 0
                smallest_z = 999
                vtx_list = []
                for i, vtx in enumerate(vtxs):
                    if id.split('_')[1][0:4] == 'bbox':
                        vtx_rot = vtx
                    else:
                        vtx_rot = r.apply(vtx)
                    if vtx_rot[2] > largest_z:
                        largest_z = vtx_rot[2]
                    if vtx_rot[2] < smallest_z:
                        smallest_z = vtx_rot[2]
                    curr_pt = Point(x=vtx_rot[0]*mesh_scale, y=vtx_rot[1]*mesh_scale, z=vtx_rot[2]*mesh_scale)
                    vtx_list.append(curr_pt)

                face_list = []
                for j, face_vtxs in enumerate(faces):
                    triangle_idxs = MeshTriangle(vertex_indices=face_vtxs)
                    face_list.append(triangle_idxs)

                processed_mesh = Mesh(triangles=face_list, vertices=vtx_list)

                # print(f'processed_mesh: {processed_mesh}')

                self.mesh_collision.meshes.append(processed_mesh)

                self.mesh_pose = Pose()
                self.mesh_pose.position.x = position[0]
                self.mesh_pose.position.y = position[1]
                self.mesh_pose.position.z = position[2]

                self.mesh_pose.orientation.x = 0.0
                self.mesh_pose.orientation.y = 0.0
                self.mesh_pose.orientation.z = 0.0
                self.mesh_pose.orientation.w = 1.0

                self.mesh_collision.pose = self.mesh_pose
                self.mesh_collision.operation = CollisionObject.ADD

                self.collision_objects.append(self.mesh_collision)

        print(f'length of collision_objects: {len(self.collision_objects)}')

        future = self.scene_planner.call_async(GetPlanningScene.Request())
        future.add_done_callback(self.future_scene_cb)

        
    def future_scene_cb(self, future):
        """Send the result for execution."""
        self.scene = future.result().scene
        print(f'Scene Type: {type(self.scene)}')

        self.scene.world.collision_objects = self.collision_objects #.append(self.box_collision)
        self.scene.is_diff = True


        for col_obj in self.collision_objects:
            print(f'\n\ncurrent object: {col_obj.id}\n\n')
            box_color = ColorRGBA()
            object_colors = ObjectColor()
            object_colors.color.r = 1.0
            object_colors.color.g = 0.0
            object_colors.color.b = 0.0
            object_colors.color.a = 0.6 #0.6
            object_colors.id = col_obj.id #"box"

            self.scene.object_colors.append(object_colors)


        future = self.apply_planning_scene.call_async(
            ApplyPlanningScene.Request(scene=self.scene)
        )
        future.add_done_callback(self.future_apply_scene_cb)

    def future_apply_scene_cb(self, future):
        """Send the result for execution."""
        self.state = XARM.DONE

    def initialize_subscriptions(self):
        self.joint_states = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_sub = self.node.create_subscription(
                JointState, "/xarm/joint_states", self.cb_joint_state, 10
            )


    def initialize_clients(self):

        # Initialize the Scene Planner
        self.scene_planner = self.node.create_client(
            GetPlanningScene, "get_planning_scene"
        )
        while not self.scene_planner.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info(
                "get planning scene service not available, waiting again..."
            )

        # Intialize the Planning Scene
        self.apply_planning_scene = self.node.create_client(
            ApplyPlanningScene, "apply_planning_scene"
        )
        while not self.apply_planning_scene.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info(
                "apply planning scence service not available, waiting again..."
            )

        # initialize the Cartesian Path Client
        self.cartesian_srv_client = self.node.create_client(
            GetCartesianPath, "compute_cartesian_path"
        )

        # Initialize the position IKClient: 
        self.ik_client = self.node.create_client(GetPositionIK, "compute_ik")

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                "Timeout waiting for 'compute_ik' service to become available."
            )
