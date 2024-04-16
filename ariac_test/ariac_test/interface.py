import math

from typing import Union, List

import ariac_msgs.srv
import geometry_msgs.msg
import rclpy
import std_srvs.srv
import tf2_ros
from gazebo_msgs.srv import GetEntityState

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from ariac_msgs.msg import (
    CompetitionState as CompetitionStateMsg,
)

from std_srvs.srv import Trigger

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from gazebo_msgs.msg import ModelStates


class Interface(Node):
    _competition_states = {
        CompetitionStateMsg.IDLE: 'idle',
        CompetitionStateMsg.READY: 'ready',
        CompetitionStateMsg.STARTED: 'started',
        CompetitionStateMsg.ORDER_ANNOUNCEMENTS_DONE: 'order_announcements_done',
        CompetitionStateMsg.ENDED: 'ended',
    }

    def __init__(self, node_name='node_interface'):
        super().__init__(node_name)
        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.set_parameters([sim_time])

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._start_competition_client = self.create_client(
            Trigger, '/ariac/start_competition')

        # Store the state of the competition
        self._competition_state: CompetitionStateMsg = None

        # Service client for moving the floor robot to the home position
        self._move_floor_robot_home = self.create_client(
            Trigger, '/competitor/move_floor_robot_home')
        # Service client for moving the ceiling robot to the home position
        self._move_ceiling_robot_home = self.create_client(
            Trigger, '/competitor/move_ceiling_robot_home')

        self._set_conveyor_power = self.create_client(
            ariac_msgs.srv.ConveyorBeltControl, "/ariac/set_conveyor_power")
        self._part_in_camera_poses = None
        self._camera_pose = None
        self._part = None
        self._conveyor_bins_camera_sub = self.create_subscription(ariac_msgs.msg.AdvancedLogicalCameraImage,
                                                                  "/ariac/sensors/conveyor_bins_camera/image",
                                                                  self._conveyor_bins_camera_cb,
                                                                  qos_profile_sensor_data)
        self._floor_robot_gripper_enable = self.create_client(
            ariac_msgs.srv.VacuumGripperControl, "/ariac/floor_robot_enable_gripper")
        self._ceiling_robot_gripper_enable = self.create_client(
            ariac_msgs.srv.VacuumGripperControl, "/ariac/ceiling_robot_enable_gripper")
        self._wait_for_attach = self.create_client(ariac_msgs.srv.Attach, "/ariac/wait_for_attach")

        self._move_to_client = self.create_client(ariac_msgs.srv.MoveToPos, "/ariac/move_to_pos")

        self._set_joint_value_target = self.create_client(ariac_msgs.srv.SetJointValueTarget,
                                                          "/ariac/set_joint_value_target")

        self._agv_sensor = self.create_client(ariac_msgs.srv.AgvPartPose, "/ariac/agv1_sensor")
        # self.camera_timer = self.create_timer(2, self._conveyor_bins_camera_cb)
        # executor = rclpy.executors.MultiThreadedExecutor()
        # executor.add_node(self)
        # Thread(target=lambda: executor.spin()).start()

    def start_competition(self):
        """Function to start the competition.
        """
        self.get_logger().info('Waiting for competition to be ready')

        if self._competition_state == CompetitionStateMsg.STARTED:
            return

        if not self._start_competition_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error('Service \'/ariac/start_competition\' is not available.')
            return
        # Create trigger request and call starter service
        request = Trigger.Request()
        future = self._start_competition_client.call_async(request)
        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info('Started competition.')
        else:
            self.get_logger().warn('Unable to start competition')

    def move_robot_home(self, robot_name):
        """Move one of the robots to its home position.
        Arguments:
            robot_name -- Name of the robot to move home
        """
        request = Trigger.Request()
        if robot_name == 'floor_robot':
            if not self._move_floor_robot_home.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return
            future = self._move_floor_robot_home.call_async(request)
        elif robot_name == 'ceiling_robot':
            if not self._move_ceiling_robot_home.wait_for_service(timeout_sec=1.0):
                self.get_logger().error('Robot commander node not running')
                return
            future = self._move_ceiling_robot_home.call_async(request)
        else:
            self.get_logger().error(f'Robot name: ({robot_name}) is not valid')
            return
        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Moved {robot_name} to home position')
        else:
            self.get_logger().warn(future.result().message)

    def set_conveyor_power(self, power: float):
        request = ariac_msgs.srv.ConveyorBeltControl.Request()
        request.power = power
        if not self._set_conveyor_power.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('conveyor belt node not running')
            return
        future = self._set_conveyor_power.call_async(request)
        # Wait until the service call is completed
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Set conveyor power to {power}')

    def _conveyor_bins_camera_cb(self, msg: ariac_msgs.msg.AdvancedLogicalCameraImage):
        # self.get_logger().info("Received data from conveyor_bins_camera")
        self._part_in_camera_poses = msg.part_poses
        self._camera_pose = msg.sensor_pose
        # if self.part_in_camera_poses is None or len(self.part_in_camera_poses) == 0:
        #     self.get_logger().info("camera: no part found")

    def get_model_state(self, model: str, frame='world'):
        client = self.create_client(GetEntityState, '/gazebo/get_entity_state')
        request = GetEntityState.Request()
        request.name = model
        request.reference_frame = frame
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'get model:{model} state: {future.result().state}')
        return future.result().state

    def get_agv_sensor_part_pose(self, agv_id: int):
        request = ariac_msgs.srv.AgvPartPose.Request()
        request.id = agv_id
        future = self._agv_sensor.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)

        if future.result().success:
            # if res.success:
            self.get_logger().info(f'get agv{agv_id} parts and pos: {future.result().parts}')
        return future.result().parts

    def robot_set_gripper_state(self, robot_name: str, enable: bool):
        request = ariac_msgs.srv.VacuumGripperControl.Request()
        request.enable = enable
        future = None
        if robot_name == 'floor':
            future = self._floor_robot_gripper_enable.call_async(request)
        elif robot_name == 'ceiling':
            future = self._ceiling_robot_gripper_enable.call_async(request)
        self.get_logger().info(f'{robot_name} set gripper {enable}')
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def move_to(self, robot_name: str, pos: Union[geometry_msgs.msg.Pose, List[geometry_msgs.msg.Pose]],
                path_type='point'):
        request = ariac_msgs.srv.MoveToPos.Request()
        request.robot_name = robot_name
        request.type = path_type
        if path_type == 'point':
            request.pose = pos
        else:
            request.waypoints = pos

        future = self._move_to_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'move {robot_name} to {pos}!')
            return True
        else:
            self.get_logger().warn(future.result().message)
        return False

    def set_joint_value_target(self, robot_name: str, joint: str, value):
        request = ariac_msgs.srv.SetJointValueTarget.Request()
        request.robot_name = robot_name
        request.joint = joint
        request.value = value
        future = self._set_joint_value_target.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'set {robot_name} joint: {joint} value: {value}!')
            return True
        else:
            self.get_logger().warn(future.result().message)
        return False

    def wait_for_attach(self, robot_name: str, part: ariac_msgs.msg.Part, pos: geometry_msgs.msg.Pose, timeout=3.0):
        request = ariac_msgs.srv.Attach.Request()
        request.robot_name = robot_name
        request.part = part
        request.pose = pos
        request.timeout = timeout
        future = self._wait_for_attach.call_async(request)
        self.get_logger().info(f'wait for {robot_name} robot attach part!')
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'{robot_name} robot has attached part!')
        else:
            self.get_logger().warn(future.result().message)

    def detach(self, robot_name: str, part: ariac_msgs.msg.Part):
        _detach_object = self.create_client(ariac_msgs.srv.Attach, '/ariac/detach_object')
        request = ariac_msgs.srv.Attach.Request()
        request.robot_name = robot_name
        request.part = part

        future = _detach_object.call_async(request)
        self.get_logger().info(f'wait for {robot_name} robot detach part!')
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'{robot_name} robot has detached part!')
        else:
            self.get_logger().warn(future.result().message)

    def move_agv(self, agv: str, destination: int):
        _move_agv = self.create_client(ariac_msgs.srv.MoveAGV, f'/ariac/move_{agv}')
        request = ariac_msgs.srv.MoveAGV.Request()
        request.location = destination
        future = _move_agv.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'move {agv} to destination {destination} !')
            return True
        else:
            self.get_logger().warn(future.result().message)
        return False

    def lock_agv_tray(self, agv_id: int):
        client = self.create_client(std_srvs.srv.Trigger, f'/ariac/agv{agv_id}_lock_tray')
        request = std_srvs.srv.Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)

        if future.result().success:
            # if res.success:
            self.get_logger().info(f'lock agv{agv_id} tray......')

    def wait_for_assemble(self, robot_name: str, station: int, part: ariac_msgs.msg.Part, install: List[float]):
        client = self.create_client(ariac_msgs.srv.Assemble, '/ariac/wait_for_assemble')
        request = ariac_msgs.srv.Assemble.Request()
        request.robot_name = robot_name
        request.station = station
        request.part = part
        request.install_direction = install
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5)

        if future.result().success:
            # if res.success:
            self.get_logger().info(f'wait for assemble on station{station}, part: {part.color}_{part.type}')

    @property
    def part_in_camera_poses(self):
        return self._part_in_camera_poses

    @property
    def camera_pose(self):
        return self._camera_pose

    def static_transform(self, to_frame_rel, from_frame_rel):
        from rclpy import time
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

        pose = geometry_msgs.msg.Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation = t.transform.rotation
        return pose

    # def get_part_pose_from_camera(self):
    #     camera_pos = tf2_kdl.from_msg_vector(self._camera_pose)
    #     part_pos = tf2_kdl.from_msg_vector(self._part_in_camera_poses)
    #     pos = camera_pos * part_pos
    #     return tf2_kdl.to_msg_vector(pos)
