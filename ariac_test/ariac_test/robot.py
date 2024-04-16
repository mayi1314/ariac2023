from abc import abstractmethod, ABC
from typing import Union, List

import rclpy
from ariac_msgs.msg import Part
from ariac_msgs.srv import Attach, SetJointValueTarget, MoveToPos
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.parameter import Parameter


class Robot(Node, ABC):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.name = self.get_name()
        self.set_parameters([sim_time])

        self._set_joint_value_cli = self.create_client(SetJointValueTarget, "/ariac/set_joint_value_target")
        self._wait_for_attach_cli = self.create_client(Attach, "/ariac/wait_for_attach")
        self._detach_object_cli = self.create_client(Attach, '/ariac/detach_object')
        self._move_cli = self.create_client(MoveToPos, "/ariac/move_to_pos")

    def set_joint_value(self, joint: str, value):
        request = SetJointValueTarget.Request()
        request.robot_name = self.name
        request.joint = joint
        request.value = value
        future = self._set_joint_value_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'set {self.name} joint: {joint} value: {value}!')
            return True
        else:
            self.get_logger().warn(future.result().message)
        return False

    def wait_for_attach(self, part: Part, pose: Pose, timeout):
        """
        attach object
        :param part: the object to attach
        :param pose: the position of object
        :param timeout: timeout
        """
        request = Attach.Request()
        request.robot_name = self.name
        request.part = part
        request.pose = pose
        request.timeout = timeout
        future = self._wait_for_attach_cli.call_async(request)
        self.get_logger().info(f'wait for {self.name} robot attach part!')
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'{self.name} robot has attached part!')
        else:
            self.get_logger().warn(future.result().message)

    def wait_for_detach(self, part: Part):
        request = Attach.Request()
        request.robot_name = self.name
        request.part = part
        future = self._detach_object_cli.call_async(request)
        self.get_logger().info(f'wait for {self.name} robot detach part!')
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'{self.name} robot has detached part!')
        else:
            self.get_logger().warn(future.result().message)

    def move_to(self, waypoints: Union[Pose, List[Pose]]):
        """
        path_type: point, cartesian. if use point, waypoints is a single pose
        """
        request = MoveToPos.Request()
        if type(waypoints) == list:
            path_type = 'cartesian'
            request.waypoints = waypoints
        else:
            path_type = 'point'
            request.pose = waypoints

        request.robot_name = self.name
        request.type = path_type

        future = self._move_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'move {self.name} to {waypoints}!')
            return True
        else:
            self.get_logger().warn(future.result().message)
        return False

    @abstractmethod
    def attach(self, part: Part, part_pose: Pose, after_pose: Pose, timeout=3.0):
        pass

    @abstractmethod
    def detach(self, part: Part):
        pass
