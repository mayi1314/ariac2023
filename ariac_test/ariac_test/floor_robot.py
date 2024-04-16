import rclpy
from ariac_msgs.msg import Part
from ariac_msgs.srv import VacuumGripperControl
from ariac_test.robot import Robot
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger


class FloorRobot(Robot):
    def __init__(self, node_name: str):
        super().__init__(node_name)

        self._floor_robot_gripper_enable = self.create_client(VacuumGripperControl, "/ariac/floor_robot_enable_gripper")
        self._move_to_home_cli = self.create_client(Trigger, '/competitor/move_floor_robot_home')

    def move_to_home(self):
        request = Trigger.Request()
        future = self._move_to_home_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Moved {self.name} to home position')
        else:
            self.get_logger().warn(future.result().message)

    def _set_gripper_state(self, enable: bool):
        request = VacuumGripperControl.Request()
        request.enable = enable
        future = self._floor_robot_gripper_enable.call_async(request)
        self.get_logger().info(f'floor robot set gripper {enable}')
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def attach(self, part: Part, part_pose: Pose, after_pose: Pose, timeout=3.0):
        """
        grip function
        """
        self._set_gripper_state(True)
        self.wait_for_attach(part, part_pose, timeout)
        self.move_to([after_pose])

    def detach(self, part: Part):
        self._set_gripper_state(False)
        self.wait_for_detach(part)
