import rclpy
from ariac_msgs.srv import AgvPartPose, MoveAGV
from ariac_test import utils
from ariac_test.utils import Utils
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
import ariac_msgs.msg

_quad_offsets = {
    ariac_msgs.msg.KittingPart.QUADRANT1: (-0.08, 0.12),
    ariac_msgs.msg.KittingPart.QUADRANT2: (0.08, 0.12),
    ariac_msgs.msg.KittingPart.QUADRANT3: (-0.08, -0.12),
    ariac_msgs.msg.KittingPart.QUADRANT4: (0.08, -0.12),
}


class AGV(Node):
    def __init__(self, node_name: str, agv_id):
        super().__init__(node_name)
        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.name = self.get_name()
        self.set_parameters([sim_time])

        self.id = agv_id
        self._agv_sensor_cli = self.create_client(AgvPartPose, f"/ariac/agv{self.id}_sensor")
        self._move_cli = self.create_client(MoveAGV, f'/ariac/move_agv{self.id}')
        self._lock_tray_cli = self.create_client(Trigger, f'/ariac/agv{agv_id}_lock_tray')
        self.first_move = True

        self.utils = Utils()

    def get_part_pose_by_sensor(self):
        request = AgvPartPose.Request()
        request.id = self.id
        future = self._agv_sensor_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)

        if future.result().success:
            # if res.success:
            self.get_logger().info(f'get agv{self.id} parts and pos: {future.result().parts}')
        return future.result().parts

    def move_to(self, destination: int):
        if self.first_move:
            self._lock_agv_tray()
            self.first_move = False
        request = MoveAGV.Request()
        request.location = destination
        future = self._move_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'move agv{self.id} to destination {destination} !')
            return True
        else:
            self.get_logger().warn(future.result().message)
        return False

    def compute_tray_loc(self, quadrant: int):
        agv_tray_pose = self.utils.static_transform('world', f'{self.name}_tray')
        part_drop_offset = utils.build_pose(_quad_offsets[quadrant][0], _quad_offsets[quadrant][1], 0.0, Quaternion())
        part_drop_pose = utils.multiply_pose([agv_tray_pose, part_drop_offset])
        return part_drop_pose

    def _lock_agv_tray(self):
        request = Trigger.Request()
        future = self._lock_tray_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)
        if future.result().success:
            # if res.success:
            self.get_logger().info(f'lock agv{self.id} tray......')
