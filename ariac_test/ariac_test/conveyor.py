import rclpy
from ariac_msgs.msg import AdvancedLogicalCameraImage
from rclpy.node import Node
from rclpy.parameter import Parameter
from ariac_msgs.srv import ConveyorBeltControl
from rclpy.qos import qos_profile_sensor_data


class Conveyor(Node):
    def __init__(self, node_name: str, conveyor_id):
        super().__init__(node_name)
        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.name = self.get_name()
        self.set_parameters([sim_time])

        self.id = conveyor_id
        self._part_poses = None
        self._sensor_pose = None
        self._set_power = self.create_client(ConveyorBeltControl, "/ariac/set_conveyor_power")
        self._conveyor_sensor_sub = self.create_subscription(AdvancedLogicalCameraImage,
                                                             "/ariac/sensors/conveyor_bins_camera/image",
                                                             self._conveyor_sensor_cb,
                                                             qos_profile_sensor_data)

    def set_power(self, power: float):
        request = ConveyorBeltControl.Request()
        request.power = power
        if not self._set_power.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('conveyor belt node not running')
            return
        future = self._set_power.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Set conveyor power to {power}')

    def _conveyor_sensor_cb(self, msg: AdvancedLogicalCameraImage):
        # self.get_logger().info("Received data from conveyor_bins_camera")
        self._part_poses = msg.part_poses
        self._sensor_pose = msg.sensor_pose
        if self.part_poses is None or len(self.part_poses) == 0:
            self.get_logger().info('part not found......')

    @property
    def part_poses(self):
        return self._part_poses

    @property
    def sensor_pose(self):
        return self._sensor_pose
