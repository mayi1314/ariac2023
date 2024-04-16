#!/usr/bin/env python3

from queue import Queue

import rclpy
from ariac_msgs.msg import Part, BinParts, AGVStatus, AdvancedLogicalCameraImage
from ariac_msgs.srv import AgvPartPose, PoseStatus

from geometry_msgs.msg import Pose
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter
import ariac_msgs.msg
from rclpy.qos import qos_profile_sensor_data

_quad_offsets = {
    ariac_msgs.msg.KittingPart.QUADRANT1: (-0.08, 0.12),
    ariac_msgs.msg.KittingPart.QUADRANT2: (0.08, 0.12),
    ariac_msgs.msg.KittingPart.QUADRANT3: (-0.08, -0.12),
    ariac_msgs.msg.KittingPart.QUADRANT4: (0.08, -0.12),
}

_as_pose = {
    1: Pose()
}

_agv_home_pose = {
    'agv1': [-2.27, 4.80, 0.01],
    'agv2': [-2.27, 1.20, 0.01],
    'agv3': [-2.27, -1.20, 0.01],
    'agv4': [-2.27, -4.80, 0.01]
}


class Monitor(Node):
    def __init__(self, node_name: str):
        # Thread.__init__(self)
        super().__init__(node_name)
        sim_time = Parameter(
            "use_sim_time",
            rclpy.Parameter.Type.BOOL,
            True
        )
        self.name = self.get_name()
        self.set_parameters([sim_time])

        cb_callback = ReentrantCallbackGroup()
        sc_callback = MutuallyExclusiveCallbackGroup()
        # agv
        self._agv_sensor_cli = None
        self._agv_status_sub = self.create_subscription(AGVStatus, f'/ariac/{self.name}_status', self._agv_status, 1,
                                                        callback_group=cb_callback)
        self._agv_reached = 99
        # part_bin
        # self._bin_parts_sub = self.create_subscription(BinParts, '/ariac/bin_parts', self._bin_parts, 1,
        #                                                callback_group=cb_callback)
        self.right_parts, self.right_camera = None, None
        self.left_parts, self.left_camera = None, None
        self._right_bin_camera_sub = self.create_subscription(AdvancedLogicalCameraImage,
                                                              "/ariac/sensors/right_bins_camera/image",
                                                              self._right_bin_camera_cb, qos_profile_sensor_data)
        self._left_bin_camera_sub = self.create_subscription(AdvancedLogicalCameraImage,
                                                             "/ariac/sensors/left_bins_camera/image",
                                                             self._left_bin_camera_cb, qos_profile_sensor_data)

        self._status_srv = self.create_service(PoseStatus, '/ariac/get_pose_status', self._pose_status_callback)

        self.orders = Queue()

    def _pose_status_callback(self, request: PoseStatus.Request, response: PoseStatus.Response):
        part, agv = request.part, request.agv_id
        response.agv_pose = self._agv_reached
        if request.robot == 'ceiling_robot':
            part_pose = self.get_part_pose_by_sensor(agv)
            if part_pose:
                for tmp in part_pose:
                    if part.color == tmp.part.color and part.type == tmp.part.type:
                        response.part_on = f'agv{agv}'
                        response.part_pose = tmp.pose
                        return response
        rs = self.check_part_on_bins(part)
        if rs:
            response.part_on = rs
        return response

    def _right_bin_camera_cb(self, msg: AdvancedLogicalCameraImage):
        self.right_parts = msg.part_poses
        self.right_camera = msg.sensor_pose

    def _left_bin_camera_cb(self, msg: AdvancedLogicalCameraImage):
        self.left_parts = msg.part_poses
        self.left_camera = msg.sensor_pose

    def _agv_status(self, msg: AGVStatus):
        self._agv_reached = msg.location

    def _bin_parts(self, msg):
        pass

    def get_part_pose_by_sensor(self, agv_id: int):
        self._agv_sensor_cli = self.create_client(AgvPartPose, f"/ariac/agv{agv_id}_sensor")
        request = AgvPartPose.Request()
        request.id = agv_id
        future = self._agv_sensor_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3)

        if future.done():
            # if res.success:
            self.get_logger().info(f'get agv{agv_id} parts and pos: {future.result().parts}')
            pass

        return future.result().parts

    def check_part_on_bins(self, part: Part):
        if self.left_parts:
            for part_pose in self.left_parts:
                _part, _pose = part_pose.part, part_pose.pose
                if part.type == _part.type and part.color == _part.color:
                    return 'left_bin'
        if self.right_parts:
            for part_pose in self.right_parts:
                _part, _pose = part_pose.part, part_pose.pose
                if part.type == _part.type and part.color == _part.color:
                    return 'right_bin'
        self.get_logger().warn(f'{len(self.left_parts)}part bin has no part: {part}........')
        return


def main(args=None):
    rclpy.init(args=args)
    node = Monitor('monitor')
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()