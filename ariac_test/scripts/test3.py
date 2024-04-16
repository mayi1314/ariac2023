#!/usr/bin/env python3

import math

import ariac_msgs.msg

import rclpy
from ariac_test.agv import AGV
from ariac_test.ceiling_robot import CeilingRobot
from ariac_test.conveyor import Conveyor
from ariac_test.floor_robot import FloorRobot
from ariac_test.utils import get_yaw, build_pose, set_robot_orientation, multiply_pose, Frame, multiply_frame, Utils
from geometry_msgs.msg import Quaternion

_pick_offset = 0.003
_drop_height = 0.002
_battery_grip_offset = -0.05
PI = math.pi
_part_heights = {
    ariac_msgs.msg.Part.BATTERY: 0.04,
    ariac_msgs.msg.Part.PUMP: 0.12,
    ariac_msgs.msg.Part.REGULATOR: 0.07,
    ariac_msgs.msg.Part.SENSOR: 0.07
}
_rail_positions = {
    "agv1": -4.5,
    "agv2": -1.2,
    "agv3": 1.2,
    "agv4": 4.5,
    "left_bins": 3,
    "right_bins": -3
}
_quad_offsets = {
    ariac_msgs.msg.KittingPart.QUADRANT1: (-0.08, 0.12),
    ariac_msgs.msg.KittingPart.QUADRANT2: (0.08, 0.12),
    ariac_msgs.msg.KittingPart.QUADRANT3: (-0.08, -0.12),
    ariac_msgs.msg.KittingPart.QUADRANT4: (0.08, -0.12),
}
_ceiling_as1 = {
    "gantry_x_axis_joint": 1,
    "gantry_y_axis_joint": -3,
    "gantry_rotation_joint": 1.571,
    "ceiling_shoulder_pan_joint": 0,
    "ceiling_shoulder_lift_joint": -2.37,
    # "ceiling_elbow_joint": 2.37,
    # "ceiling_wrist_1_joint": 3.14,
    # "ceiling_wrist_2_joint": -1.57,
    # "ceiling_wrist_3_joint": 0
}

assembly_cnf = {
    ariac_msgs.msg.Part.REGULATOR: {'assembled_pose': [0.175, -0.223, 0.215, PI / 2, 0, -PI / 2],
                                    'assembly_direction': [0, 0, -1]},
    ariac_msgs.msg.Part.BATTERY: {'assembled_pose': [-0.15, 0.035, 0.043, 0, 0, PI / 2],
                                  'assembly_direction': [0, 1, 0]},
    ariac_msgs.msg.Part.PUMP: {'assembled_pose': [0.14, 0.0, 0.02, 0, 0, -PI / 2],
                               'assembly_direction': [0, 0, -1]},
    ariac_msgs.msg.Part.SENSOR: {'assembled_pose': [-0.1, 0.395, 0.045, 0, 0, -PI / 2],
                                 'assembly_direction': [0, -1, 0]},
}
agv_id = 1
quadrant = 4
station = 1


def main(args=None):
    rclpy.init(args=args)
    floor_robot = FloorRobot('floor_robot')
    ceiling_robot = CeilingRobot('ceiling_robot')
    agv_1 = AGV('agv1', agv_id=agv_id)
    conveyor = Conveyor('conveyor', 1)
    utils = Utils('utils')

    utils.start_competition()
    floor_robot.move_to_home()
    ceiling_robot.move_to_home()

    # ------ kitting task --------- #
    while conveyor.part_poses is None or len(conveyor.part_poses) == 0:
        rclpy.spin_once(conveyor)
    # detected part, then stop conveyor
    conveyor.set_power(0.0)

    part = conveyor.part_poses[0].part
    part_pose = multiply_pose([conveyor.sensor_pose, conveyor.part_poses[0].pose])
    pose_yaw = get_yaw(part_pose)
    # pre-action
    floor_robot.set_joint_value('linear_actuator_joint', -1 * part_pose.position.y + 0.5)
    floor_robot.set_joint_value('floor_shoulder_pan_joint', -3.14159)

    start_pos = build_pose(x=part_pose.position.x, y=part_pose.position.y, z=part_pose.position.z + 0.3,
                           orientation=set_robot_orientation(pose_yaw))
    end_pos = build_pose(x=part_pose.position.x, y=part_pose.position.y,
                         z=part_pose.position.z + _part_heights[part.type] + _pick_offset,
                         orientation=set_robot_orientation(pose_yaw))
    floor_robot.move_to([start_pos, end_pos])
    # grip part
    after_pose = build_pose(x=part_pose.position.x, y=part_pose.position.y, z=part_pose.position.z + 0.3,
                            orientation=set_robot_orientation(0))
    floor_robot.attach(part, part_pose, after_pose)

    # Move to agv
    # pre-action
    floor_robot.set_joint_value('linear_actuator_joint', _rail_positions[f'agv{agv_id}'])
    floor_robot.set_joint_value('floor_shoulder_pan_joint', 0.0)
    # compute part drop position on agv
    agv_tray_pose = utils.static_transform('world', f'agv{agv_id}_tray')
    part_drop_offset = build_pose(_quad_offsets[quadrant][0], _quad_offsets[quadrant][1], 0.0, Quaternion())
    part_drop_pose = multiply_pose([agv_tray_pose, part_drop_offset])

    start_pos = build_pose(part_drop_pose.position.x, part_drop_pose.position.y,
                           part_drop_pose.position.z + 0.3, set_robot_orientation(0))
    end_pos = build_pose(part_drop_pose.position.x, part_drop_pose.position.y,
                         part_drop_pose.position.z + _part_heights[part.type] +
                         _drop_height + 0.01, set_robot_orientation(0))
    floor_robot.move_to([start_pos, end_pos])
    # detach part
    floor_robot.detach(part)

    # ------ assembly task --------- #
    # move agv to station 1
    agv_1.move_to(station)
    # ceiling_robot pre-action
    for joint, value in _ceiling_as1.items():
        ceiling_robot.set_joint_value(joint, float(value))
    # get parts position and state by agv_sensor
    parts = agv_1.get_part_pose_by_sensor()
    while parts is None or len(parts) == 0:
        rclpy.spin_once(agv_1)

    for part in parts:
        # pick part
        part_pose = part.pose
        part_rotation = get_yaw(part_pose)
        agv_1.get_logger().info(f'part: {part.part} at pose: {part_pose}')
        dx, dy = 0, 0
        if part.part.type == ariac_msgs.msg.Part.BATTERY:
            dx = _battery_grip_offset * math.cos(part_rotation)
            dy = _battery_grip_offset * math.sin(part_rotation)
        start_pos = build_pose(part_pose.position.x + dx, part_pose.position.y + dy,
                               part_pose.position.z + 0.4, set_robot_orientation(part_rotation))
        end_pos = build_pose(part_pose.position.x + dx, part_pose.position.y + dy,
                             part_pose.position.z + _part_heights[part.part.type] + _pick_offset,
                             set_robot_orientation(part_rotation))
        ceiling_robot.move_to([start_pos, end_pos])

        after_pose = build_pose(x=part_pose.position.x, y=part_pose.position.y, z=part_pose.position.z + 0.2,
                                orientation=set_robot_orientation(part_rotation))
        ceiling_robot.attach(part.part, part_pose, after_pose, 3.0)
        # pre-action
        for joint, value in _ceiling_as1.items():
            ceiling_robot.set_joint_value(joint, float(value))

        #  CeilingRobot Assemble Part
        cnf = assembly_cnf.get(part.part.type)
        insert_frame_name = "as1_insert_frame"

        insert_frame = Frame()
        insert_frame.frame_from_pose(utils.static_transform('world', insert_frame_name))
        vec = cnf.get('assembled_pose')
        part_assemble_frame = Frame()
        part_assemble_frame.frame_from_pose(
            build_pose(vec[0], vec[1], vec[2], set_robot_orientation(vec[5], vec[3], vec[4])))  # y, r, p
        install = cnf.get('assembly_direction')
        part_to_gripper_frame = Frame()
        part_to_gripper_frame.frame_from_pose(
            build_pose(0., 0., _part_heights[part.part.type], set_robot_orientation(PI, PI, 0)))
        up = Frame(p=[0, 0, 0.1])
        install_frame = Frame(p=list(map(lambda x: x * -0.1, install)))
        if part.part.type == ariac_msgs.msg.Part.BATTERY:
            install_frame = Frame(p=list(map(lambda x: x * -0.06, install)))
            part_to_gripper_frame.frame_from_pose(
                build_pose(_battery_grip_offset, 0., _part_heights[part.part.type], set_robot_orientation(PI, PI, 0)))
            start_pos = multiply_frame([insert_frame, up, install_frame, part_assemble_frame, part_to_gripper_frame])
            end_pos = multiply_frame([insert_frame, install_frame, part_assemble_frame, part_to_gripper_frame])
            ceiling_robot.move_to([start_pos.frame_to_pose(), end_pos.frame_to_pose()])
        else:
            end_pos = multiply_frame([insert_frame, install_frame, part_assemble_frame, part_to_gripper_frame])
            ceiling_robot.move_to([end_pos.frame_to_pose()])

        install_frame = Frame(p=list(map(lambda x: x * -0.003, install)))
        end_pos = multiply_frame([insert_frame, install_frame, part_assemble_frame, part_to_gripper_frame])
        ceiling_robot.move_to([end_pos.frame_to_pose()])

        #  WaitForAssemble
        ceiling_robot.wait_for_assemble(station, part.part, list(map(float, install)))
        # detach part
        ceiling_robot.detach(part.part)

        for joint, value in _ceiling_as1.items():
            ceiling_robot.set_joint_value(joint, float(value))

    floor_robot.destroy_node()
    ceiling_robot.destroy_node()
    agv_1.destroy_node()
    conveyor.destroy_node()
    utils.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
