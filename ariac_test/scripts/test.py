#!/usr/bin/env python3
"""
To test this script, run the following commands in separate terminals:

- ros2 launch ariac_gazebo ariac.launch.py trial_name:=tutorial competitor_pkg:=ariac_test
- ros2 launch ariac_test robot_commander.launch.py
- ros2 run ariac_test test.py
- ./experiment_skip_koord.sh /home/guest/Documents/cym/CyPhyHouseExperiments/experiments_koord/krd_py config.yaml
"""
import math

import ariac_msgs.msg
import geometry_msgs.msg
import rclpy
from ariac_test.interface import Interface
from ariac_test.utils import get_yaw, build_pose, set_robot_orientation, multiply_pose, Frame, multiply_frame

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
agv = 'agv1'
quadrant = 4


def kit_task(interface):
    rate = interface.create_rate(1)
    while interface.part_in_camera_poses is None or len(interface.part_in_camera_poses) == 0:
        rclpy.spin_once(interface)

    interface.set_conveyor_power(0.0)

    # pos = interface.get_transform_pos()
    part = interface.part_in_camera_poses[0].part  # type and color
    pose = multiply_pose([interface.camera_pose, interface.part_in_camera_poses[0].pose])
    pose_yaw = get_yaw(pose)

    b1 = interface.set_joint_value_target('floor', 'linear_actuator_joint', -1 * pose.position.y + 0.5)
    b2 = interface.set_joint_value_target('floor', "floor_shoulder_pan_joint", -3.14159)

    # sync
    while not (b1 and b2):
        pass

    start_pos = build_pose(x=pose.position.x, y=pose.position.y, z=pose.position.z + 0.3,
                           orientation=set_robot_orientation(pose_yaw))

    end_pos = build_pose(x=pose.position.x, y=pose.position.y,
                         z=pose.position.z + _part_heights[part.type] + _pick_offset,
                         orientation=set_robot_orientation(pose_yaw))

    b3 = interface.move_to('floor', [start_pos, end_pos], 'cartesian')
    while not b3:
        pass

    interface.robot_set_gripper_state('floor', True)

    interface.wait_for_attach('floor', part, pose, 3.0)
    rate.sleep()

    interface.move_to('floor', [build_pose(x=pose.position.x, y=pose.position.y, z=pose.position.z + 0.3,
                                           orientation=set_robot_orientation(0))], 'cartesian')
    rate.sleep()

    # Move to agv

    interface.set_joint_value_target('floor', 'linear_actuator_joint', _rail_positions[agv])
    interface.set_joint_value_target('floor', 'floor_shoulder_pan_joint', 0.0)

    agv_tray_pose = interface.static_transform('world', agv + "_tray")
    part_drop_offset = build_pose(_quad_offsets[quadrant][0], _quad_offsets[quadrant][1], 0.0,
                                  geometry_msgs.msg.Quaternion())
    part_drop_pose = multiply_pose([agv_tray_pose, part_drop_offset])

    start_pos = build_pose(part_drop_pose.position.x, part_drop_pose.position.y,
                           part_drop_pose.position.z + 0.3, set_robot_orientation(0))
    end_pos = build_pose(part_drop_pose.position.x, part_drop_pose.position.y,
                         part_drop_pose.position.z + _part_heights[part.type] +
                         _drop_height + 0.01, set_robot_orientation(0))
    b4 = interface.move_to('floor', [start_pos, end_pos], 'cartesian')
    while not b4:
        pass

    # interface.floor_robot_place_part_on_tray("agv1", 4)
    # rate.sleep()
    interface.robot_set_gripper_state('floor', False)
    interface.detach('floor', part)


def main(args=None):
    rclpy.init(args=args)
    interface = Interface()
    rate = interface.create_rate(1)

    interface.start_competition()
    interface.move_robot_home("floor_robot")
    interface.move_robot_home("ceiling_robot")

    kit_task(interface)
    # move agv to station
    interface.lock_agv_tray(1)  # agv_1
    interface.move_agv(agv, 1)  # station1

    # ceiling robot move to station
    for joint, value in _ceiling_as1.items():
        interface.set_joint_value_target('ceiling', joint, float(value))
    # b1 = interface.set_joint_value_target('ceiling', 'gantry_x_axis_joint', 1.0)
    # b2 = interface.set_joint_value_target('ceiling', 'gantry_y_axis_joint', -3.0)
    # b3 = interface.set_joint_value_target('ceiling', 'gantry_rotation_joint', 1.571)
    # b4 = interface.set_joint_value_target('ceiling', 'ceiling_shoulder_lift_joint', -2.37)
    # # sync
    # while not (b1 and b2 and b3 and b4):
    #     pass

    parts = interface.get_agv_sensor_part_pose(1)
    while parts is None or len(parts) == 0:
        rclpy.spin_once(interface)

    for part in parts:
        # pick part
        part_pose = part.pose
        part_rotation = get_yaw(part_pose)
        interface.get_logger().info(f'part: {part.part} at pose: {part_pose}')
        dx, dy = 0, 0
        if part.part.type == ariac_msgs.msg.Part.BATTERY:
            dx = _battery_grip_offset * math.cos(part_rotation)
            dy = _battery_grip_offset * math.sin(part_rotation)
        start_pos = build_pose(part_pose.position.x + dx, part_pose.position.y + dy,
                               part_pose.position.z + 0.4, set_robot_orientation(part_rotation))
        end_pos = build_pose(part_pose.position.x + dx, part_pose.position.y + dy,
                             part_pose.position.z + _part_heights[part.part.type] + _pick_offset,
                             set_robot_orientation(part_rotation))
        interface.move_to('ceiling', [start_pos, end_pos], 'cartesian')

        interface.robot_set_gripper_state('ceiling', True)
        interface.wait_for_attach('ceiling', part.part, part_pose, 3.0)
        rate.sleep()

        robot_pose = interface.get_model_state('ceiling_gripper').pose
        b = interface.move_to('ceiling', [
            build_pose(x=robot_pose.position.x, y=robot_pose.position.y, z=robot_pose.position.z + 0.2,
                       orientation=robot_pose.orientation)], 'cartesian')
        # b = interface.move_to('ceiling', [
        #     build_pose(x=part_pose.position.x, y=part_pose.position.y, z=part_pose.position.z + 0.3,
        #                orientation=set_robot_orientation(0))], 'cartesian')
        while not b:
            rate.sleep()

        for joint, value in _ceiling_as1.items():
            interface.set_joint_value_target('ceiling', joint, float(value))

        #  CeilingRobot Assemble Part
        cnf = assembly_cnf.get(part.part.type)
        insert_frame_name = "as1_insert_frame"

        insert_frame = Frame()
        insert_frame.frame_from_pose(interface.static_transform('world', insert_frame_name))
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
            b = interface.move_to('ceiling', [start_pos.frame_to_pose(), end_pos.frame_to_pose()], 'cartesian')
        else:
            end_pos = multiply_frame([insert_frame, install_frame, part_assemble_frame, part_to_gripper_frame])
            b = interface.move_to('ceiling', [end_pos.frame_to_pose()], 'cartesian')
        while not b:
            rate.sleep()
        install_frame = Frame(p=list(map(lambda x: x * -0.003, install)))
        end_pos = multiply_frame([insert_frame, install_frame, part_assemble_frame, part_to_gripper_frame])
        b = interface.move_to('ceiling', [end_pos.frame_to_pose()], 'cartesian')
        while not b:
            rate.sleep()
        #  WaitForAssemble
        rate.sleep()
        station = 1
        interface.wait_for_assemble('ceiling', station, part.part, list(map(float, install)))
        interface.robot_set_gripper_state('ceiling', False)
        interface.detach('ceiling', part.part)

        for joint, value in _ceiling_as1.items():
            interface.set_joint_value_target('ceiling', joint, float(value))

    interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
