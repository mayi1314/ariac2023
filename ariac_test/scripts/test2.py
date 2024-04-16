from src.harness.agentThread import AgentThread
from geometry_msgs.msg import Pose


class Floor_robot(AgentThread):

    def __init__(self, config, motion_config):
        super(Floor_robot, self).__init__(config, motion_config)

    def initialize_vars(self):
        self.locals = {}
        self.locals['state'] = "IDLE"
        self.locals['state1'] = "IDLE"
        self.create_aw_var('process', str, 'main')
        self.create_aw_var('exit', bool, False)
        self.locals['process'] = 'main'
        self.locals['part_count'] = 0
        self.create_aw_var('all_loaded', bool, None)
        self.create_aw_var('part_locs_on_tray', list, None)
        self.create_aw_var('update_agv_tray', bool, None)
        self.create_aw_var('part_pose', Pose, None)
        self.create_aw_var('target_pose', Pose, None)
        self.create_aw_var('part', Pose, None)
        self.create_aw_var('agv_reached', bool, None)
        self.create_aw_var('part_to_pick_loc', Pose, None)
        self.create_aw_var('to_station_loc', int, None)
        self.create_ar_var('mypos', list, None)
        self.create_aw_var('need_to_pick', bool, None)
        self.create_aw_var('agv_has_part', bool, None)

    def loop_body(self):
        #print('floor robot enter into loop..................')
        if self.read_from_shared('process', None) == 'main':
            if self.locals['state'] == "IDLE" and self.read_from_shared('part_pose', None) != None:
                self.write_to_actuator('Arm.reach_pos', self.read_from_shared('part_pose', None))
                self.locals['state'] = "PICK"
                return
            if self.locals['state'] == "PICK" and self.read_from_sensor('Arm.reached'):
                self.write_to_actuator('Gripper.enable_suction', True)
                self.locals['state'] = "PICKED"
                return
            if self.locals['state'] == "PICKED" and self.read_from_sensor('Gripper.enabled'):
                #self.check_duration('Gripper.attached', True)
                self.write_to_actuator('Arm.reach_pos', self.read_from_shared('part_to_pick_loc', None))
                self.locals['state'] = "DROP"
                return
            if self.locals['state'] == "DROP" and self.read_from_sensor('Arm.reached'):
                self.write_to_actuator('Gripper.enable_suction', False)
                self.locals['state'] = "DROPPED"
                return
            if self.locals['state'] == "DROPPED":
                self.locals['state'] = "END"
                self.reset()
                self.write_to_shared('all_loaded', None, True)
                return
        if self.read_from_shared('process', None) == 'floor_robot_duration':
            if self.read_from_shared('exit', None):
                self.write_to_shared('process', None, 'main')
                return
            if self.locals['state2'] == 'IDLE':
                self.locals['target_pose1'] = self.check_part_on_bin(self.read_from_shared('part', None))
                if self.locals['target_pose1']:
                    self.write_to_actuator('Arm.reach_pos', self.locals['target_pose1'])
                    self.locals['state2'] = 'PICK'
                else:
                    self.write_to_shared('exit', None, True)
                return
            if self.locals['state2'] == 'PICK':
                self.write_to_actuator('Gripper.enable_suction', True)
                self.locals['state2'] = "PICKED"
                return
            if self.locals['state2'] == 'PICKED':
                self.write_to_actuator('Arm.reach_pos', self.moat.utils.compute_tray_loc('agv1', 4))
                self.locals['state2'] = "DROP"
                return
            if self.locals['state2'] == 'DROP' and self.read_from_sensor('Arm.reached'):
                self.write_to_actuator('Gripper.enable_suction', False)
                self.locals['state2'] = "DROPPED"
                return
            if self.locals['state2'] == "DROPPED":
                self.locals['state2'] = "END"
                self.reset()
                return
        if self.read_from_shared('process', None) == 'ceiling_robot_duration':
            if self.read_from_shared('exit', None):
                self.write_to_shared('process', None, 'main')
                return
            if self.locals['state1'] == 'IDLE' and self.read_from_shared('need_to_pick', None):
                self.locals['target_pose'] = self.check_part_on_bin(self.read_from_shared('part', None))
                if self.locals['target_pose']:
                    self.write_to_actuator('Arm.reach_pos', self.locals['target_pose'])
                    self.locals['state1'] = 'PICK'
                else:
                    self.write_to_shared('exit', None, True)
                return
            if self.locals['state1'] == 'PICK':
                self.write_to_actuator('Gripper.enable_suction', True)
                self.locals['state1'] = "PICKED"
                return
            if self.locals['state1'] == 'PICKED':
                self.write_to_actuator('Arm.reach_pos', self.moat.utils.compute_tray_loc('agv1', 1))
                print('-'*50, self.moat.utils.compute_tray_loc('agv1', 1))
                self.locals['state1'] = "DROP"
                return
            if self.locals['state1'] == 'DROP' and self.read_from_sensor('Arm.reached'):
                self.write_to_actuator('Gripper.enable_suction', False)
                self.locals['state1'] = "DROPPED"
                return
            if self.locals['state1'] == "DROPPED":
                self.locals['state1'] = "END"
                self.write_to_shared('agv_has_part', None, True)
                self.reset()
                return

