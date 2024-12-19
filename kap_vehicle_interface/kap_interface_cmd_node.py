from rclpy.node import Node
import rclpy
from time import time_ns
import math

from can_utils.can_sender import CANSender

from autoware_auto_vehicle_msgs.msg import GearCommand
from tier4_control_msgs.msg import GateMode
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
from autoware_auto_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_auto_vehicle_msgs.msg import HazardLightsCommand
from tier4_vehicle_msgs.msg import ActuationCommandStamped

from kap_dataclass.SteerCtrl import SteerCtrl
from kap_dataclass.BrakeCtrl import BrakeCtrl
from kap_dataclass.DriveCtrl import DriveCtrl
from kap_dataclass.VehicleCtrl import VehicleCtrl


class CANCommandNode(Node):

    def __init__(self):
        super().__init__('CANCommandNode')
        self.can_channel = 'can2'
        self.can_sender = CANSender(self.can_channel)

        self.sub_gear_cmd = self.create_subscription(GearCommand, '/control/command/gear_cmd',
                                                     self.dispatch_command, 10)

        self.sub_gate_mode_cmd = self.create_subscription(GateMode, '/control/current_gate_mode',
                                                          self.dispatch_command, 10)

        self.sub_vehicle_emergency_cmd = self.create_subscription(VehicleEmergencyStamped,
                                                                  '/control/command/emergency_cmd',
                                                                  self.dispatch_command, 10)

        self.sub_turn_indicators_cmd = self.create_subscription(TurnIndicatorsCommand,
                                                                '/control/command/turn_indicators_cmd',
                                                                self.dispatch_command, 10)

        self.sub_turn_hazard_lights_cmd = self.create_subscription(HazardLightsCommand,
                                                                   '/control/command/hazard_lights_cmd',
                                                                   self.dispatch_command, 10)

        self.sub_actuation = self.create_subscription(ActuationCommandStamped,
                                                      '/control/command/actuation_cmd',
                                                      self.dispatch_command, 10)

        self.drive_ctrl_send_timer = self.create_timer(0.02, self.drive_ctrl_data_timer_callback)
        self.brake_ctrl_send_timer = self.create_timer(0.02, self.brake_ctrl_data_timer_callback)
        self.steer_ctrl_send_timer = self.create_timer(0.02, self.steer_data_timer_callback)
        self.vehicle_ctrl_send_timer = self.create_timer(0.02, self.vehicle_ctrl_data_timer_callback)

        self.steer_ctrl_data = SteerCtrl()
        self.brake_ctrl_data = BrakeCtrl()
        self.drive_ctrl_data = DriveCtrl()
        self.vehicle_ctrl_data = VehicleCtrl()

        self.set_steer_ctrl_default_value()
        self.set_brake_ctrl_default_value()
        self.set_drive_ctrl_default_value()
        self.set_vehicle_ctrl_default_value()

    def set_steer_ctrl_default_value(self):

        steer_ctrl_cmd = {
            'steer_en_ctrl': 1,
            'steer_mode_ctrl': 0,
            'steer_angle_target': 0,
            'steer_angle_speed_ctrl': 480
        }

        self.steer_ctrl_data.update_value(**steer_ctrl_cmd)

    def set_brake_ctrl_default_value(self):

        brake_ctrl_cmd = {
            'brake_en': 1,
            'brake_pdl_target': 0,
        }

        self.brake_ctrl_data.update_value(**brake_ctrl_cmd)

    def set_drive_ctrl_default_value(self):

        drive_ctrl_cmd = {
            'driver_en_ctrl': 1,
            'driver_mode_ctrl': 1,
            'gear_ctrl': 1,
            'speed_ctrl': 0,
            'throttle_pdl_target': 0,
        }

        self.drive_ctrl_data.update_value(**drive_ctrl_cmd)

    def set_vehicle_ctrl_default_value(self):

        vehicle_ctrl_cmd = {
            'pos_lamp_ctrl': 0,
            'head_lamp_ctrl': 0,
            'left_lamp_ctrl': 0,
            'right_lamp_ctrl': 0,
            'speed_limit_mode': 1,
            'speed_limit_val': 25,
            'checksum_en': 1
        }
        self.vehicle_ctrl_data.update_value(**vehicle_ctrl_cmd)

    def dispatch_command(self, msg):

        if isinstance(msg, ActuationCommandStamped):
            command_data_throttle = {
                'driver_en_ctrl': 1,
                'throttle_pdl_target': int(msg.actuation.accel_cmd / 0.001),
            }
            command_data_brake = {
                'brake_en': 1,
                'brake_pdl_target': int(msg.actuation.brake_cmd / 0.001),
            }
            command_data_steering = {
                'steer_en_ctrl': 1,
                'steer_angle_target': int(msg.actuation.steer_cmd / 0.001) * -1,
            }
            self.drive_ctrl_data.update_value(**command_data_throttle)
            self.brake_ctrl_data.update_value(**command_data_brake)
            self.steer_ctrl_data.update_value(**command_data_steering)

        elif isinstance(msg, GearCommand):
            gear_command = 0
            parking_command = 2

            # NEUTRAL
            if msg.command == 1:
                gear_command = 2

            # DRIVE
            elif 2 <= msg.command <= 19:
                gear_command = 1

            # REVERSE
            elif 20 <= msg.command <= 21:
                gear_command = 3

            # PARK :
            elif msg.command == 22:
                gear_command = 2
                parking_command = 1

            parking_cmd = {
                'brake_en': 1,
                'epb_ctrl': parking_command
            }

            gear_ctrl_cmd = {
                'driver_en_ctrl': 1,
                'gear_ctrl': gear_command,
            }

            self.drive_ctrl_data.update_value(**gear_ctrl_cmd)
            self.brake_ctrl_data.update_value(**parking_cmd)
        elif isinstance(msg, GateMode):
            pass

        elif isinstance(msg, VehicleEmergencyStamped):
            if msg.emergency:
                epb_ctrl = 1
            else:
                epb_ctrl = 0

            epb_ctrl_cmd = {
                'brake_en': 1,
                'epb_ctrl': epb_ctrl
            }

            self.brake_ctrl_data.update_value(**epb_ctrl_cmd)
        elif isinstance(msg, TurnIndicatorsCommand):
            left_lamp_ctrl = 0
            right_lamp_ctrl = 0

            if msg.command == 2:
                left_lamp_ctrl = 1
            elif msg.command == 3:
                right_lamp_ctrl = 1

            turn_indicator_ctrl_cmd = {
                'left_lamp_ctrl': left_lamp_ctrl,
                'right_lamp_ctrl': right_lamp_ctrl
            }
            self.vehicle_ctrl_data.update_value(**turn_indicator_ctrl_cmd)

        elif isinstance(msg, HazardLightsCommand):
            if msg.command == 2:
                turn_indicator_ctrl_cmd = {
                    'left_lamp_ctrl': 1,
                    'right_lamp_ctrl': 1
                }
                self.vehicle_ctrl_data.update_value(**turn_indicator_ctrl_cmd)
            else:
                pass

    def drive_ctrl_data_timer_callback(self):
        self.drive_ctrl_data.add_cycle_count()
        self.can_sender.send(0x130, self.drive_ctrl_data.get_bytearray())

    def brake_ctrl_data_timer_callback(self):
        self.brake_ctrl_data.add_cycle_count()
        self.can_sender.send(0x131, self.brake_ctrl_data.get_bytearray())

    def steer_data_timer_callback(self):
        self.steer_ctrl_data.add_cycle_count()
        self.can_sender.send(0x132, self.steer_ctrl_data.get_bytearray())

    def vehicle_ctrl_data_timer_callback(self):
        self.can_sender.send(0x133, self.vehicle_ctrl_data.get_bytearray())


def main(args=None):
    rclpy.init(args=args)
    node = CANCommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
