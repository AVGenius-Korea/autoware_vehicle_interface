import time
import math
from rclpy.node import Node
import rclpy

from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_auto_vehicle_msgs.msg import GearCommand
from tier4_control_msgs.msg import GateMode
from tier4_vehicle_msgs.msg import VehicleEmergencyStamped
from autoware_auto_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_auto_vehicle_msgs.msg import HazardLightsCommand
from tier4_vehicle_msgs.msg import ActuationCommandStamped


class TestControlNode(Node):

    def __init__(self):
        super().__init__('test_vehicle_cmd')

        self.timer = self.create_timer(0.02, self.timer_call_back)

        self.control_cmd_publisher = self.create_publisher(AckermannControlCommand, '/control/command/control_cmd', 10)

        self.gear_cmd_publisher = self.create_publisher(GearCommand, '/control/command/gear_cmd', 10)

        self.current_gate_mode_publisher = self.create_publisher(GateMode, '/control/current_gate_mode', 10)

        self.current_emergency_publisher = self.create_publisher(VehicleEmergencyStamped,
                                                                 '/control/command/emergency_cmd', 10)
        self.turn_indicators_cmd_publisher = self.create_publisher(TurnIndicatorsCommand,
                                                                   '/control/command/turn_indicators_cmd', 10)
        self.hazard_lights_cmd_publisher = self.create_publisher(HazardLightsCommand,
                                                                 '/control/command/hazard_lights_cmd', 10)

        self.actuation_cmd_publisher = self.create_publisher(ActuationCommandStamped, '/control/command/actuation_cmd',
                                                             10)

    def timer_call_back(self):
        self.get_logger().info('Control test')

        msg_obj_gear_cmd = GearCommand()

        msg_obj_current_gate_mode = GateMode()

        msg_obj_emergency_cmd = VehicleEmergencyStamped()

        msg_obj_turn_indicators_cmd = TurnIndicatorsCommand()

        msg_obj_hazard_lights_cmd = HazardLightsCommand()

        msg_obj_actuation_cmd = ActuationCommandStamped()

        msg_obj_actuation_cmd.actuation.accel_cmd = 0.0
        msg_obj_actuation_cmd.actuation.brake_cmd = 0.0
        msg_obj_actuation_cmd.actuation.steer_cmd = 0.0

        # NEUTRAL = 1
        # DRIVE = 2
        # REVERSE = 20
        # PARK = 22
        msg_obj_gear_cmd.command = 1

        msg_obj_emergency_cmd.emergency = False
        msg_obj_hazard_lights_cmd.command = 1
        msg_obj_turn_indicators_cmd.command = 3

        self.actuation_cmd_publisher.publish(msg_obj_actuation_cmd)
        self.gear_cmd_publisher.publish(msg_obj_gear_cmd)
        self.current_emergency_publisher.publish(msg_obj_emergency_cmd)
        self.hazard_lights_cmd_publisher.publish(msg_obj_hazard_lights_cmd)
        self.turn_indicators_cmd_publisher.publish(msg_obj_turn_indicators_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = TestControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
