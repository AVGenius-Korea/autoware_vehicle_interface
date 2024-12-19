import math

from rclpy.node import Node
from autoware_auto_vehicle_msgs.msg import (ControlModeReport,
                                            GearReport,
                                            HazardLightsReport,
                                            TurnIndicatorsReport,
                                            SteeringReport,
                                            VelocityReport)
import can
import rclpy
import threading
from struct import unpack




class CANReceiverNode(Node):
    """
    Node responsible for receiving CAN messages and publishing them as ROS2 messages.

    TODO: Check publisher queue size.
    """

    def __init__(self):
        super().__init__('CANReportNode')

        self.bus = can.Bus(interface='socketcan', channel='can2', bitrate=500000)
        self.running = False
        self.receive_thread = threading.Thread(target=self.receive_data)

        # Report message object
        self.msg_obj_control_mode_rpt = ControlModeReport()
        self.msg_obj_gear_rpt = GearReport()
        self.msg_obj_hazardLights_rpt = HazardLightsReport()
        self.msg_obj_indicators_rpt = TurnIndicatorsReport()
        self.msg_obj_steering_rpt = SteeringReport()
        self.msg_obj_velocity_rpt = VelocityReport()

        # Report data class


        # vehicle status report publisher
        self.control_mode_rpt_pub = self.create_publisher(ControlModeReport, '/vehicle/status/control_mode', 10)
        self.gear_rpt_pub = self.create_publisher(GearReport, '/vehicle/status/gear_status', 10)
        self.hazard_lights_rpt_pub = self.create_publisher(HazardLightsReport,
                                                           '/vehicle/status/hazard_lights_status', 10)
        self.turn_indicators_rpt_pub = self.create_publisher(TurnIndicatorsReport,
                                                             '/vehicle/status/turn_indicators_status', 10)
        self.steering_rpt_pub = self.create_publisher(SteeringReport, '/vehicle/status/steering_status', 10)
        self.velocity_rpt_pub = self.create_publisher(VelocityReport, '/vehicle/status/velocity_status', 10)

        # publisher timer
        self.control_mode_rpt_timer = self.create_timer(0.05, self.control_mode_rpt_timer_callback)
        self.gear_rpt_timer = self.create_timer(0.05, self.gear_rpt_timer_callback)
        self.hazard_lights_rpt_timer = self.create_timer(0.05, self.hazard_lights_rpt_timer_callback)
        self.turn_indicators_rpt_timer = self.create_timer(0.05, self.turn_indicators_rpt_timer_callback)
        self.steering_rpt_timer = self.create_timer(0.05, self.steering_rpt_timer_callback)
        self.velocity_rpt_timer = self.create_timer(0.05, self.velocity_rpt_timer_callback)

        self.heading_rate = 0.0

        self.start()

    def start(self):
        self.running = True
        self.receive_thread.start()

    def stop(self):
        self.running = False
        self.receive_thread.join()

    def imu_callback(self, msg):
        self.heading_rate = msg.angular_velocity.z

    def receive_data(self):
        """
        Continuously receives CAN message and processes them.
        """
        while self.running:
            try:
                message = self.bus.recv()
                can_id = message.arbitration_id
                data = message.data

                if message.is_error_frame:
                    pass
                # TODO : If the message contains an error frame, additional code for data processing is required.
                if message.is_remote_frame:
                    pass
                # TODO : Requires processing when the message contains a remote request frame
                if not message.is_extended_id and message.dlc == 8 and not message.is_remote_frame:
                    self.process_can_data_and_publish(can_id, data)
            except can.CanError as _:
                pass

    def process_can_data_and_publish(self, can_id, data):
        # Byte Order : little endian = intel endian
        # Drive Status Feedback Parsing
        if can_id == 0x530:
            driver_en_sta = unpack('<B', data[0:1])[0] & 0b00000001
            diver_slop_over = unpack('<B', data[0:1])[0] >> 1 & 0b00000001
            driver_mode_sta = unpack('<B', data[0:1])[0] >> 2 & 0b00000011
            gear_fb = unpack('<B', data[0:1])[0] >> 4 & 0b00000011
            speed_fb = unpack('<h', data[1:3])[0] * 0.01
            throttle_pald_fb = unpack('<H', data[3:5])[0] * 0.1
            acceleration_fb = unpack('<h', data[5:7])[0] * 0.01
            drive_life = unpack('<B', data[7:8])[0] & 0b00001111

            data = {
                'driver_en_sta': driver_en_sta,
                'diver_slop_over': diver_slop_over,
                'driver_mode_sta': driver_mode_sta,
                'gear_fb': gear_fb,
                'speed_fb': speed_fb,
                'throttle_pald_fb': throttle_pald_fb,
                'acceleration_fb': acceleration_fb,
                'drive_life': drive_life,
            }
            self.drive_sta_fb.update_value(**data)

        # Brake Status FeedBack Parsing
        elif can_id == 0x531:
            brake_en_sta = unpack('<B', data[0:1])[0] & 0b00000001
            vehicle_brake_lamp_fb = unpack('<B', data[0:1])[0] >> 2 & 0b00000001
            epb_fb = unpack('<B', data[0:1])[0] >> 4 & 0b00000011
            brake_padl_fb = unpack('<H', data[1:3])[0] * 0.1
            brake_pressure_fb = unpack('<B', data[3:4])[0]
            brake_life = unpack('<B', data[6:7])[0] & 0b00001111

            data = {
                'brake_en_sta': brake_en_sta,
                'vehicle_brake_lamp_fb': vehicle_brake_lamp_fb,
                'epb_fb': epb_fb,
                'brake_padl_fb': brake_padl_fb,
                'brake_pressure_fb': brake_pressure_fb,
                'brake_life': brake_life

            }
            self.brake_sta_fb.update_value(**data)

        # Steering Status FeedBack Parsing
        elif can_id == 0x532:
            steer_en_sta = unpack('<B', data[0:1])[0] & 0b00000001
            steer_slop_over = unpack('<B', data[0:1])[0] >> 1 & 0b00000001
            steer_mode_fb = unpack('<B', data[0:1])[0] >> 4 & 0b00001111
            steer_angle_fb = unpack('<h', data[1:3])[0]
            steer_angle_rear_fb = unpack('<h', data[3:5])[0]
            steer_angle_speed_fb = unpack('<B', data[5:6])[0] * 2
            steer_life = unpack('<B', data[6:7])[0] & 0b00001111

            data = {
                'steer_en_sta': steer_en_sta,
                'steer_slop_over': steer_slop_over,
                'steer_mode_fb': steer_mode_fb,
                'steer_angle_fb': steer_angle_fb,
                'steer_angle_rear_fb': steer_angle_rear_fb,
                'steer_angle_speed_fb': steer_angle_speed_fb,
                'steer_life': steer_life

            }

            self.steer_sta_fb.update_value(**data)

        # Vehicle Work Status FeedBack Parsing
        elif can_id == 0x534:
            driving_mode_fb = unpack('<B', data[0:1])[0] & 0b00000011
            power_sta_fb = unpack('<B', data[0:1])[0] >> 2 & 0b00000011
            power_dc_sta = unpack('<B', data[0:1])[0] >> 4 & 0b00000011
            speed_limited_mode_fb = unpack('<B', data[1:2])[0] & 0b00000001
            speed_limited_val_fb = unpack('<H', data[2:4])[0] * 0.1
            low_power_volt_sta = unpack('<B', data[4:5])[0] * 0.1
            estop_sta_fb = unpack('<B', data[5:6])[0] & 0b00001111
            crash_front_sta = unpack('<B', data[5:6])[0] >> 4 & 0b00000001
            crash_rear_sta = unpack('<B', data[5:6])[0] >> 5 & 0b00000001
            life = unpack('<B', data[6:7])[0] & 0b00001111
            checksum = unpack('<B', data[7:8])[0]

            data = {
                'driving_mode_fb': driving_mode_fb,
                'power_sta_fb': power_sta_fb,
                'power_dc_sta': power_dc_sta,
                'speed_limited_mode_fb': speed_limited_mode_fb,
                'speed_limited_val_fb': speed_limited_val_fb,
                'low_power_volt_sta': low_power_volt_sta,
                'estop_sta_fb': estop_sta_fb,
                'crash_front_sta': crash_front_sta,
                'crash_rear_sta': crash_rear_sta,
                'life': life,
                'checksum': checksum
            }
            self.vehicle_work_sta_fb.update_value(**data)

        # Power Status FeedBack Parsing
        elif can_id == 0x535:
            power_charge_sta = unpack('<B', data[0:1])[0] >> 4 & 0b00000011
            power_soc_fb = unpack('<B', data[1:2])[0]
            power_volt_fb = unpack('<H', data[2:4])[0] * 0.1
            power_curr_fb = unpack('<H', data[4:6])[0] * 0.1 - 1000
            bms_max_temp = unpack('<B', data[6:7])[0] - 40

            data = {
                'power_charge_sta': power_charge_sta,
                'power_soc_fb': power_soc_fb,
                'power_volt_fb': power_volt_fb,
                'power_curr_fb': power_curr_fb,
                'bms_max_temp': bms_max_temp
            }

            self.power_sta_fb.update_value(**data)

        # Vehicle Status FeedBack Parsing
        elif can_id == 0x536:
            pos_lamp_fb = unpack('<B', data[0:1])[0] & 0b00000001
            head_lamp_fb = unpack('<B', data[0:1])[0] >> 1 & 0b00000001
            left_lamp_fb = unpack('<B', data[0:1])[0] >> 2 & 0b00000001
            right_lamp_fb = unpack('<B', data[0:1])[0] >> 3 & 0b00000001
            hazard_war_lamp_fb = unpack('<B', data[0:1])[0] >> 6 & 0b00000001

            data = {
                'pos_lamp_fb': pos_lamp_fb,
                'head_lamp_fb': head_lamp_fb,
                'left_lamp_fb': left_lamp_fb,
                'right_lamp_fb': right_lamp_fb,
                'hazard_war_lamp_fb': hazard_war_lamp_fb
            }
            self.vehicle_sta_fb.update_value(**data)

        # Vehicle Fault Status Parsing
        elif can_id == 0x537:
            motor_over_temp_sta = unpack('<B', data[0:1])[0] & 0b00000001
            bms_over_temp_sta = unpack('<B', data[0:1])[0] >> 1 & 0b00000001
            brake_over_temp_sta = unpack('<B', data[0:1])[0] >> 2 & 0b00000001
            steer_over_temp_sta = unpack('<B', data[0:1])[0] >> 3 & 0b00000001
            under_volt = unpack('<B', data[0:1])[0] >> 4 & 0b00000001
            sys_flt = unpack('<B', data[1:2])[0] & 0b00001111
            brake_flt = unpack('<B', data[1:2])[0] >> 4 & 0b00001111
            parking_flt = unpack('<B', data[2:3])[0] & 0b00001111
            steer_front_flt = unpack('<B', data[2:3])[0] >> 4 & 0b00001111
            steer_back_flt = unpack('<B', data[3:4])[0] & 0b00001111
            motor_lf_flt = unpack('<B', data[3:4])[0] >> 4 & 0b00001111
            motor_rf_flt = unpack('<B', data[4:5])[0] & 0b00001111
            motor_lr_flt = unpack('<B', data[4:5])[0] >> 4 & 0b00001111
            motor_rr_flt = unpack('<B', data[5:6])[0] & 0b00001111
            bms_flt = unpack('<B', data[5:6])[0] >> 4 & 0b00001111
            dc_flt = unpack('<B', data[6:7])[0] & 0b00001111

            data = {
                'motor_over_temp_sta': motor_over_temp_sta,
                'bms_over_temp_sta': bms_over_temp_sta,
                'brake_over_temp_sta': brake_over_temp_sta,
                'steer_over_temp_sta': steer_over_temp_sta,
                'under_volt': under_volt,
                'sys_flt': sys_flt,
                'brake_flt': brake_flt,
                'parking_flt': parking_flt,
                'steer_front_flt': steer_front_flt,
                'steer_back_flt': steer_back_flt,
                'motor_lf_flt': motor_lf_flt,
                'motor_rf_flt': motor_rf_flt,
                'motor_lr_flt': motor_lr_flt,
                'motor_rr_flt': motor_rr_flt,
                'bms_flt': bms_flt,
                'dc_flt': dc_flt
            }
            self.vehicle_flt_sta.update_value(**data)

        # Chassis Wheel Rpm FeedBack Parsing
        elif can_id == 0x539:
            wheel_rpm_lf = unpack('<h', data[0:2])[0]
            wheel_rpm_rf = unpack('<h', data[2:4])[0]
            wheel_rpm_lr = unpack('<h', data[4:6])[0]
            wheel_rpm_rr = unpack('<h', data[6:8])[0]

            data = {
                'wheel_rpm_lf': wheel_rpm_lf,
                'wheel_rpm_rf': wheel_rpm_rf,
                'wheel_rpm_lr': wheel_rpm_lr,
                'wheel_rpm_rr': wheel_rpm_rr
            }

            self.wheel_rpm_fb.update_value(**data)

    def control_mode_rpt_timer_callback(self):
        casted_control_mode = 0
        _val = self.vehicle_work_sta_fb.get_value('driving_mode_fb')
        if _val == 0 or _val == 1:
            casted_control_mode = 1
        elif _val == 2 or _val == 3:
            casted_control_mode = 4

        self.msg_obj_control_mode_rpt.mode = casted_control_mode
        self.control_mode_rpt_pub.publish(self.msg_obj_control_mode_rpt)

    def hazard_lights_rpt_timer_callback(self):

        casted_hazard_lights = 0
        _val = self.vehicle_sta_fb.get_value('hazard_war_lamp_fb')
        if _val == 0:
            casted_hazard_lights = 1
        elif _val == 1:
            casted_hazard_lights = 2

        self.msg_obj_hazardLights_rpt.report = casted_hazard_lights
        self.hazard_lights_rpt_pub.publish(self.msg_obj_hazardLights_rpt)

    def turn_indicators_rpt_timer_callback(self):

        _val1 = self.vehicle_sta_fb.get_value('left_lamp_fb')
        _val2 = self.vehicle_sta_fb.get_value('right_lamp_fb')
        _val3 = self.vehicle_sta_fb.get_value('hazard_war_lamp_fb')

        if _val3 == 0 and _val1 == 1:
            casted_turn_indicators = 2
        elif _val3 == 0 and _val2 == 1:
            casted_turn_indicators = 3
        else:
            casted_turn_indicators = 1

        self.msg_obj_indicators_rpt.report = casted_turn_indicators
        self.turn_indicators_rpt_pub.publish(self.msg_obj_indicators_rpt)

    def steering_rpt_timer_callback(self):
        casted_steering_tire_angle = 0.0
        _val1 = float(self.steer_sta_fb.get_value('steer_angle_fb')) * 0.001

        if _val1 != 0.0:
            casted_steering_tire_angle = _val1 * -1

        self.msg_obj_steering_rpt.steering_tire_angle = casted_steering_tire_angle
        self.steering_rpt_pub.publish(self.msg_obj_steering_rpt)

    def gear_rpt_timer_callback(self):
        casted_gear = 0
        _val = self.drive_sta_fb.get_value('gear_fb')
        _val2 = self.brake_sta_fb.get_value('epb_fb')
        # D
        if _val2 == 0 and _val == 1:
            casted_gear = 2
        # N
        elif _val2 == 0 and _val == 2:
            casted_gear = 1
        # R
        elif _val2 == 0 and _val == 3:
            casted_gear = 20
        # P
        elif _val2 == 1:
            casted_gear = 22

        self.msg_obj_gear_rpt.report = casted_gear
        self.gear_rpt_pub.publish(self.msg_obj_gear_rpt)

    def velocity_rpt_timer_callback(self):
        self.msg_obj_velocity_rpt.header.stamp = self.get_clock().now().to_msg()
        self.msg_obj_velocity_rpt.header.frame_id = "base_link"
        self.msg_obj_velocity_rpt.longitudinal_velocity = float(self.drive_sta_fb.get_value('speed_fb'))
        self.msg_obj_velocity_rpt.heading_rate = self.heading_rate
        self.velocity_rpt_pub.publish(self.msg_obj_velocity_rpt)


def main(args=None):
    rclpy.init(args=args)
    node = CANReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
