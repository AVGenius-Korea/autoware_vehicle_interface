from dataclasses import dataclass, fields
from time import time_ns
from kap_dataclass.data_utils import generate_byte_array


@dataclass
class SteerCtrl:
    steer_en_ctrl: int = 0
    steer_mode_ctrl: int = 0
    steer_angle_target: int = 0
    steer_angle_rear_target: int = 0
    steer_angle_speed_ctrl: int = 0
    steer_life_sig: int = 0
    checksum_132: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'SteerCtrl' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_cycle_count(self):

        if self.steer_life_sig >= 15:
            self.steer_life_sig = 0

        if self.checksum_132 >= 255:
            self.checksum_132 = 0

        self.steer_life_sig += 1
        self.checksum_132 += 1

    def get_bytearray(self):
        steer_en_ctrl = (self.steer_en_ctrl, 0, 0)
        steer_mode_ctrl = (self.steer_mode_ctrl, 4, 7)
        steer_angle_target_lower = (self.steer_angle_target & 0xFF, 8, 15)
        steer_angle_target_upper = ((self.steer_angle_target >> 8) & 0xFF, 16, 23)
        steer_angle_rear_target_lower = (self.steer_angle_rear_target & 0xFF, 24, 31)
        steer_angle_rear_target_upper = ((self.steer_angle_rear_target >> 8) & 0xFF, 32, 39)
        steer_angle_speed_ctrl = (self.steer_angle_speed_ctrl, 40, 47)
        steer_life_sig = (self.steer_life_sig, 48, 53)
        checksum_132 = (self.checksum_132, 56, 63)

        return generate_byte_array(8,
                                   steer_en_ctrl,
                                   steer_mode_ctrl,
                                   steer_angle_target_lower,
                                   steer_angle_target_upper,
                                   steer_angle_rear_target_lower,
                                   steer_angle_rear_target_upper,
                                   steer_angle_speed_ctrl,
                                   steer_life_sig,
                                   checksum_132,
                                   xor_checksum=False)
