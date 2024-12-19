from dataclasses import dataclass, fields
from time import time_ns
from kap_dataclass.data_utils import generate_byte_array


@dataclass
class DriveCtrl:
    driver_en_ctrl: int = 0
    driver_mode_ctrl: int = 0
    gear_ctrl: int = 0
    speed_ctrl: int = 0
    throttle_pdl_target: int = 0
    drive_life_sig: int = 0
    checksum_130: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'DriveCtrl' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_cycle_count(self):

        if self.drive_life_sig >= 15:
            self.drive_life_sig = 0

        if self.checksum_130 >= 255:
            self.checksum_130 = 0

        self.drive_life_sig += 1
        self.checksum_130 += 1

    def get_bytearray(self):
        driver_en_ctrl = (self.driver_en_ctrl, 0, 0)
        driver_mode_ctrl = (self.driver_mode_ctrl, 2, 3)
        gear_ctrl = (self.gear_ctrl, 4, 5)
        speed_ctrl_lower = (self.speed_ctrl & 0xFF, 8, 15)
        speed_ctrl_upper = ((self.speed_ctrl >> 8) & 0xFF, 16, 23)
        throttle_pdl_target_lower = (self.throttle_pdl_target & 0xFF, 24, 31)
        throttle_pdl_target_upper = (self.throttle_pdl_target >> 8, 32, 33)
        drive_life_sig = (self.drive_life_sig, 48, 51)
        checksum_130 = (self.checksum_130, 56, 63)

        return generate_byte_array(8,
                                   driver_en_ctrl,
                                   driver_mode_ctrl,
                                   gear_ctrl,
                                   speed_ctrl_lower,
                                   speed_ctrl_upper,
                                   throttle_pdl_target_lower,
                                   throttle_pdl_target_upper,
                                   drive_life_sig,
                                   checksum_130,
                                   xor_checksum=False)
