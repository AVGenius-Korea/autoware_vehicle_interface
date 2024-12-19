from dataclasses import dataclass, fields
from time import time_ns
from kap_dataclass.data_utils import generate_byte_array


@dataclass
class VehicleCtrl:
    pos_lamp_ctrl: int = 0
    head_lamp_ctrl: int = 0
    left_lamp_ctrl: int = 0
    right_lamp_ctrl: int = 0
    speed_limit_mode: int = 0
    speed_limit_val: int = 0
    checksum_en: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'VehicleCtrl' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def get_bytearray(self):
        pos_lamp_ctrl = (self.pos_lamp_ctrl, 0, 0)
        head_lamp_ctrl = (self.head_lamp_ctrl, 1, 1)
        left_lamp_ctrl = (self.left_lamp_ctrl, 2, 2)
        right_lamp_ctrl = (self.right_lamp_ctrl, 3, 3)
        speed_limit_mode = (self.speed_limit_mode, 24, 24)
        speed_limit_val = (self.speed_limit_val, 32, 39)
        checksum_en = (self.checksum_en, 48, 48)

        return generate_byte_array(8,
                                   pos_lamp_ctrl,
                                   head_lamp_ctrl,
                                   left_lamp_ctrl,
                                   right_lamp_ctrl,
                                   speed_limit_mode,
                                   speed_limit_val,
                                   checksum_en,
                                   xor_checksum=False)
