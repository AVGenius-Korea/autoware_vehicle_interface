from dataclasses import dataclass, fields
from time import time_ns
from kap_dataclass.data_utils import generate_byte_array


@dataclass
class BrakeCtrl:
    brake_en: int = 0
    brake_lamp_ctrl: int = 0
    brake_pdl_target: int = 0
    epb_ctrl: int = 0
    brake_life_sig: int = 0
    checksum_131: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'{self.__class__.__name__}' object has no attribute '{field_name}'")
        return getattr(self, field_name)

    def add_cycle_count(self):

        if self.brake_life_sig >= 15:
            self.brake_life_sig = 0

        if self.checksum_131 >= 255:
            self.checksum_131 = 0

        self.brake_life_sig += 1
        self.checksum_131 += 1

    def get_bytearray(self):
        brake_en = (self.brake_en, 0, 0)
        brake_lamp_ctrl = (self.brake_lamp_ctrl, 1, 1)
        brake_pdl_target_lower = (self.brake_pdl_target & 0xFF, 8, 15)
        brake_pdl_target_upper = (self.brake_pdl_target >> 8, 16, 17)
        epb_ctrl = (self.epb_ctrl, 24, 25)
        brake_life_sig = (self.brake_life_sig, 48, 51)
        checksum_131 = (self.checksum_131, 56, 63)

        return generate_byte_array(8,
                                   brake_en,
                                   brake_lamp_ctrl,
                                   brake_pdl_target_lower,
                                   brake_pdl_target_upper,
                                   epb_ctrl,
                                   brake_life_sig,
                                   checksum_131,
                                   xor_checksum=False)
