from dataclasses import dataclass, fields

@dataclass
class VehicleFltSta:
    motor_over_temp_sta: int = 0
    bms_over_temp_sta: int = 0
    brake_over_temp_sta: int = 0
    steer_over_temp_sta: int = 0
    under_volt: int = 0
    sys_flt: int = 0
    brake_flt: int = 0
    parking_flt: int = 0
    steer_front_flt: int = 0
    steer_back_flt: int = 0
    motor_lf_flt: int = 0
    motor_rf_flt: int = 0
    motor_lr_flt: int = 0
    motor_rr_flt: int = 0
    bms_flt: int = 0
    dc_flt: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'VehicleFltSta' object has no attribute '{field_name}'")
        return getattr(self, field_name)
