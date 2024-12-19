from dataclasses import dataclass, fields

@dataclass
class VehicleWorkStaFb:
    driving_mode_fb: int = 0
    power_sta_fb: int = 0
    power_dc_sta: int = 0
    speed_limited_mode_fb: int = 0
    speed_limited_val_fb: float = 0.0
    low_power_volt_sta: float = 0.0
    estop_sta_fb: int = 0
    crash_front_sta: int = 0
    crash_rear_sta: int = 0
    life: int = 0
    checksum: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'ThrottleCtrlData' object has no attribute '{field_name}'")
        return getattr(self, field_name)
