from dataclasses import dataclass, fields

@dataclass
class DriveStaFb:
    driver_en_sta: int = 0
    driver_slop_over: int = 0
    driver_mode_sta: int = 0
    gear_fb: int = 0
    speed_fb: float = 0.00
    throttle_pald_fb: float = 0.0
    acceleration_fb: float = 0.00
    drive_life: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'DriveStaFb' object has no attribute '{field_name}'")
        return getattr(self, field_name)
