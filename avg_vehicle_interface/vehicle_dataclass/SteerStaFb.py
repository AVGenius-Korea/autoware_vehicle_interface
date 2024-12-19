from dataclasses import dataclass, fields

@dataclass
class SteerStaFb:
    steer_en_sta: int = 0
    steer_slop_over: int = 0
    steer_mode_fb: int = 0
    steer_angle_fb: int = 0
    steer_angle_rear_fb: int = 0
    steer_angle_speed_fb: int = 0
    steer_life: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'SteerStaFb' object has no attribute '{field_name}'")
        return getattr(self, field_name)
