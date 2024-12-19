from dataclasses import dataclass, fields

@dataclass
class VehicleStaFb:
    pos_lamp_fb: int = 0
    head_lamp_fb: int = 0
    left_lamp_fb: int = 0
    right_lamp_fb: int = 0
    hazard_war_lamp_fb: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'VehicleStaFb' object has no attribute '{field_name}'")
        return getattr(self, field_name)
