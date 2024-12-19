from dataclasses import dataclass, fields


@dataclass
class BrakeStaFb:
    brake_en_sta: int = 0
    vehicle_brake_lamp_fb: int = 0
    epb_fb: int = 0
    brake_padl_fb: float = 0.0
    brake_pressure_fb: int = 0
    brake_life: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'BrakeStaFb' object has no attribute '{field_name}'")
        return getattr(self, field_name)
