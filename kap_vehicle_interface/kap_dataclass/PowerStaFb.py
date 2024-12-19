from dataclasses import dataclass, fields

@dataclass
class PowerStaFb:
    power_charge_sta: int = 0
    power_soc_fb: int = 0
    power_volt_fb: float = 0.0
    power_curr_fb: float = 0.0
    bms_max_temp: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'PowerStaFb' object has no attribute '{field_name}'")
        return getattr(self, field_name)
