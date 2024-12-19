from dataclasses import dataclass, fields


@dataclass
class WheelRpmFb:
    wheel_rpm_lf: int = 0
    wheel_rpm_rf: int = 0
    wheel_rpm_lr: int = 0
    wheel_rpm_rr: int = 0

    def update_value(self, **kwargs) -> None:

        for field in fields(self):
            if field.name in kwargs:
                value = kwargs[field.name]
                setattr(self, field.name, value)

    def get_value(self, field_name):

        if not hasattr(self, field_name):
            raise AttributeError(f"'WheelRpmFb' object has no attribute '{field_name}'")
        return getattr(self, field_name)
