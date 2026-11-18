from pydantic import BaseModel, Field, model_validator
from typing import Literal, Self


class MotorConfig(BaseModel):
    direction_pin: int = Field(description="GPIO pin controlling the motor direction signal.")
    enable_pin: int = Field(description="GPIO pin toggling the motor driver enable line.")
    step_pin: int = Field(description="GPIO pin used to emit step pulses.")

    backend: Literal["gpio", "marlin"] = Field(
        "gpio", description="Motion backend used to drive this motor."
    )
    marlin_axis: Literal["X", "Y"] | None = Field(
        None, description="Marlin axis for this motor; inferred from the motor name when omitted."
    )
    marlin_port: str = Field(
        "/dev/ttyUSB0", description="Serial device of the Marlin controller."
    )
    marlin_baudrate: int = Field(
        115200, ge=9600, le=1000000, description="Baud rate of the Marlin controller."
    )
    marlin_max_feedrate: float = Field(
        600.0, gt=0, le=18000, description="Maximum Marlin feedrate in degrees per minute."
    )

    acceleration: int = Field(20000, ge=1000, le=80000, description="Acceleration in steps/s², Limits tested on Rpi 4 2GB under full load --> time estimation within 0.5%")
    max_speed: int = Field(5000, ge=1, le=7500, description="Steps per second, Limits tested on RPi 4 2GB under full load --> time estimation within 0.5%")

    direction: Literal[1, -1] = Field(1, description="Motor direction (1 or -1).")
    steps_per_rotation: int = Field(description="Number of steps for a full motor rotation.")

    min_angle: float = Field(0, ge=0, le=360, description="Minimum allowed angle for the motor in degrees.")
    max_angle: float = Field(360, ge=0, le=360, description="Maximum allowed angle for the motor in degrees.")
    home_angle: float = Field(90, ge=0, le=360, description="Angle for home position in degrees.")

    @model_validator(mode='after')
    def check_angles(self) -> Self:
        if self.min_angle > self.max_angle:
            raise ValueError("min_angle cannot be greater than max_angle.")
        if  self.home_angle  < self.min_angle or self.home_angle > self.max_angle:
            raise ValueError("home angle must be between min_angle and max_angle.")
        return self

   
