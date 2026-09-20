# Copyright 2026 Mangdang

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at:

#     http://www.apache.org/licenses/LICENSE-2.0

from bam.actuator import CurrentControlledActuator, VoltageControlledActuator
from bam.parameter import Parameter
from bam.testbench import Testbench


class MD01Actuator(VoltageControlledActuator):
    """Mangdang MD01 voltage-controlled servo-actuator.

    Uses the standard BAM voltage-controlled control law
    (``duty_cycle = clip(kp * error_gain * Δq, -max_pwm, max_pwm)``) and the
    DC motor torque equation with back-EMF. The MD01 is reached through the
    AT32F413 driver board behind the ESP32-S3 SPI bridge implemented in
    :mod:`bam.mangdang.spi`; :mod:`bam.mangdang.record` is the matching
    recorder.

    The bridge firmware also exposes a derivative gain (``kd_position``), but
    BAM's voltage-controlled model has no damping term, so ``kd`` is recorded
    as ``0`` by default and is not fitted here.

    .. note::

        Values marked ``TODO`` must be measured on the bench and/or read from
        the MD01 datasheet before this actuator can be identified. See
        ``ADDING_A_MOTOR.md`` for where each quantity comes from.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            # Supply voltage [V] — nominal MD01 bus voltage. The driver board
            # has no rail ADC, so recordings carry this constant; override it
            # with the bench supply's real output when assembling a dataset.
            vin=12.0,
            # Default firmware P-gain, overridden per log by ``load_log``.
            kp=80.0,
            # Converts kp * Δq into a duty cycle in [-1, 1]. Measured with an
            # oscilloscope (ADDING_A_MOTOR.md §3.2); the recorder starts from
            # the same value and passes it through as log metadata.
            error_gain=0.0104,
            # Maximum duty-cycle magnitude; inherited from the measurements on
            # the other BAM voltage-controlled servos — TODO(md01): measure.
            max_pwm=1.0,
            # Firmware current limit [A]. The AT32 reads the torque field of a
            # position command as a max current cap, so this is a real cap; it
            # is conservative until the MD01 rating is confirmed.
            max_current=1.4,
        )

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)] — TODO: datasheet.
        self.model.kt = Parameter(0.223, 0.0, 1.0) 

        # Motor resistance [Ohm]; often estimable as vin / I_stall — TODO.
        self.model.R = Parameter(8.5, 7.5, 9.5)  # TODO

        # Rotor / apparent inertia [kg m^2] — TODO: datasheet or fit seed.
        self.model.armature = Parameter(2.25e-4, 1.1e-4, 6.8e-4) 

        # Optional: fit a ratio on top of error_gain (see ST3025Actuator).
        # self.model.error_gain_ratio = Parameter(1.0, 0.1, 10.0)

    def load_log(self, log: dict):
        """Load per-log settings, tolerating the recorder's extra metadata.

        :class:`~bam.actuator.DCMotorActuator` reads ``kp`` and ``vin``; the
        MD01 recorder also writes ``error_gain`` and ``max_pwm``, which take
        precedence over the class defaults when present, so a dataset stays
        self-consistent with how it was recorded.
        """
        super().load_log(log)
        if "error_gain" in log:
            self.error_gain = log["error_gain"]
        if "max_pwm" in log:
            self.max_pwm = log["max_pwm"]

    def get_extra_inertia(self) -> float:
        return self.model.armature.value


class MD01CurrentActuator(CurrentControlledActuator):
    """Mangdang MD01 modelled as a current-controlled servo (name ``md01i``).

    The AT32 driver board runs a position loop whose output is a *current*
    setpoint for an inner current loop (``setpoint_cur_mA`` in its live
    values). On the servo-3 bench recordings (``data_md01-3``) the measured
    motor current is a function of ``kp * error`` alone — the same curve for
    every ``kp`` from 60 to 140 — roughly linear at ~30 mA per rad of error
    per unit of ``kp`` and saturating at ~440 mA. That is the
    :class:`~bam.actuator.CurrentControlledActuator` structure:

    ``i = clip(kp * error_gain * error_gain_ratio * (q_target - q), ±current_limit)``,
    ``tau = kt * i``

    with ``error_gain_ratio`` and ``current_limit`` fitted. Fitted this way the
    simulated current reproduces the recorded one (which the fit never sees),
    whereas the voltage-controlled :class:`MD01Actuator` needs 3–10x the real
    current to match the same angles. Identification MAE on ``data_md01-3``:
    24–29 mrad here vs 50 mrad (voltage law, 0.45 A cap) vs 90 mrad (as
    originally committed).

    The fitted ``current_limit`` (~0.45 A) is the bench configuration of the
    current loop, not the motor's rating: the robot firmware presets reach
    ~0.9 A. Re-identify if the AT32 current-loop gains change.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(
            testbench_class,
            vin=12.0,
            kp=80.0,
            # A per rad of error per unit kp; measured on servo 3 in the
            # linear region of the I(kp*error) curve at |dq| < 0.3 rad/s.
            error_gain=0.030,
        )

    def initialize(self):
        # Torque constant at the output [Nm/A]; bench stall gives ~0.5.
        self.model.kt = Parameter(0.5, 0.05, 2.0)

        # Effective resistance [Ohm]; only bounds the current at speed here.
        self.model.R = Parameter(10.0, 2.0, 40.0)

        # Apparent inertia at the output [kg m^2].
        self.model.armature = Parameter(3e-4, 1e-5, 5e-3)

        # Sustained current limit of the current loop as configured [A].
        self.model.current_limit = Parameter(0.45, 0.2, 1.0)

        # Scale on error_gain, so the measured 0.030 is only a starting point.
        self.model.error_gain_ratio = Parameter(1.0, 0.3, 3.0)

    def compute_control(self, q_target, q, dq, dt):
        current = (
            (q_target - q)
            * self.kp
            * self.error_gain
            * self.model.error_gain_ratio.value
        )

        # What the supply can drive against the back-EMF
        current_high = (self.vin - self.model.kt.value * dq) / self.model.R.value
        current_low = (-self.vin - self.model.kt.value * dq) / self.model.R.value
        current = self.backend.clamp(current, current_low, current_high)

        limit = self.model.current_limit.value
        return self.backend.clamp(current, -limit, limit)

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
