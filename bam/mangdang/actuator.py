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

        ``max_current`` and the kt / R / armature ranges below come from the
        servo-3 bench recordings (``data_md01-3``), not from a datasheet;
        ``error_gain`` and ``max_pwm`` are still inherited from the Feetech
        STS3215 and have not been measured on the MD01.
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
            # Effective current limit [A]. The recorder sends a 900 mA cap in
            # the position frame, but the current the board actually sustains
            # in data_md01-2/3 plateaus at ~440 mA (0.171 kg on 0.15 m stalls
            # there for seconds; transients reach ~590 mA). This is the limit
            # the pendulum sees, so it is the one the model must saturate at.
            # It is a property of the AT32 current loop as configured on the
            # bench (robot presets reach ~900 mA) — re-measure if that changes.
            max_current=0.45,
        )

    def initialize(self):
        # Torque constant [Nm/A] or [V/(rad/s)]. Stall on the bench
        # (0.20 Nm at ~440 mA) gives ~0.45 at the output; bounds leave room
        # for the fit to trade it against R.
        self.model.kt = Parameter(0.5, 0.05, 2.0)

        # Effective resistance [Ohm], output side. Fits on data_md01-3 pinned
        # the previous 7.5–9.5 range at its lower bound, so keep it wide.
        self.model.R = Parameter(10.0, 2.0, 40.0)

        # Rotor / apparent inertia at the output [kg m^2]; earlier fits sat on
        # the 1.1e-4 floor of the old range.
        self.model.armature = Parameter(3e-4, 1e-5, 5e-3)

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


class MD01LoopActuator(CurrentControlledActuator):
    """MD01 with the AT32's measured position/current loops (name ``md01c``).

    Measured at a blocked output on 2026-09-21 (servo 6, three gain sets,
    ``data_md01-6v2_raw/at32_stall_test.csv``):

    ``i_set = clip(kp_position * error_deg, +/-cap)``  [mA] and
    ``duty = clip(kp_current * (i_set - i) + kff_current * i_set, +/-max_pwm)``,

    both exact to three digits, with the plant at stall
    ``i = (duty * vin - V0) / R`` (``V0`` ~ 1.2 V, ``R`` ~ 11 ohm at the
    output). Solved algebraically per step with the back-EMF ``kt * dq`` in
    the plant, which gives the current the loop settles to; BAM's
    ``command_delay`` absorbs the loop's few-ms rise. The firmware constants
    come from the log (``kp``, ``at32_kp_current``, ``at32_kff_current``,
    ``at32_max_pwm_duty_cycle``, ``cur_cap_ma``), so a change of preset needs
    no refit; ``kp_ratio`` is a check parameter expected to fit near 1.
    """

    def __init__(self, testbench_class: Testbench):
        super().__init__(testbench_class, vin=12.0, kp=80.0, error_gain=1.0)
        # Firmware constants (flash values read on 2026-09-21); load_log
        # overrides them from the log metadata when present.
        self.kp_current = 6e-4
        self.kff_current = 3e-4
        self.max_pwm = 0.99
        self.cap_ma = 1500.0

    def load_log(self, log: dict):
        super().load_log(log)
        self.kp_current = log.get("at32_kp_current", self.kp_current)
        self.kff_current = log.get("at32_kff_current", self.kff_current)
        self.max_pwm = log.get("at32_max_pwm_duty_cycle", self.max_pwm)
        self.cap_ma = log.get("cur_cap_ma", self.cap_ma)

    def initialize(self):
        # Torque constant at the output [Nm/A].
        self.model.kt = Parameter(0.6, 0.05, 2.0)
        # The plant is measured, not fitted: a position-only fit cannot pin
        # the current scale (kt trades against R and V0 - with free ranges
        # the 2026-09-21 fits went to R = 2, V0 = 0, kt / 3 and a current
        # 3x too high). The ranges are the stall-test spread: R_eff 11-13.4
        # ohm cold to warm, V0 1.2 V, kp_ratio 1.00 to 1 %.
        self.model.R = Parameter(11.5, 10.0, 14.0)
        self.model.V0 = Parameter(1.2, 1.0, 1.4)
        # Apparent inertia at the output [kg m^2].
        self.model.armature = Parameter(3e-4, 1e-5, 5e-3)
        # Scale on the measured mA/deg position gain.
        self.model.kp_ratio = Parameter(1.0, 0.95, 1.05)

    def compute_control(self, q_target, q, dq, dt):
        kt = self.model.kt.value
        R = self.model.R.value
        V0 = self.model.V0.value
        vin = self.vin

        # Position loop: current setpoint [A], capped by the frame's cap.
        cap = self.cap_ma / 1000.0
        i_set = (q_target - q) * (180.0 / 3.141592653589793) * self.kp * self.model.kp_ratio.value / 1000.0
        i_set = self.backend.clamp(i_set, -cap, cap)

        # Current loop (P + feed-forward, gains per mA -> per A) in steady
        # state with the plant i = (duty*vin - V0*sign - kt*dq)/R:
        #   i (R + vin kp_c) = vin (kp_c + kff) i_set - V0 sign(x) - kt dq
        kp_c = self.kp_current * 1000.0
        kff = self.kff_current * 1000.0
        x = vin * (kp_c + kff) * i_set - kt * dq
        sign = self.backend.sign(x)
        magnitude = self.backend.clamp(sign * x - V0, 0.0, float("inf"))
        current = sign * magnitude / (R + vin * kp_c)

        # What the bridge can actually apply: clip the duty and recompute.
        duty = (current * R + V0 * sign + kt * dq) / vin
        duty = self.backend.clamp(duty, -self.max_pwm, self.max_pwm)
        self.duty_cycle = duty
        return (duty * vin - V0 * sign - kt * dq) / R

    def get_extra_inertia(self) -> float:
        return self.model.armature.value
