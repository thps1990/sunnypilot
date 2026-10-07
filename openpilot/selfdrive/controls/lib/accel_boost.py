import numpy as np
from openpilot.common.realtime import DT_MDL

ACCEL_BOOST_MAX = 0.6
ACCEL_BOOST_RATE = 0.4
ACCEL_BOOST_TAP = 0.15
ACCEL_BOOST_PER_OVERRIDE = 0.3


class AccelBoost:
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.value = 0.0
    self.override_boost = 0.0
    self.gas_pressed_prev = False

  def update(self, enabled, gas_pressed, model_limited, is_e2e=True, active=None, v_ego=0.0):
    if active is None:
      active = enabled

    # Reset on disengage, standstill, or non-E2E mode
    if not active or not enabled or not is_e2e or v_ego < 0.5:
      self.value = 0.0
      self.override_boost = 0.0
      self.gas_pressed_prev = gas_pressed
      return

    if not gas_pressed:
      self.override_boost = 0.0
    elif enabled and model_limited:
      # Initial boost step on rising edge of gas pedal
      if not self.gas_pressed_prev:
        initial_bump = min(ACCEL_BOOST_TAP, ACCEL_BOOST_PER_OVERRIDE - self.override_boost, ACCEL_BOOST_MAX - self.value)
        self.value += initial_bump
        self.override_boost += initial_bump
      else:
        increase = min(ACCEL_BOOST_RATE * self.dt, ACCEL_BOOST_PER_OVERRIDE - self.override_boost, ACCEL_BOOST_MAX - self.value)
        self.value += increase
        self.override_boost += increase

    self.gas_pressed_prev = gas_pressed

  def apply(self, accel):
    return accel + np.interp(accel, [-1.0, -0.5, 5.0], [0.0, self.value, self.value], right=0.0)
