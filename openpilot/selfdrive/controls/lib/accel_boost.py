import numpy as np
from openpilot.common.realtime import DT_MDL

ACCEL_BOOST_MAX = 0.2
ACCEL_BOOST_RATE = 0.025
ACCEL_BOOST_PER_OVERRIDE = 0.05


class AccelBoost:
  def __init__(self, dt=DT_MDL):
    self.dt = dt
    self.value = 0.0
    self.override_boost = 0.0

  def update(self, enabled, gas_pressed, model_limited, active=None):
    if active is None:
      active = enabled

    if not enabled or not gas_pressed:
      self.override_boost = 0.0

    if not active:
      self.value = 0.0
    elif enabled and gas_pressed and model_limited:
      increase = min(ACCEL_BOOST_RATE * self.dt, ACCEL_BOOST_PER_OVERRIDE - self.override_boost, ACCEL_BOOST_MAX - self.value)
      self.value += increase
      self.override_boost += increase

  def apply(self, accel):
    return accel + np.interp(accel, [-1.0, -0.5, 5.0], [0.0, self.value, self.value], right=0.0)
