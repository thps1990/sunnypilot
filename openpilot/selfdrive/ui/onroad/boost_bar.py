import pyray as rl
from openpilot.selfdrive.controls.lib.accel_boost import ACCEL_BOOST_MAX
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.widgets import Widget


class BoostBar(Widget):
  def _render(self, rect: rl.Rectangle):
    boost = 0.0
    if ui_state.engaged and ui_state.sm.seen['longitudinalPlan']:
      boost = ui_state.sm['longitudinalPlan'].accelBoost

    # Only show in experimental mode or if boost is currently active
    is_exp = ui_state.sm.seen['selfdriveState'] and ui_state.sm['selfdriveState'].experimentalMode
    if not ui_state.engaged or not (is_exp or boost > 0.0):
      return

    bar_width = 24
    bar_height = 420
    bar_x = rect.x + rect.width - bar_width - 24
    bar_y = rect.y + 260
    bar_rect = rl.Rectangle(bar_x, bar_y, bar_width, bar_height)

    # Background pill
    rl.draw_rectangle_rounded(bar_rect, 1.0, 10, rl.Color(40, 40, 40, 180))
    rl.draw_rectangle_rounded_lines_ex(bar_rect, 1.0, 10, 2, rl.Color(255, 255, 255, 60))

    # Active boost fill
    if boost > 0.0:
      fill_height = bar_rect.height * min(1.0, max(0.0, boost / ACCEL_BOOST_MAX))
      fill_rect = rl.Rectangle(bar_rect.x, bar_rect.y + bar_rect.height - fill_height, bar_width, fill_height)
      rl.draw_rectangle_rounded(fill_rect, 1.0, 10, rl.Color(0, 255, 204, 255))
