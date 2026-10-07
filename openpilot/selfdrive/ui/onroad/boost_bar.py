import pyray as rl
from openpilot.cereal import custom
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.accel_boost import ACCEL_BOOST_MAX
from openpilot.selfdrive.ui.sunnypilot.onroad.developer_ui import DeveloperUiState
from openpilot.selfdrive.ui.ui_state import ui_state
from openpilot.system.ui.lib.application import gui_app, FontWeight
from openpilot.system.ui.lib.text_measure import measure_text_cached
from openpilot.system.ui.widgets import Widget

DECState = custom.LongitudinalPlanSP.DynamicExperimentalControl.DynamicExperimentalControlState


class BoostBar(Widget):
  def __init__(self):
    super().__init__()
    self._boost_filter = FirstOrderFilter(0.0, 0.08, 1.0 / gui_app.target_fps)
    self._font_bold: rl.Font = gui_app.font(FontWeight.BOLD)
    self._font_semi_bold: rl.Font = gui_app.font(FontWeight.SEMI_BOLD)

  def _render(self, rect: rl.Rectangle):
    raw_boost = 0.0
    if ui_state.engaged and ui_state.sm.seen['longitudinalPlan']:
      raw_boost = ui_state.sm['longitudinalPlan'].accelBoost

    # Check whether experimental (e2e) mode is currently active
    is_exp = ui_state.sm.seen['selfdriveState'] and ui_state.sm['selfdriveState'].experimentalMode
    if ui_state.sm.seen['longitudinalPlanSP'] and ui_state.sm['longitudinalPlanSP'].dec.active:
      is_exp = is_exp and (ui_state.sm['longitudinalPlanSP'].dec.state == DECState.blended)

    # Only show in experimental mode or if boost is currently active
    if not ui_state.engaged or not (is_exp or raw_boost > 0.0):
      self._boost_filter.x = 0.0
      return

    filtered_boost = self._boost_filter.update(raw_boost)

    bar_width = 30
    bar_height = 400
    right_margin = 32
    if getattr(ui_state, 'developer_ui', DeveloperUiState.OFF) in (DeveloperUiState.RIGHT, DeveloperUiState.BOTH):
      right_margin += 190

    bar_x = rect.x + rect.width - bar_width - right_margin
    bar_y = rect.y + 260
    bar_rect = rl.Rectangle(bar_x, bar_y, bar_width, bar_height)

    # Background pill
    rl.draw_rectangle_rounded(bar_rect, 1.0, 10, rl.Color(30, 30, 30, 180))
    rl.draw_rectangle_rounded_lines_ex(bar_rect, 1.0, 10, 2, rl.Color(255, 255, 255, 60))

    # Active boost fill
    if filtered_boost > 0.005:
      fill_ratio = min(1.0, max(0.0, filtered_boost / ACCEL_BOOST_MAX))
      fill_height = bar_rect.height * fill_ratio
      fill_rect = rl.Rectangle(bar_rect.x, bar_rect.y + bar_rect.height - fill_height, bar_width, fill_height)
      rl.draw_rectangle_rounded(fill_rect, 1.0, 10, rl.Color(0, 255, 204, 240))

    # "BOOST" text header above bar
    label_text = "BOOST"
    label_size = 20
    label_width = measure_text_cached(self._font_bold, label_text, label_size, 0).x
    label_x = bar_x + (bar_width - label_width) / 2
    label_color = rl.Color(0, 255, 204, 255) if raw_boost > 0.0 else rl.Color(200, 200, 200, 180)
    rl.draw_text_ex(self._font_bold, label_text, rl.Vector2(label_x, bar_y - 26), label_size, 0, label_color)

    # Boost value display below bar when active
    if raw_boost > 0.005:
      val_text = f"+{raw_boost:.1f}"
      val_size = 22
      val_width = measure_text_cached(self._font_bold, val_text, val_size, 0).x
      val_x = bar_x + (bar_width - val_width) / 2
      rl.draw_text_ex(self._font_bold, val_text, rl.Vector2(val_x, bar_y + bar_height + 8), val_size, 0, rl.Color(0, 255, 204, 255))
