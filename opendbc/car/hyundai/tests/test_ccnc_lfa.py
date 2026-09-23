from opendbc.car import DT_CTRL
from opendbc.car.hyundai.ccnc_lfa import CameraLfaOff, ON_TIME, PRESS_TIME, RETRY_TIME, SLOW_RETRY_TIME, USER_BUTTON_QUIET, MAX_TRIES

ON_FRAMES = int(ON_TIME / DT_CTRL)
PRESS_FRAMES = int(PRESS_TIME / DT_CTRL)
RETRY_FRAMES = int(RETRY_TIME / DT_CTRL)
SLOW_FRAMES = int(SLOW_RETRY_TIME / DT_CTRL)


def run(c, frames, icon, start=0, user_button=False):
  return [f for f in range(start, start + frames) if c.update(f, icon, user_button)]


def presses(held):
  """group consecutive held frames into (start, length)"""
  out = []
  for f in held:
    if out and f == out[-1][0] + out[-1][1]:
      out[-1][1] += 1
    else:
      out.append([f, 1])
  return [tuple(p) for p in out]


class TestCameraLfaOff:
  def test_off_never_presses(self):
    c = CameraLfaOff()
    assert run(c, 10 * ON_FRAMES, 0) == []

  def test_blink_never_presses(self):
    # handback blink (3/0) is not "on"
    c = CameraLfaOff()
    held = [f for f in range(2000) if c.update(f, 3 if (f // 25) % 2 else 0, False)]
    assert held == []

  def test_presses_once_held_for_press_time(self):
    for icon in (1, 2):
      c = CameraLfaOff()
      p = presses(run(c, ON_FRAMES + PRESS_FRAMES + 5, icon))
      assert p == [(ON_FRAMES - 1, PRESS_FRAMES)]

  def test_short_on_ignored(self):
    c = CameraLfaOff()
    assert run(c, ON_FRAMES - 10, 2) == []
    assert run(c, 50, 0, start=ON_FRAMES) == []

  def test_quick_retries_then_slow(self):
    c = CameraLfaOff()
    n = ON_FRAMES + RETRY_FRAMES * MAX_TRIES + SLOW_FRAMES * 3
    p = presses(run(c, n, 2))
    assert len(p) == MAX_TRIES + 3
    starts = [s for s, _ in p]
    gaps = [b - a for a, b in zip(starts, starts[1:], strict=False)]
    assert all(g >= RETRY_FRAMES for g in gaps[:MAX_TRIES - 1])
    assert all(g >= SLOW_FRAMES for g in gaps[MAX_TRIES - 1:])
    assert all(length == PRESS_FRAMES for _, length in p)

  def test_budget_resets_when_camera_goes_off(self):
    c = CameraLfaOff()
    frame = 0
    for _ in range(3):
      p = presses(run(c, ON_FRAMES + RETRY_FRAMES * MAX_TRIES, 2, start=frame))
      assert len(p) == MAX_TRIES
      frame += ON_FRAMES + RETRY_FRAMES * MAX_TRIES
      run(c, 2 * ON_FRAMES, 0, start=frame)
      frame += 2 * ON_FRAMES

  def test_waits_for_driver_buttons(self):
    c = CameraLfaOff()
    # driver holding a button the whole time: never press
    assert run(c, 3 * ON_FRAMES, 2, user_button=True) == []
    # released: press only after the quiet time
    start = 3 * ON_FRAMES
    p = presses(run(c, ON_FRAMES, 2, start=start))
    assert p == [(start - 1 + round(USER_BUTTON_QUIET / DT_CTRL), PRESS_FRAMES)]

  def test_driver_button_aborts_press(self):
    c = CameraLfaOff()
    run(c, ON_FRAMES + 3, 2)  # press in progress
    assert c.update(ON_FRAMES + 3, 2, True) is False
    assert run(c, 10, 2, start=ON_FRAMES + 4) == []


class TestCcncLaneChangeIcon:
  # the dash lane-change graphics follow openpilot's lane-change availability: >= 50 mph and not switched off
  def _cc_sp(self, timer=None):
    from opendbc.car import structs
    cc_sp = structs.CarControlSP()
    if timer is not None:
      cc_sp.params = [structs.CarControlSP.Param(key="AutoLaneChangeTimer", value=str(timer).encode(), type="int")]
    return cc_sp

  def test_available(self):
    from opendbc.car.common.conversions import Conversions as CV
    from opendbc.car.hyundai.hyundaicanfd import lane_change_available
    assert not lane_change_available(49 * CV.MPH_TO_MS, self._cc_sp(3))
    assert lane_change_available(51 * CV.MPH_TO_MS, self._cc_sp(3))
    assert lane_change_available(51 * CV.MPH_TO_MS, self._cc_sp(0))
    assert not lane_change_available(70 * CV.MPH_TO_MS, self._cc_sp(-1))
    assert lane_change_available(70 * CV.MPH_TO_MS, self._cc_sp())  # param not passed: behave as before

  def _icons(self, available, left_blinker=True):
    from types import SimpleNamespace as NS
    from opendbc.can import CANPacker, CANParser
    from opendbc.car import gen_empty_fingerprint
    from opendbc.car.hyundai.hyundaicanfd import CanBus, create_ccnc
    packer = CANPacker("hyundai_canfd_generated")
    parser = CANParser("hyundai_canfd_generated", [("CCNC_0x161", 20)], 0)
    names = [s.name for s in packer.dbc.addr_to_msg[0x161].sigs.values()] if hasattr(packer, "dbc") else []
    msg_161 = dict.fromkeys(names, 0) if names else {}
    msg_162 = {}
    msg_1b5 = {"Info_LftLnPosVal": -1.7, "Info_RtLnPosVal": 1.7, "Info_LftLnQualSta": 3, "Info_RtLnQualSta": 3,
               "Longitudinal_Distance": 0}
    hud = NS(leftLaneVisible=True, rightLaneVisible=True, leftLaneDepart=False, rightLaneDepart=False, leadDistanceBars=2,
             leadVisible=False)
    out = NS(steeringAngleDeg=0.0, vEgo=30.0, leftBlindspot=False, rightBlindspot=False, vCruiseCluster=0)
    msgs = create_ccnc(packer, CanBus(None, gen_empty_fingerprint()), False, True, hud, left_blinker, False, msg_161, msg_162,
                       msg_1b5, False, out, True, 2, available)
    parser.update([(0, [(m[0], m[1], 0) for m in msgs])])
    return parser.vl["CCNC_0x161"]

  def test_icon_hidden_when_unavailable(self):
    v = self._icons(False)
    assert v["LCA_LEFT_ICON"] == 0 and v["LCA_RIGHT_ICON"] == 0 and v["LCA_LEFT_ARROW"] == 0
    assert v["LANELINE_LEFT"] == 2  # plain lane line, not the lane-change style

  def test_icon_shown_when_available(self):
    v = self._icons(True)
    assert v["LCA_LEFT_ICON"] == 2 and v["LCA_LEFT_ARROW"] == 2
    assert v["LANELINE_LEFT"] == 6

  def test_no_white_available_icon(self):
    # available but no blinker and nothing in the blind spot: icon hidden, never code 4
    v = self._icons(True, left_blinker=False)
    assert v["LCA_LEFT_ICON"] == 0 and v["LCA_RIGHT_ICON"] == 0
