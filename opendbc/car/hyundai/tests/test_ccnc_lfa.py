from opendbc.car import DT_CTRL
from opendbc.car.hyundai.ccnc_lfa import CameraLfaOff, ON_TIME, RETRY_TIME, USER_BUTTON_QUIET, MAX_TRIES

ON_FRAMES = int(ON_TIME / DT_CTRL)
RETRY_FRAMES = int(RETRY_TIME / DT_CTRL)


def run(c, frames, icon, start=0, user_button=False):
  return [f for f in range(start, start + frames) if c.update(f, icon, user_button)]


class TestCameraLfaOff:
  def test_off_never_taps(self):
    c = CameraLfaOff()
    assert run(c, 10 * ON_FRAMES, 0) == []

  def test_blink_never_taps(self):
    # handback blink (3/0) is not "on"
    c = CameraLfaOff()
    taps = [f for f in range(2000) if c.update(f, 3 if (f // 25) % 2 else 0, False)]
    assert taps == []

  def test_taps_once_after_on_time(self):
    for icon in (1, 2):
      c = CameraLfaOff()
      taps = run(c, ON_FRAMES + 5, icon)
      assert taps == [ON_FRAMES - 1]

  def test_short_on_ignored(self):
    c = CameraLfaOff()
    assert run(c, ON_FRAMES - 10, 2) == []
    assert run(c, 50, 0, start=ON_FRAMES) == []

  def test_retries_then_gives_up(self):
    c = CameraLfaOff()
    taps = run(c, ON_FRAMES + RETRY_FRAMES * (MAX_TRIES + 3), 2)
    assert len(taps) == MAX_TRIES
    assert all(b - a >= RETRY_FRAMES for a, b in zip(taps, taps[1:], strict=False))

  def test_budget_resets_when_camera_goes_off(self):
    c = CameraLfaOff()
    frame = 0
    for _ in range(3):
      taps = run(c, ON_FRAMES + RETRY_FRAMES * (MAX_TRIES + 1), 2, start=frame)
      assert len(taps) == MAX_TRIES
      frame += ON_FRAMES + RETRY_FRAMES * (MAX_TRIES + 1)
      run(c, 2 * ON_FRAMES, 0, start=frame)
      frame += 2 * ON_FRAMES

  def test_waits_for_driver_buttons(self):
    c = CameraLfaOff()
    # driver holding a button the whole time: never tap
    assert run(c, 3 * ON_FRAMES, 2, user_button=True) == []
    # released: tap only after the quiet time
    start = 3 * ON_FRAMES
    taps = run(c, ON_FRAMES, 2, start=start)
    assert taps == [start - 1 + round(USER_BUTTON_QUIET / DT_CTRL)]
