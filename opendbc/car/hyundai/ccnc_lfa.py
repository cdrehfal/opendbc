"""
LFA_ALT + ccNC (HDA1 cars with the ccNC cluster, e.g. 2025+ Ioniq 5 without HDA II).

openpilot replaces the camera's steering command (LFA_ALT, 0xCB) and gives the camera a copy of the rack
status that agrees with the camera's own request, so the camera stays healthy. A healthy camera turns its
own LFA on with the LKA button and with cruise MAIN. It isn't steering (its 0xCB never reaches the rack),
but it still runs the stock hands-on timer, and at the end of that escalation it cancels cruise and turns
MAIN off, which also ends openpilot lateral.

The camera LFA has no settings-menu switch, only the wheel button. So whenever the camera reports its LFA
as on (standby or active), press the LFA button for it, the way a driver would switch it off: held for
~0.2 s with the alive counter advancing, because the camera times the press (it distinguishes a short press
from a > 2 s hold), so a one-shot burst of identical frames may not register.
Only the camera bus sees the press; openpilot reads the LKA button from the car bus, so lateral is untouched.
"""
from opendbc.car import DT_CTRL

LFA_ICON_STANDBY = 1
LFA_ICON_ACTIVE = 2

ON_TIME = 1.0          # camera LFA seen on this long before pressing (its off transition blinks for ~1 s)
PRESS_TIME = 0.2       # hold the button this long: a real short press is ~0.15 s; the camera's long press is > 2 s
RETRY_TIME = 3.0       # wait this long for the camera to react before pressing again
SLOW_RETRY_TIME = 15.0  # after MAX_TRIES quick presses, keep trying at this rate instead of giving up
USER_BUTTON_QUIET = 0.5  # don't press within this long of the driver touching any wheel button
MAX_TRIES = 5          # quick presses without the camera going off before dropping to the slow rate
COPIES_PER_FRAME = 2   # frames sent per 10 ms control loop while the button is held (the car's own stream is 50 Hz)


class CameraLfaOff:
  def __init__(self):
    self.on_frames = 0
    self.off_frames = 0
    self.press_start = None
    self.last_press_frame = None
    self.last_user_button_frame = None
    self.tries = 0

  def update(self, frame: int, cam_lfa_icon: int, user_button_pressed: bool) -> bool:
    """Returns True on every frame the LFA button should be held down."""
    if user_button_pressed:
      self.last_user_button_frame = frame
      self.press_start = None  # the driver is on the buttons: let go

    if cam_lfa_icon in (LFA_ICON_STANDBY, LFA_ICON_ACTIVE):
      self.on_frames += 1
      self.off_frames = 0
    else:
      self.on_frames = 0
      self.off_frames += 1
      # the camera LFA is off (not just blinking through a handback): presses are working, reset the budget
      if cam_lfa_icon == 0 and self.off_frames * DT_CTRL >= ON_TIME:
        self.tries = 0

    # a press in progress: keep holding until PRESS_TIME, then release
    if self.press_start is not None:
      if (frame - self.press_start) * DT_CTRL < PRESS_TIME:
        return True
      self.press_start = None
      return False

    if self.on_frames * DT_CTRL < ON_TIME:
      return False
    retry = RETRY_TIME if self.tries < MAX_TRIES else SLOW_RETRY_TIME
    if self.last_press_frame is not None and (frame - self.last_press_frame) * DT_CTRL < retry:
      return False
    if self.last_user_button_frame is not None and (frame - self.last_user_button_frame) * DT_CTRL < USER_BUTTON_QUIET:
      return False

    self.press_start = frame
    self.last_press_frame = frame
    self.tries += 1
    return True
