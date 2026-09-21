"""
LFA_ALT + ccNC (HDA1 cars with the ccNC cluster, e.g. 2025+ Ioniq 5 without HDA II).

openpilot replaces the camera's steering command (LFA_ALT, 0xCB) and gives the camera a copy of the rack
status that agrees with the camera's own request, so the camera stays healthy. A healthy camera turns its
own LFA on with the LKA button and with cruise MAIN. It isn't steering (its 0xCB never reaches the rack),
but it still runs the stock hands-on timer, and at the end of that escalation it cancels cruise and turns
MAIN off, which also ends openpilot lateral.

The camera LFA has no settings-menu switch, only the wheel button. So whenever the camera reports its LFA
as on (standby or active), send it one bare LFA button tap, the same way a driver would switch it off.
Only the camera bus sees the tap; openpilot reads the LKA button from the car bus, so lateral is untouched.
"""
from opendbc.car import DT_CTRL

LFA_ICON_STANDBY = 1
LFA_ICON_ACTIVE = 2

ON_TIME = 1.0          # camera LFA seen on this long before tapping (its off transition blinks for ~1 s)
RETRY_TIME = 3.0       # wait this long for the camera to react before tapping again
USER_BUTTON_QUIET = 0.5  # don't tap within this long of the driver touching any wheel button
MAX_TRIES = 5          # consecutive taps without the camera going off, then stop until it goes off by itself
TAP_COPIES = 20        # same burst the cruise button spoofing uses; one burst reads as one press + release


class CameraLfaOff:
  def __init__(self):
    self.on_frames = 0
    self.off_frames = 0
    self.last_tap_frame = None
    self.last_user_button_frame = None
    self.tries = 0

  def update(self, frame: int, cam_lfa_icon: int, user_button_pressed: bool) -> bool:
    """Returns True on the frame a tap should be sent."""
    if user_button_pressed:
      self.last_user_button_frame = frame

    if cam_lfa_icon in (LFA_ICON_STANDBY, LFA_ICON_ACTIVE):
      self.on_frames += 1
      self.off_frames = 0
    else:
      self.on_frames = 0
      self.off_frames += 1
      # the camera LFA is off (not just blinking through a handback): taps are working, reset the budget
      if cam_lfa_icon == 0 and self.off_frames * DT_CTRL >= ON_TIME:
        self.tries = 0

    if self.on_frames * DT_CTRL < ON_TIME or self.tries >= MAX_TRIES:
      return False
    if self.last_tap_frame is not None and (frame - self.last_tap_frame) * DT_CTRL < RETRY_TIME:
      return False
    if self.last_user_button_frame is not None and (frame - self.last_user_button_frame) * DT_CTRL < USER_BUTTON_QUIET:
      return False

    self.last_tap_frame = frame
    self.tries += 1
    return True
