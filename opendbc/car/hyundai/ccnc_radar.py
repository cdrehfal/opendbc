"""
Cars in the neighbouring lanes on the ccNC cluster (LFA_ALT, HDA1 cars with the ccNC cluster, e.g. 2025+ Ioniq 5).

The cluster's driver-assistance view draws up to six vehicles from CCNC_0x162: the lead, a second lead, one ahead
in the left and one in the right lane, and one behind-left and behind-right. On HDA2 cars the ADAS ECU fills all six
from the front radar, the corner radars and the camera. On HDA1 cars the camera fills only the lead and leaves the
other five at zero, so the cluster shows nothing beside you.

The front radar still reports its 32 tracked objects on A-CAN (0x3A5-0x3C4; layout reverse-engineered by royjr,
sunnypilot/opendbc ccnc-port-radar-tracks). openpilot already receives them on this harness, so it can fill:
  - LEAD_LEFT / LEAD_RIGHT: the nearest moving car ahead in the lane to the left / right, placed in lanes using the
    camera's own lane lines (position, heading, curvature) so a car in our lane on a bend isn't put in the next lane
  - LEAD_LEFT_REAR / LEAD_RIGHT_REAR: the corner radars only report "blind spot occupied" (BCW), not a position, so a
    car is drawn at a fixed distance while the blind-spot indicator is on

Display only. The tracks are decoded here, outside the CAN parsers, so a missing or malformed radar frame can never
make CAN invalid (that would drop openpilot); with no radar data the side slots simply stay empty. The lead and the
second-lead slots are left exactly as the camera sent them.
"""
from opendbc.can.dbc import Signal
from opendbc.can.parser import get_raw_value
from opendbc.car.carlog import carlog

RADAR_TRACK_START = 0x3A5
RADAR_TRACK_COUNT = 32
RADAR_TRACK_ADDRS = range(RADAR_TRACK_START, RADAR_TRACK_START + RADAR_TRACK_COUNT)
RADAR_TRACK_SIZE = 24

STATE_MEASURED, STATE_COASTED = 3, 4
MOTION_MOVING = 2

TRACK_STALE = 0.3          # s without an update before a track is dropped
MOVING_MEMORY = 3.0        # s a track keeps counting as a car after it was last seen moving (slowing to a stop beside us)
ONCOMING_SPEED = -0.5      # m/s absolute speed below this is oncoming traffic
MAX_DIST = 80.             # m with the camera's lane lines
MAX_DIST_NO_LANES = 40.    # m without them (straight-line lanes only)
CONFIRM_TIME = 0.1         # s a lane must have a car before it is drawn
HOLD_TIME = 0.5            # s a drawn car is kept after the lane looks empty (radar dropouts, lane-edge flicker)
# The camera's own lead (our lane) is never also drawn beside us: a track within this distance of it and less than
# LEAD_GUARD_MARGIN past our lane's edge is taken to be the lead, mis-placed by the lane fit (mostly beyond 50 m).
# A real car beside the lead sits a whole lane over and is kept.
LEAD_GUARD_DIST = 2.5
LEAD_GUARD_MARGIN = 1.0

DEFAULT_LANE_WIDTH = 3.6
MIN_LANE_WIDTH, MAX_LANE_WIDTH = 2.8, 4.5
# The radar's lateral zero sits ~0.3 m right of the camera's (Oct 6 drives: own-lane lead 0.29 m right of the camera's
# lane centre at every distance). Added to the radar's lateral so both use the camera's frame.
RADAR_LATERAL_OFFSET = 0.29

# What the cluster is sent. Side cars get the lane-centre lateral HDA2 sends (royjr: 3.5 m in both side slots); the
# blind-spot cars a fixed distance, since the corner radars give no position. Status 1 = a vehicle (2 is the camera's
# highlighted SCC target, only used for the lead).
SIDE_STATUS = 1
SIDE_LATERAL = 3.5
REAR_DISTANCE = 3.0
MAX_SIDE_DISTANCE = 204.7
SLOTS = ("LEFT", "RIGHT", "LEFT_REAR", "RIGHT_REAR")


def _sig(name: str, start_bit: int, size: int, little_endian: bool, signed: bool = False, factor: float = 1.) -> Signal:
  # same bit math as opendbc.can.dbc.DBC._parse_lines
  if little_endian:
    lsb, msb = start_bit, start_bit + size - 1
  else:
    be_bits = [j + i * 8 for i in range(64) for j in range(7, -1, -1)]
    idx = be_bits.index(start_bit)
    lsb, msb = be_bits[idx + size - 1], start_bit
  return Signal(name, start_bit, msb, lsb, size, signed, factor, 0., little_endian)


# the signals used from the 3A5-3C4 track layout (royjr, hyundai_radar_3a5_3c4.py)
TRACK_SIGNALS = {
  "MOTION_STATE": _sig("MOTION_STATE", 28, 2, False),
  "STATE": _sig("STATE", 54, 3, False),
  "LONG_DIST": _sig("LONG_DIST", 63, 13, True, factor=0.05),
  "LAT_DIST": _sig("LAT_DIST", 76, 12, True, signed=True, factor=0.05),     # m, left positive
  "REL_SPEED": _sig("REL_SPEED", 88, 14, True, signed=True, factor=0.01),   # m/s
}


def _value(dat: bytes | bytearray, sig: Signal) -> float:
  v = get_raw_value(dat, sig)
  if sig.is_signed:
    v -= ((v >> (sig.size - 1)) & 0x1) * (1 << sig.size)
  return v * sig.factor


def decode_track(dat: bytes | bytearray) -> dict[str, float]:
  return {name: _value(dat, sig) for name, sig in TRACK_SIGNALS.items()}


def lane_model(msg_1b5: dict) -> tuple[float, float, float, float] | None:
  """(centre, heading, curvature term, width) of our lane from the camera's lane lines, left positive, so that the
  lane centre at distance x is centre + heading * x + curvature * x^2. None if either line isn't tracked well.

  The camera reports line positions and curvature right positive but heading left positive (signs and the factor of
  one half fitted against the own-lane lead on the Oct 6 drives: 0.2-0.3 m rms within 50 m, the other combinations
  0.9-2.0 m).
  """
  if not msg_1b5 or msg_1b5.get("Info_LftLnQualSta") not in (2, 3) or msg_1b5.get("Info_RtLnQualSta") not in (2, 3):
    return None
  left, right = msg_1b5["Info_LftLnPosVal"], msg_1b5["Info_RtLnPosVal"]
  width = right - left
  if not MIN_LANE_WIDTH <= width <= MAX_LANE_WIDTH:
    return None
  centre = -(left + right) / 2.
  heading = (msg_1b5["Info_LftLnHdingAnglVal"] + msg_1b5["Info_RtLnHdingAnglVal"]) / 2.
  curvature = -(msg_1b5["Info_LftLnCvtrVal"] + msg_1b5["Info_RtLnCvtrVal"]) / 4.
  return centre, heading, curvature, width


class ClusterRadar:
  def __init__(self, bus: int):
    self.bus = bus
    self.tracks: dict[int, tuple[int, dict[str, float]]] = {}   # addr -> (nanos, decoded)
    self.last_moving: dict[int, int] = {}                        # addr -> nanos last seen moving
    self.now = 0
    self.first_seen: dict[str, int | None] = {"LEFT": None, "RIGHT": None}
    self.shown: dict[str, tuple[int, float] | None] = {"LEFT": None, "RIGHT": None}  # (nanos last confirmed, distance)
    self.failed = False

  def update_frames(self, can_packets) -> None:
    """Keep the latest frame of each radar track. Called with every batch of CAN packets, before the parsers."""
    if self.failed:
      return
    try:
      for t, frames in can_packets:
        self.now = max(self.now, t)
        for address, dat, src in frames:
          if src == self.bus and RADAR_TRACK_START <= address < RADAR_TRACK_START + RADAR_TRACK_COUNT and len(dat) >= RADAR_TRACK_SIZE:
            track = decode_track(dat)
            self.tracks[address] = (t, track)
            if int(track["MOTION_STATE"]) == MOTION_MOVING:
              self.last_moving[address] = t
    except Exception:
      carlog.exception("ccNC cluster radar: decoding failed, side cars off")
      self.failed = True

  def _candidates(self, v_ego: float, lane, lead_dist: float) -> dict[str, float]:
    """nearest car distance per side lane"""
    if lane is None:
      centre, heading, curvature, width, max_dist = 0., 0., 0., DEFAULT_LANE_WIDTH, MAX_DIST_NO_LANES
    else:
      (centre, heading, curvature, width), max_dist = lane, MAX_DIST

    nearest: dict[str, float] = {}
    for address, (t, tr) in self.tracks.items():
      if (self.now - t) * 1e-9 > TRACK_STALE or int(tr["STATE"]) not in (STATE_MEASURED, STATE_COASTED):
        continue
      last_moving = self.last_moving.get(address)
      if last_moving is None or (self.now - last_moving) * 1e-9 > MOVING_MEMORY:
        continue  # stationary objects: signs, barriers, parked cars
      if v_ego + tr["REL_SPEED"] < ONCOMING_SPEED:
        continue
      x = tr["LONG_DIST"]
      if not 0. <= x <= max_dist:
        continue
      offset = tr["LAT_DIST"] + RADAR_LATERAL_OFFSET - (centre + heading * x + curvature * x * x)
      side = "LEFT" if width / 2. < offset < 1.5 * width else "RIGHT" if -1.5 * width < offset < -width / 2. else None
      if side is not None and lead_dist > 0. and abs(x - lead_dist) < LEAD_GUARD_DIST and abs(offset) < width / 2. + LEAD_GUARD_MARGIN:
        continue
      if side is not None and x < nearest.get(side, float("inf")):
        nearest[side] = x
    return nearest

  def update(self, v_ego: float, msg_1b5: dict, left_blindspot: bool, right_blindspot: bool) -> dict[str, float]:
    """CCNC_0x162 signal values for the four side slots, at the cluster rate."""
    values: dict[str, float] = {}
    for slot in SLOTS:
      status = f"LEAD_{slot}_STATUS" if slot.endswith("REAR") else f"LEAD_{slot}"
      values.update({status: 0, f"LEAD_{slot}_DISTANCE": 0., f"LEAD_{slot}_LATERAL": 0.})

    if not self.failed:
      try:
        lead_dist = float(msg_1b5.get("Longitudinal_Distance", 0.)) if msg_1b5 else 0.
        nearest = self._candidates(v_ego, lane_model(msg_1b5), lead_dist)
        for side in ("LEFT", "RIGHT"):
          if side in nearest:
            if self.first_seen[side] is None:
              self.first_seen[side] = self.now
            if (self.now - self.first_seen[side]) * 1e-9 >= CONFIRM_TIME or self.shown[side] is not None:
              self.shown[side] = (self.now, nearest[side])
          else:
            self.first_seen[side] = None
          shown = self.shown[side]
          if shown is not None and (self.now - shown[0]) * 1e-9 > HOLD_TIME:
            self.shown[side] = shown = None
          if shown is not None:
            values.update({f"LEAD_{side}": SIDE_STATUS, f"LEAD_{side}_DISTANCE": min(shown[1], MAX_SIDE_DISTANCE),
                           f"LEAD_{side}_LATERAL": SIDE_LATERAL})
      except Exception:
        carlog.exception("ccNC cluster radar: selection failed, side cars off")
        self.failed = True
        for side in ("LEFT", "RIGHT"):
          values.update({f"LEAD_{side}": 0, f"LEAD_{side}_DISTANCE": 0., f"LEAD_{side}_LATERAL": 0.})

    for side, occupied in (("LEFT", left_blindspot), ("RIGHT", right_blindspot)):
      if occupied:
        values.update({f"LEAD_{side}_REAR_STATUS": SIDE_STATUS, f"LEAD_{side}_REAR_DISTANCE": REAR_DISTANCE,
                       f"LEAD_{side}_REAR_LATERAL": SIDE_LATERAL})
    return values
