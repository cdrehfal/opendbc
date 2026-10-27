import random

from opendbc.can import CANPacker, CANParser
from opendbc.can.packer import set_value
from opendbc.car import gen_empty_fingerprint
from opendbc.car.hyundai import ccnc_radar as cr
from opendbc.car.hyundai.ccnc_radar import ClusterRadar, decode_track, lane_model, TRACK_SIGNALS, SIDE_LATERAL, REAR_DISTANCE
from opendbc.car.hyundai.hyundaicanfd import CanBus, create_ccnc
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, HyundaiFlags

BUS = 1
SEC = 1_000_000_000
STEP = SEC // 20  # cluster rate

# real tracks from the Oct 6 drive (route 33, segment 4)
REAL_FRAMES = [
  ("293318122535308881d2f8e800f8df0600000000d0020000", dict(MOTION_STATE=2, STATE=3, LONG_DIST=64.15, LAT_DIST=-5.75, REL_SPEED=2.32)),
  ("bef6181224ff30025542053a3fdbdf0528000000d0020000", dict(MOTION_STATE=2, STATE=3, LONG_DIST=59.7, LAT_DIST=4.2, REL_SPEED=-1.98)),
  ("f868181224463082f742fdc7bea5df0600000000d0020000", dict(MOTION_STATE=2, STATE=3, LONG_DIST=75.95, LAT_DIST=-2.2, REL_SPEED=-3.13)),
]

STRAIGHT = {"Info_LftLnPosVal": -1.8, "Info_RtLnPosVal": 1.8, "Info_LftLnQualSta": 3, "Info_RtLnQualSta": 3,
            "Info_LftLnHdingAnglVal": 0., "Info_RtLnHdingAnglVal": 0., "Info_LftLnCvtrVal": 0., "Info_RtLnCvtrVal": 0.,
            "Longitudinal_Distance": 0.}
NO_LANES = dict(STRAIGHT, Info_LftLnQualSta=0, Info_RtLnQualSta=0)


def track_frame(long_dist, lat, rel_speed=0., state=3, motion=2) -> bytes:
  dat = bytearray(24)
  for name, value in (("MOTION_STATE", motion), ("STATE", state), ("LONG_DIST", long_dist), ("LAT_DIST", lat), ("REL_SPEED", rel_speed)):
    sig = TRACK_SIGNALS[name]
    ival = int(round(value / sig.factor))
    if ival < 0:
      ival += 1 << sig.size
    set_value(dat, sig, ival)
  return bytes(dat)


class Sim:
  """feeds radar frames at 20 Hz and reads the side slots, like card does"""
  def __init__(self):
    self.r = ClusterRadar(BUS)
    self.t = SEC

  def step(self, tracks, msg_1b5=STRAIGHT, v_ego=25., lbsm=False, rbsm=False, n=1):
    out = None
    for _ in range(n):
      self.t += STEP
      frames = [(cr.RADAR_TRACK_START + i, track_frame(*tr), BUS) for i, tr in enumerate(tracks)]
      frames.append((0x1a0, bytes(32), 2))  # other traffic keeps the clock moving
      self.r.update_frames([(self.t, frames)])
      out = self.r.update(v_ego, msg_1b5, lbsm, rbsm)
    return out


def shown(out, side):
  return out[f"LEAD_{side}"] == cr.SIDE_STATUS, out[f"LEAD_{side}_DISTANCE"]


class TestDecode:
  def test_real_frames(self):
    for hexdat, expected in REAL_FRAMES:
      got = decode_track(bytes.fromhex(hexdat))
      for k, v in expected.items():
        assert abs(got[k] - v) < 1e-6, (k, got[k], v)

  def test_roundtrip(self):
    got = decode_track(track_frame(42.35, -3.4, -1.25, 4, 1))
    for k, v in (("LONG_DIST", 42.35), ("LAT_DIST", -3.4), ("REL_SPEED", -1.25), ("STATE", 4), ("MOTION_STATE", 1)):
      assert abs(got[k] - v) < 1e-9, (k, got[k])

  def test_malformed_and_foreign_frames_ignored(self):
    r = ClusterRadar(BUS)
    rng = random.Random(0)
    pkts = [(SEC, [(cr.RADAR_TRACK_START, b"\x01\x02", BUS),                      # short
                   (cr.RADAR_TRACK_START + 1, track_frame(30, 3.6), 0),          # other bus
                   (0x3C5, track_frame(30, 3.6), BUS)])]                         # outside the range
    pkts += [(SEC + i, [(cr.RADAR_TRACK_START + rng.randrange(32), bytes(rng.randrange(256) for _ in range(24)), BUS)]) for i in range(500)]
    r.update_frames(pkts[:1])
    assert r.tracks == {}
    r.update_frames(pkts[1:])
    assert not r.failed
    r.update(25., STRAIGHT, False, False)  # random data must never raise
    assert not r.failed


class TestLanes:
  def test_lane_model(self):
    centre, heading, curvature, width = lane_model(dict(STRAIGHT, Info_LftLnPosVal=-1.5, Info_RtLnPosVal=2.0))
    assert abs(width - 3.5) < 1e-9 and abs(centre - -0.25) < 1e-9  # lines right positive -> centre left positive
    assert lane_model(NO_LANES) is None
    assert lane_model(dict(STRAIGHT, Info_RtLnPosVal=4.0)) is None  # implausible width
    assert lane_model({}) is None

  def test_assignment_straight(self):
    lat0 = -cr.RADAR_LATERAL_OFFSET
    for lat, side in ((lat0 + 3.6, "LEFT"), (lat0 - 3.6, "RIGHT"), (lat0, None), (lat0 + 7.5, None), (lat0 - 7.5, None)):
      out = Sim().step([(30., lat)], n=5)
      for s in ("LEFT", "RIGHT"):
        assert shown(out, s) == ((True, 30.) if s == side else (False, 0.)), (lat, s, out)
      if side:
        assert out[f"LEAD_{side}_LATERAL"] == SIDE_LATERAL

  def test_nearest_per_lane(self):
    out = Sim().step([(60., 3.3), (25., 3.3), (40., -3.9)], n=5)
    assert shown(out, "LEFT") == (True, 25.) and shown(out, "RIGHT") == (True, 40.)

  def test_curve(self):
    # lanes bending left: our lane's centre is 3.0 m left at 50 m (camera curvature is right positive)
    k = -3.0 / 50 ** 2 * 2
    bend = dict(STRAIGHT, Info_LftLnCvtrVal=k, Info_RtLnCvtrVal=k)
    in_our_lane = 3.0 - cr.RADAR_LATERAL_OFFSET
    assert shown(Sim().step([(50., in_our_lane)], msg_1b5=bend, n=5), "LEFT") == (False, 0.)
    assert shown(Sim().step([(50., in_our_lane)], msg_1b5=STRAIGHT, n=5), "LEFT") == (True, 50.)  # straight lanes would misplace it
    assert shown(Sim().step([(50., in_our_lane + 3.6)], msg_1b5=bend, n=5), "LEFT") == (True, 50.)

  def test_range(self):
    assert shown(Sim().step([(75., 3.3)], n=5), "LEFT")[0]
    assert not shown(Sim().step([(85., 3.3)], n=5), "LEFT")[0]
    assert not shown(Sim().step([(60., 3.3)], msg_1b5=NO_LANES, n=5), "LEFT")[0]
    assert shown(Sim().step([(30., 3.3)], msg_1b5=NO_LANES, n=5), "LEFT")[0]

  def test_lead_guard(self):
    with_lead = dict(STRAIGHT, Longitudinal_Distance=30.)
    # the lead itself, mis-placed just past our lane's edge: not drawn beside us
    assert not shown(Sim().step([(30.5, 2.3)], msg_1b5=with_lead, n=5), "LEFT")[0]
    # a car a whole lane over, level with the lead: drawn
    assert shown(Sim().step([(30.5, 3.5)], msg_1b5=with_lead, n=5), "LEFT")[0]
    # the same mis-placed track far from the lead: drawn
    assert shown(Sim().step([(50., 2.3)], msg_1b5=with_lead, n=5), "LEFT")[0]


class TestFilters:
  def test_stationary_ignored(self):
    assert not shown(Sim().step([(30., 3.3, -25., 3, 1)], n=10), "LEFT")[0]

  def test_stopping_car_kept(self):
    s = Sim()
    assert shown(s.step([(20., 3.3, -5., 3, 2)], n=5), "LEFT")[0]
    assert shown(s.step([(20., 3.3, -25., 3, 1)], n=int(2.5 * 20)), "LEFT")[0]       # stopped beside us: still a car
    assert not shown(s.step([(20., 3.3, -25., 3, 1)], n=int(1.5 * 20)), "LEFT")[0]   # until MOVING_MEMORY runs out

  def test_oncoming_ignored(self):
    assert not shown(Sim().step([(60., 3.3, -50., 3, 2)], v_ego=25., n=10), "LEFT")[0]

  def test_empty_and_tentative_ignored(self):
    for state in (0, 1, 2, 7):
      assert not shown(Sim().step([(30., 3.3, 0., state, 2)], n=10), "LEFT")[0]
    assert shown(Sim().step([(30., 3.3, 0., 4, 2)], n=10), "LEFT")[0]  # coasted


class TestTiming:
  def test_confirm_then_hold(self):
    s = Sim()
    assert not shown(s.step([(30., 3.3)]), "LEFT")[0]                   # first sighting
    assert shown(s.step([(30., 3.3)], n=2), "LEFT")[0]                  # 0.1 s later
    dropout = [(30., 3.3, 0., 0, 2)]                                     # the radar reports the slot empty
    assert shown(s.step(dropout, n=int(cr.HOLD_TIME * 20) - 1), "LEFT")[0]   # held
    assert not shown(s.step(dropout, n=3), "LEFT")[0]                        # gone

  def test_one_frame_blip_never_drawn(self):
    s = Sim()
    s.step([(30., 3.3)])
    assert not any(shown(s.step([(30., 3.3, 0., 0, 2)]), "LEFT")[0] for _ in range(20))

  def test_stale_radar(self):
    s = Sim()
    assert shown(s.step([(30., 3.3)], n=5), "LEFT")[0]
    # the radar goes silent (other CAN keeps arriving): the tracks expire and the slot clears
    for _ in range(20):
      s.t += STEP
      s.r.update_frames([(s.t, [(0x1a0, bytes(32), 2)])])
      out = s.r.update(25., STRAIGHT, False, False)
    assert not shown(out, "LEFT")[0]


class TestBlindSpot:
  def test_rear_slots(self):
    out = Sim().step([], lbsm=True)
    assert (out["LEAD_LEFT_REAR_STATUS"], out["LEAD_LEFT_REAR_DISTANCE"], out["LEAD_LEFT_REAR_LATERAL"]) == (cr.SIDE_STATUS, REAR_DISTANCE, SIDE_LATERAL)
    assert out["LEAD_RIGHT_REAR_STATUS"] == 0
    out = Sim().step([], rbsm=True)
    assert out["LEAD_RIGHT_REAR_STATUS"] == cr.SIDE_STATUS and out["LEAD_LEFT_REAR_STATUS"] == 0

  def test_rear_slots_without_radar(self):
    r = ClusterRadar(BUS)
    r.failed = True
    out = r.update(25., STRAIGHT, True, False)
    assert out["LEAD_LEFT_REAR_STATUS"] == cr.SIDE_STATUS and out["LEAD_LEFT"] == 0


class TestCluster:
  def _cluster_162(self, side_cars, cam_162=None):
    from types import SimpleNamespace as NS
    packer = CANPacker("hyundai_canfd_generated")
    parser = CANParser("hyundai_canfd_generated", [("CCNC_0x161", 20), ("CCNC_0x162", 20)], 0)
    msg_161 = dict.fromkeys([s.name for s in packer.dbc.addr_to_msg[0x161].sigs.values()], 0)
    msg_162 = dict.fromkeys([s.name for s in packer.dbc.addr_to_msg[0x162].sigs.values()], 0)
    msg_162.update(cam_162 or {})
    hud = NS(leftLaneVisible=True, rightLaneVisible=True, leftLaneDepart=False, rightLaneDepart=False, leadDistanceBars=2, leadVisible=False)
    out = NS(steeringAngleDeg=0.0, vEgo=30.0, leftBlindspot=False, rightBlindspot=False, vCruiseCluster=0)
    msgs = create_ccnc(packer, CanBus(None, gen_empty_fingerprint()), False, True, hud, False, False, msg_161, msg_162,
                       STRAIGHT, False, out, True, 2, True, side_cars=side_cars)
    parser.update([(0, [(m[0], m[1], 0) for m in msgs])])
    return parser.vl["CCNC_0x162"]

  def test_slots_packed(self):
    side = Sim().step([(30., 3.3), (45.5, -3.9)], lbsm=True, rbsm=True, n=5)
    v = self._cluster_162(side, cam_162={"LEAD": 1, "LEAD_DISTANCE": 52.3, "LEAD_LATERAL": 12.4, "SPEEDLIMIT": 55})
    assert (v["LEAD_LEFT"], v["LEAD_LEFT_DISTANCE"], v["LEAD_LEFT_LATERAL"]) == (1, 30., SIDE_LATERAL)
    assert (v["LEAD_RIGHT"], round(v["LEAD_RIGHT_DISTANCE"], 1), v["LEAD_RIGHT_LATERAL"]) == (1, 45.5, SIDE_LATERAL)
    assert (v["LEAD_LEFT_REAR_STATUS"], v["LEAD_LEFT_REAR_DISTANCE"], v["LEAD_LEFT_REAR_LATERAL"]) == (1, REAR_DISTANCE, SIDE_LATERAL)
    assert (v["LEAD_RIGHT_REAR_STATUS"], v["LEAD_RIGHT_REAR_DISTANCE"], v["LEAD_RIGHT_REAR_LATERAL"]) == (1, REAR_DISTANCE, SIDE_LATERAL)
    # the camera's own lead and everything else pass through untouched
    assert (v["LEAD"], round(v["LEAD_DISTANCE"], 1), round(v["LEAD_LATERAL"], 1), v["LEAD_ALT"], v["SPEEDLIMIT"]) == (1, 52.3, 12.4, 0, 55)

  def test_unchanged_without_side_cars(self):
    cam = {"LEAD": 1, "LEAD_DISTANCE": 20., "LEAD_LEFT": 0}
    assert self._cluster_162(None, cam) == self._cluster_162(Sim().step([]), cam)


class TestCarIntegration:
  def _ci(self, ccnc=True):
    fp = gen_empty_fingerprint()
    cam = CanBus(None, fp).CAM
    fp[cam].update({0xCB: 24, 0x12A: 16, 0x1A0: 32} | ({0x161: 32, 0x162: 32} if ccnc else {}))
    fp[0].update({0x1ba: 24})
    CP = CarInterface.get_params(CAR.HYUNDAI_IONIQ_5_PE, fp, [], False, False, False)
    return CarInterface(CP, CarInterface.get_params_sp(CP, CAR.HYUNDAI_IONIQ_5_PE, fp, [], False, False, False))

  def test_only_lfa_alt_ccnc_cars(self):
    ci = self._ci()
    assert ci.CP.flags & HyundaiFlags.CANFD_LFA_ALT and ci.CP.flags & HyundaiFlags.CCNC
    assert ci.CS.cluster_radar is not None and ci.CS.cluster_radar.bus == CanBus(ci.CP).ACAN
    assert self._ci(ccnc=False).CS.cluster_radar is None

  def test_radar_never_in_can_parsers(self):
    # canValid comes from the parsers; the radar tracks must not be one of their messages
    ci = self._ci()
    ci.update([(SEC, [(cr.RADAR_TRACK_START + i, track_frame(30, 3.3), CanBus(ci.CP).ACAN) for i in range(32)])])
    for cp in ci.can_parsers.values():
      assert not set(cp.addresses) & set(cr.RADAR_TRACK_ADDRS)
    assert len(ci.CS.cluster_radar.tracks) == 32

  def test_garbage_radar_doesnt_change_can_valid(self):
    a, b = self._ci(), self._ci()
    rng = random.Random(1)
    acan = CanBus(a.CP).ACAN
    for i in range(300):
      t = SEC + i * 10_000_000
      base = [(0x1a0, bytes(32), CanBus(a.CP).CAM)]
      noisy = base + [(cr.RADAR_TRACK_START + rng.randrange(32), bytes(rng.randrange(256) for _ in range(rng.choice((8, 24, 64)))), acan)]
      ra, _ = a.update([(t, base)])
      rb, _ = b.update([(t, noisy)])
      assert ra.canValid == rb.canValid and ra.canTimeout == rb.canTimeout
    assert not b.CS.cluster_radar.failed


def test_card_cost():
  # a full radar cycle (32 tracks) plus selection must stay a tiny fraction of card's 10 ms budget
  import time
  s = Sim()
  tracks = [(10. + 2 * i, (-1) ** i * 3.4) for i in range(32)]
  start = time.perf_counter()
  s.step(tracks, n=40)
  per_cycle = (time.perf_counter() - start) / 40
  assert per_cycle < 0.005, per_cycle
