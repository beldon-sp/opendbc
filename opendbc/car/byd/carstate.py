from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.byd.radar_gap import (GAP_Y_FRESH_S, GAP_Y_PAIR_GATE_M,
                                       PRESENCE_VISIBLE, RadarGapMonitor)
from opendbc.car.byd.radar_multitarget import (SLOT_Y_ADDRS, Y_BAND_MAX_M, Y_BAND_MIN_M,
                                               Y_C0, Y_CODE_ZERO, Y_K)
from opendbc.car.byd.values import DBC, CarControllerParams

# Boot-mid-cruise engage hold (see the latch block in update): when the device
# boots while the stock ACC is already engaged, cruiseState.enabled is held
# False for this long after CAN start, then let through - so the engagement
# edge happens exactly once, late, when every NO_ENTRY gate is ready. Matches
# the selfdrived ready window (~15 s) and the vendor's own boot-mid-cruise
# drive (route 7--12e: radar ACC active from t=0, their latch held 0 through
# t=17.5 s, then one edge - OP enabled the same frame).
BOOT_LATCH_HOLD_TIME = 15.0  # seconds

# ACC-main fall debounce: the camera itself drops the ACC main posture for
# ~2 s with no button event (route 11 t=156.4, route 12 t=200.3: AccState and
# AccOn1 fall together, then recover). Since main-on is the arm for BOTH axes,
# a raw fall flaps every state machine OP has. Hold main through drops shorter
# than this. The panda holds its own copy slightly LONGER (BYD_ACC_MAIN_FALL_HOLD
# in safety_byd.h) so OP always stops transmitting first - an OP/panda window
# mismatch in the other direction would punch holes into the 0x316/0x32E streams.
AVAIL_FALL_DEBOUNCE_TIME = 0.5  # seconds of card loop (100 Hz)


class CarState(CarStateBase):
  def __init__(self, CP, CP_SP):
    super().__init__(CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["DRIVE_STATE"]["Gear"]

    self.lkas_prepared = False
    # EPS's own warning flag (0x318 SteerWarning). Observed in the two
    # post-v2 "LKAS Fault" latches (routes 28/2b 2026-10): 0.4-0.7 s BEFORE
    # err=4+TorqueFailed the EPS drops MainTorque to 0 and raises Warn while
    # keeping CruiseActivated=1 - a torque abort we could not see in
    # SteerErrorCode alone (it stayed 0 through the whole abort). The
    # controller stands down on it (root cause 18).
    self.steer_warning = False
    # EPS session-phase bit ("executing now") - read by the controller's c0
    # guard (root cause 17 B): Active=1 that never gets accepted is a dead
    # stream, not a session to keep storming at
    self.cruise_activated = False
    self.torque_failed = False
    self.steer_error = 0
    self.res_btn_pressed = False
    self.counter_pcm_buttons = 0
    # Stalk-blinker side mapping override (see update): 1 = exchange the
    # (2,3) left / (4,5) right detent sets - only for a stalk that reads the
    # opposite way; the 宋 was confirmed correct with the default
    # (2026-10-05..07 road test, sunnypilot). Read once at init - restart to
    # pick up changes.
    self.blinker_stalk_swap = False
    try:
      with open('/data/params/BydBlinkerStalkSwap') as f:
        self.blinker_stalk_swap = int(f.read().strip()) != 0
    except Exception:
      try:
        from openpilot.common.params import Params
        val = Params().get('BydBlinkerStalkSwap', block=False)
        self.blinker_stalk_swap = val is not None and int(val) != 0
      except Exception:
        pass
    self.eps_state_msg = {}
    self.is_cruise_latch = False
    self.boot_frames = 0  # update() calls since process start (100 Hz card loop)
    self.avail_fall_frames = 0  # consecutive updates without the ACC main posture
    # one genuine radar session (AccControlActive / AccState 2/3/5) seen this
    # drive cycle - same lifecycle as boot_frames (process start == ignition).
    # Gates the engage latch: the ignition-leftover posture (AccState=1,
    # SetSpeed=30, AccOn1=1, still in Park - route 0000000f) reports main
    # without a session and must never arm OP by itself.
    self.ever_engaged = False
    # stock camera's ACC_MPC_STATE (bus 2); echoed back on bus 0 by the
    # controller so the spoofed LKAS request keeps the camera's SETME_* fields
    self.cam_lkas = {}
    # ACC domain frames (bus 2) as the OP-longitudinal echo base
    self.radar_acc_msg = {}
    self.adas_msg = {}
    self.aeb_msg = {}

    # BYD T8 forward-gap monitor (docs/byd-radar-010x.md §④e/§④f): its OWN
    # CANParser for bus1 0x109/0x10A, driven by CarInterface.update. Kept
    # outside get_can_parsers on purpose - the base aggregates canValid/
    # canTimeout over those dicts, and bus1 availability must not start
    # emitting CAN FAULTED on trims whose MRR stays silent. Telemetry only
    # until the BydRadarGapGate param flips (road-test gate, desire_helper_byd).
    self.gap_cp = None
    if Bus.radar in DBC[CP.carFingerprint]:
      # stage-3 y supply (byd-radar-y-wire-port 3.2): the C-F0 rows ride this
      # same independent parser (canValid/canTimeout semantics untouched).
      gap_msgs = [(265, 25), (266, 25)] + [(a, 15) for a in SLOT_Y_ADDRS]
      self.gap_cp = CANParser(DBC[CP.carFingerprint][Bus.radar], gap_msgs, 1)
    self.gap_mon = RadarGapMonitor()
    # D3 y pairing state: per-row (d, y) latched at its own F0 frame, plus
    # pairing counters for the gap telemetry surface (3.5, no new capnp)
    self._gap_y_rows: dict[int, tuple[float, float]] = {}
    self._gap_y_steps: dict[int, int] = {}
    self._gap_step = 0
    self.gap_y_pair = (0, 0)  # (paired, seen) steps

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_adas = can_parsers[Bus.adas]

    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    # EPS feedback (also read back by the ADAS domain)
    self.lkas_prepared = bool(cp.vl["ACC_EPS_STATE"]["LKAS_Prepared"])
    self.cruise_activated = bool(cp.vl["ACC_EPS_STATE"]["CruiseActivated"])
    self.torque_failed = bool(cp.vl["ACC_EPS_STATE"]["TorqueFailed"])
    self.steer_warning = bool(cp.vl["ACC_EPS_STATE"]["SteerWarning"])
    # EPS's own warning level: observed 0 while idle and in clean sessions,
    # 2 as a ~0.5 s stand-down warning before it escalates to 4 + SteerWarning
    # + TorqueFailed (route c9f1698c82 seg 0). The controller obeys it like
    # the vendor obeys the EPS state bits.
    self.steer_error = int(cp.vl["ACC_EPS_STATE"]["SteerErrorCode"])
    self.eps_state_msg = cp.vl["ACC_EPS_STATE"]

    # stock camera's ACC_MPC_STATE (bus 2); the controller echoes this frame
    # onto bus 0 with the LKAS torque overridden. Echoing the camera's SETME_*
    # / MPC_State fields is required - building the frame from scratch faults
    # the DiPilot ADAS domain ('check multifunction video controller').
    self.cam_lkas = cp_adas.vl["ACC_MPC_STATE"]

    # ACC domain frames (bus 2) cached as the echo base for OP longitudinal
    # (transparent ACC_CMD/ACC_HUD/ACC_AEB replacement onto bus 0). Empty dicts
    # before the first frame - the controller synthesizes an idle ACC_CMD then.
    self.radar_acc_msg = dict(cp_adas.vl["ACC_CMD"])
    self.adas_msg = dict(cp_adas.vl["ACC_HUD_ADAS"])
    self.aeb_msg = dict(cp_adas.vl["ACC_AEB"])

    # speed
    # Song Plus DM-i: the 0x122 wheel-speed message reads all zeros and the
    # 0x418 BSD_RADAR VEHICLE_SPEED field is constant (~8.5), so neither is a
    # usable speed. The real vehicle speed comes from 0x1f0 ESP_SPEED
    # (20 Hz, bus 0) byte 4 - note the signal is 0.9 km/h per bit, NOT 1.0
    # (GPS fit on the 2026-10-02/03 road tests: true = 1.113 x raw, flat over
    # 20-120 km/h; the earlier "verified against camera odometry" claim was
    # circular - the odometry scale was itself anchored to this signal).
    # CarStateBase folds that in via CP.wheelSpeedFactor = HUD_MULTIPLIER.
    vehicle_speed_kph = cp.vl["ESP_SPEED"]["VehicleSpeed"]
    ret.vEgoRaw = vehicle_speed_kph * CV.KPH_TO_MS * self.CP.wheelSpeedFactor
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    # The BYD cluster over-reads the calibrated (GPS-anchored, 10/9) speed by
    # ~9% (drive route 4: cluster 38/48 vs device 35/44 km/h, both episodes).
    # Put the UI number on the cluster's scale so driver and screen agree;
    # control stays anchored to calibrated truth (this field is display-only).
    ret.vEgoCluster = ret.vEgo * CarControllerParams.CLUSTER_OVERREAD
    ret.standstill = ret.vEgoRaw < 0.05

    ret.brakeHoldActive = False

    # gear
    can_gear = int(cp.vl["DRIVE_STATE"]["Gear"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    ret.doorOpen = any([cp.vl["BCM"]["RearLeftDoor"],
                        cp.vl["BCM"]["FrontLeftDoor"],
                        cp.vl["BCM"]["RearRightDoor"],
                        cp.vl["BCM"]["FrontRightDoor"]])
    # the car already enforces the seatbelt warning itself; OP does not need to
    # block engagement on it. The DriverSeatBeltFasten bit encoding is also
    # unverified on Song Plus DM-i (would otherwise report unlatched while
    # buckled). Matches the community BYD_Files port.
    ret.seatbeltUnlatched = False

    # pedals - the analog 0.01/unit signals idle at LSB noise, and >0.01 means
    # a single count of noise fires the press. The 2026-10 road tests show the
    # cost: openpilot state flapped enabled<->overriding 8x in 2 s (route 2d
    # t=1485, 31 t=1486) - each false gas fire releases the longitudinal
    # actuator for seconds while the stock radar FORCE_ACCEL-bounces (st 5,
    # AccControlActive 0) - the "车控退出几秒才回来" the driver feels.
    # 0.05 (5 counts) clears the noise band and still trips on a real touch.
    ret.gasPressed = cp.vl["PEDAL"]["AcceleratorPedal"] > CarControllerParams.PEDAL_PRESS_THRESHOLD
    # raw brake is a deprecated-group member in this fork's CarState (no consumer);
    # the analog pedal press feeds brakePressed below (0.05 = 5 counts, see PEDAL_PRESS_THRESHOLD)
    ret.brakePressed = bool(cp.vl["DRIVE_STATE"]["BrakePressed"]) or \
      cp.vl["PEDAL"]["BrakePedal"] > CarControllerParams.PEDAL_PRESS_THRESHOLD

    # steering
    ret.steeringAngleDeg = cp.vl["EPS"]["SteeringAngle"]
    # EPS-reported wheel rate in deg/s (4 deg/s/bit, 0-1020); used by the LKAS
    # envelope gate to tell a settled wheel from a full-lock-speed swing
    ret.steeringRateDeg = cp.vl["EPS"]["SteeringAngleRate"]
    # bus0 0x222 YAW_RATE (dbc from carrot2 byd_han_dmev_2020.dbc, validated
    # against kinematic yaw on our car: corr 0.90-0.97, slope ~1.0, left
    # positive - docs/byd-carrot2-reference-2026-10-21.md s7.6). Guard: fields
    # decoded from an all-zero raw payload are the ECU-not-ready pattern -
    # (0*0.002133-2.094)-(0*0.002133-0.13) = -1.964 rad/s of pure offset-factor
    # garbage, never a real sample (straight driving idles at raw ~981/61,
    # i.e. mid-scale). Before the very first frame the parser leaves vl at its
    # 0.0 init (no factor applied) and both branches below yield 0.0 anyway.
    yaw_rate = cp.vl["YAW_RATE"]["YawRate"]
    yaw_offset = cp.vl["YAW_RATE"]["YawRateOffset"]
    if abs(yaw_rate + 2.094) < 1e-9 and abs(yaw_offset + 0.13) < 1e-9:
      ret.yawRate = 0.0
    else:
      ret.yawRate = yaw_rate - yaw_offset
    ret.steeringTorque = cp.vl["ACC_EPS_STATE"]["SteerDriverTorque"]
    ret.steeringTorqueEps = cp.vl["ACC_EPS_STATE"]["MainTorque"]
    # 5-frame debounce: single-frame torque noise spikes must not flap
    # steeringPressed (they would keep selfdrived in overriding)
    ret.steeringPressed = self.update_steering_pressed(
      bool(abs(ret.steeringTorque) > CarControllerParams.STEER_THRESHOLD), 5)

    # stock ACC status; LKA is coupled to the stock ACC on this platform
    # Song Plus DM-i encodes AccState differently from Han: 1 = main on /
    # standby (also the ignition-leftover state, SetSpeed=30, AccOn1=1, still
    # in Park - route 0000000f), 2/3/5 = engaged-only states. Two independent
    # things live in here:
    #
    #   main    = AccOn1 or AccState in (1,2,3,5)  - the stalk's main switch.
    #             Survives brake-cancel / CANCEL / the radar's bounce blips
    #             (AccOn1 stays 1 through all of them, route 11/12); drops
    #             only when the driver turns ACC off (or the camera glitches,
    #             hence the debounce).
    #   session = AccControlActive or AccState in (2, 3, 5) - the radar
    #             actually commanding (0x32e). Bounces (60-110 ms AccState
    #             2<->1 flaps), drops on brake (stock auto-resumes ~40 ms
    #             after release, route 11/12: 5/5 windows) and on CANCEL.
    #
    # cruiseState.available = debounced main - it must track the stalk, not
    # the radar's session churn.
    acc_state = int(cp_adas.vl["ACC_HUD_ADAS"]["AccState"])
    acc_on1 = bool(cp_adas.vl["ACC_HUD_ADAS"]["AccOn1"])
    raw_main = acc_on1 or acc_state in (1, 2, 3, 5)
    self.avail_fall_frames = 0 if raw_main else self.avail_fall_frames + 1
    acc_main = raw_main or self.avail_fall_frames < int(AVAIL_FALL_DEBOUNCE_TIME * 100)
    ret.cruiseState.available = acc_main
    set_speed = cp_adas.vl["ACC_HUD_ADAS"]["SetSpeed"]
    # follow the stock ACC set speed directly (community port does the same);
    # no artificial 30 km/h floor
    ret.cruiseState.speedCluster = set_speed * CV.KPH_TO_MS if acc_main else 0.
    ret.cruiseState.speed = ret.cruiseState.speedCluster

    acc_control_active = bool(cp_adas.vl["ACC_CMD"]["AccControlActive"])
    standstill_state = bool(cp_adas.vl["ACC_CMD"]["StandstillState"])
    ret.cruiseState.standstill = standstill_state
    ret.cruiseState.nonAdaptive = False

    self.res_btn_pressed = cp.vl["PCM_BUTTONS"]["BTN_AccUpDown_Cmd"] != 0
    self.counter_pcm_buttons = cp.vl["PCM_BUTTONS"]["Counter"]

    stock_acc_on = acc_control_active or acc_state in (2, 3, 5)
    # vendor parity (op_byd decompile 2026-10-05): the arm is re-judged EVERY
    # main-on cycle - main off clears ever_engaged, so an ACC off/on round
    # trip must show a fresh genuine session before OP arms again. Our old
    # process-monotonic flag let a stale session arm a later pure-set cycle.
    if acc_main:
      self.ever_engaged = self.ever_engaged or stock_acc_on
    else:
      self.ever_engaged = False

    # cruiseState.enabled = the arm latch: main-on (debounced) + ever engaged.
    # Brake and session drops deliberately do NOT clear it - that is the whole
    # point: the longitudinal state machine stops churning with the radar's
    # session (brake -> stock standby -> old latch fell -> pcmDisable killed
    # both axes; CANCEL/bounce did the same with no auto-recovery), lateral
    # survives everything short of main-off, and the longitudinal output is
    # gated on the live session downstream (carcontroller session gate +
    # panda acc checks). AccControlActive is the reliable session signal: 0
    # at ignition-leftover/standby, 1 whenever the MPC actually commands ACC,
    # including the SNG standstill hold (StandstillState=1).
    self.is_cruise_latch = acc_main and self.ever_engaged

    # boot-mid-cruise engage hold: reporting the raw latch immediately after
    # boot fires the pcmEnable edge while OP cannot act on it (canValid still
    # false / NO_ENTRY ready window), eating it for good - OP stays disabled
    # until the driver cycles ACC (route 0000000f: 20.6 s ACC-on/OP-off).
    # Holding the latch low here makes the edge land late and genuine instead:
    # OP engages the frame the hold expires. A SET pressed during the hold is
    # absorbed into the hold-end edge. cruiseState.available stays live.
    self.boot_frames += 1
    if self.boot_frames < int(BOOT_LATCH_HOLD_TIME * 100):  # card loop = 100 Hz
      self.is_cruise_latch = False

    ret.cruiseState.enabled = self.is_cruise_latch
    # stock LKA is coupled to the stock ACC on this platform; used by the
    # experimental angle path for engagement gating
    self.lka_on = ret.cruiseState.enabled

    # stalks and blind spots
    # Song Plus: the Han-layout LeftIndicator/RightIndicator bits (STALKS
    # byte0) NEVER move on this car - drive route 4 (2026-10-04): byte0
    # constant 0x01 through two 5-7 s stalk holds. The stalk position lives in
    # TURN_SIGNAL_SWITCH (byte4, 36|3@1+) and the vendor decodes it as two
    # SETS, not two singles - carstate.py.disasm 10706/10756:
    #   leftBlinker = switch in (2, 3), rightBlinker = switch in (4, 5)
    # (BYD_SONG_PLUS is in its ALT_BLINKER_CARS list: stalk-based blinkers).
    # Confirmed on the car (sunnypilot road test, 415-segment raw sweep):
    # a flick reads 1 -> transient -> held detent, right = 4 then 5,
    # left = 2 then 3. The earlier single-value guess (5 = left) inverted both
    # sides and is what the road test hit: every "right lane change" fired off
    # the 0.2-0.3 s transient 4 (60 starting-right events, all on blips; zero
    # long right holds in 415 segments), while a real left hold (3) decoded as
    # no blinker at all - "right works, left does nothing".
    switch = int(cp.vl["STALKS"]["TURN_SIGNAL_SWITCH"])
    left_vals, right_vals = ((4, 5), (2, 3)) if self.blinker_stalk_swap else ((2, 3), (4, 5))
    ret.leftBlinker = switch in left_vals
    ret.rightBlinker = switch in right_vals
    ret.espDisabled = False
    ret.stockAeb = bool(cp_adas.vl["ACC_HUD_ADAS"]["AEB"])
    ret.stockFcw = bool(cp_adas.vl["ACC_HUD_ADAS"]["FCW"])

    ret.leftBlindspot = bool(cp.vl["BSD_RADAR"]["LEFT_APPROACH"])
    ret.rightBlindspot = bool(cp.vl["BSD_RADAR"]["RIGHT_APPROACH"])

    # The EPS latches TorqueFailed and gives up all steering input until it is
    # power-cycled (real vehicle, route 0000001c/20/22; the independent
    # yysnet/opendbc port documents the same behavior). Surface it so selfdrived
    # alerts the driver and blocks engagement instead of silently not steering;
    # the controller additionally stands down on the same bit.
    ret.steerFaultPermanent = bool(cp.vl["ACC_EPS_STATE"]["TorqueFailed"])

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    # Song Plus DM-i bus layout (verified from real vehicle CAN logs):
    #   bus0 = powertrain/chassis + ADAS feedback (EPS, BSD_RADAR, ACC_EPS_STATE, ...)
    #   bus2 = ACC/ADAS command domain (ACC_HUD_ADAS, ACC_CMD)
    dbc = DBC[CP.carFingerprint][Bus.pt]
    pt_messages = [
      ("EPS", 100),              # 0x11f
      ("WHEEL_SPEED", 50),       # 0x122
      ("ESP_SPEED", 20),         # 0x1f0, real vehicle speed in km/h
      ("BCM", 10),               # 0x12d
      ("STALKS", 10),            # 0x133
      ("DRIVE_STATE", 20),       # 0x242
      ("ACC_EPS_STATE", 50),     # 0x318
      ("PEDAL", 20),             # 0x342
      ("PCM_BUTTONS", 10),       # 0x3b0
      ("BSD_RADAR", 10),         # 0x418
      ("YAW_RATE", 50),          # 0x222, real-vehicle rate ~50 Hz (NOT nominal 60)
    ]
    adas_messages = [
      ("ACC_HUD_ADAS", 50),      # 0x32d
      ("ACC_CMD", 50),           # 0x32e
      ("ACC_AEB", 50),           # 0x32f, stock radar AEB heartbeat (echoed on bus 0 by OP long)
      ("ACC_MPC_STATE", 50),     # 0x316, stock camera LKAS frame (echoed on bus 0)
    ]
    return {
      Bus.pt: CANParser(dbc, pt_messages, 0),
      Bus.adas: CANParser(dbc, adas_messages, 2),
    }

  def update_radar_gap(self, can_packets, v_ego: float) -> tuple[bool, bool]:
    """BYD T8 (docs/byd-radar-010x.md): feed the bus1 gap monitor. Called once
    per card step (100 Hz) from CarInterface.update. Returns (closing, new)."""
    if self.gap_cp is None:
      return False, False
    updated = self.gap_cp.update(can_packets)
    fresh = 265 in updated
    # b7 via the factor-1 MSG_109_B7 placeholder: the monitor is u-free and the
    # dbc LongDist_109 scale is runtime-configurable (BydRadarDistScale), so the
    # counts path must never depend on the scale factor (u pinned 0.25 m/count
    # 2026-10-19; docs/byd-radar-010x.md ②/⑧)
    b7 = int(self.gap_cp.vl[265]["MSG_109_B7"])
    presence = int(self.gap_cp.vl[266]["Presence_10A"])
    hud = self.adas_msg
    cls = int(hud.get("HasLead", 0)) * int(hud.get("LeadingDistance", 0))
    y_rel = self._slot_y_rel(updated, b7, presence)
    self.gap_mon.update(0.01, b7, presence, cls, float(v_ego), fresh, y_rel=y_rel)
    return self.gap_mon.closing, self.gap_mon.new_target

  def _slot_y_rel(self, updated, b7: int, presence: int):
    """byd-radar-y-wire-port 3.2: distance-gated slot-row y supply (design D3).

    The C-F0 lateral code is PINNED (2026-11-02, radar_multitarget.py Y_*) and
    rides the gap parser as a factor-1 raw twin. Each row's (d, y) latches at
    its own F0 frame (F0-instant data, y trusted only inside the 8-60 m band).
    At each step the fresh rows pair against the current b7 lead: nearest
    |d_row - b7*0.25| within GAP_Y_PAIR_GATE_M wins; no pair -> None, which is
    exactly the monitor's LANE_UNKNOWN degradation. The lead-validity gate
    mirrors the monitor's b7 source (presence==7 AND 0 < b7 <= 245). Counters
    feed the gap telemetry surface (3.5)."""
    self._gap_step += 1
    seen = self.gap_y_pair[1] + 1
    y_rel = None
    if presence == PRESENCE_VISIBLE and 0 < b7 <= 245:
      b7_m = b7 * 0.25
      best = None
      for addr in SLOT_Y_ADDRS:
        if addr in updated:
          vl = self.gap_cp.vl[addr]
          d = float(vl[f"LongDist_C{addr:X}"])
          code = int(vl[f"MSG_SLOTC_LatCodeRaw_C{addr:X}"]) - Y_CODE_ZERO
          if Y_BAND_MIN_M <= d <= Y_BAND_MAX_M:
            # (base + d_off) = d: the dbc distance already carries the offset
            self._gap_y_rows[addr] = (d, d * Y_K * (code - Y_C0))
            self._gap_y_steps[addr] = self._gap_step
        row = self._gap_y_rows.get(addr)
        step = self._gap_y_steps.get(addr)
        if row is None or self._gap_step - step > round(GAP_Y_FRESH_S / 0.01):
          continue
        dj = abs(row[0] - b7_m)
        if dj <= GAP_Y_PAIR_GATE_M and (best is None or dj < best[0]):
          best = (dj, row[1])
      if best is not None:
        y_rel = best[1]
        self.gap_y_pair = (self.gap_y_pair[0] + 1, seen)
        return y_rel
    self.gap_y_pair = (self.gap_y_pair[0], seen)
    return None
