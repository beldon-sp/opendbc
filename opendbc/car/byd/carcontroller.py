from opendbc.can import CANPacker
from opendbc.car import Bus, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_std_steer_angle_limits
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.byd import bydcan
from opendbc.car.byd.values import USE_ANGLE_STEERING, CarControllerParams

LongCtrlState = structs.CarControl.Actuators.LongControlState

# Accel command slew limit (m/s^3), applied at the 50 Hz ACC_CMD rate. The
# vendor's own plan derives JerkUpper/Lower from its jerk and never steps the
# accel; we previously plugged the 100 Hz controller output straight into
# 0x32E, so every plan discontinuity (model lead jitter, session hand-back,
# the SetSpeed zero-blip clamp) landed on the powertrain as a full step - the
# 2026-10-05 driver complaint "一直在加油刹车" (never smooth, alternating
# gas/brake). Asymmetric like the transmitted envelope: rising 1.5, falling
# -1.5 keeps emergency braking reachable (0 -> -4 in ~2.7 s) while cutting
# the command's high-frequency energy.
ACCEL_SLEW_UP = 1.5
ACCEL_SLEW_DOWN = 1.5

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.packer = CANPacker(dbc_names[Bus.pt])

    # lateral state (vendor session architecture)
    self.apply_torque_last = 0
    self.apply_angle_last = 0
    self.lkas_active = False
    self.retry_burst = 0      # remaining ReqPrepare frames of the engage/retry burst
    self.silence_counter = 0  # armed |request|~0 backstop
    # low-speed fight guards A+B (root cause 17, values.py)
    self.conflict = False     # A: latched while our demand opposes a heavy hand
    self.conflict_frames = 0  # entry debounce (a momentary demand flip must not flap it)
    self.c0_frames = 0        # B: frames of Active=1 with EPS CruiseActivated=0
    self.c0_hold_frames = 0   # B: re-arm hold after a c0 stand-down
    self.echo_frames = 0      # C: request-vs-Echo divergence streak (root cause 18)
    self.echo_hold_frames = 0  # C: re-arm hold after an echo stand-down

    # longitudinal output state: the accel value actually last broadcast on
    # 0x32E (m/s^2, OP units) - the reference the slew limiter ramps from so
    # OP<->radar handovers are continuous, not stepped
    self.accel_cmd_sent = 0.0
    # minimal-brake (over set speed) latch, hysteresis state - see
    # _update_longitudinal (the bare threshold chatters on cluster noise)
    self.min_brake_latched = False

  def _update_torque_lateral(self, CC, CS):
    """Default torque path: steer via the LKAS_Output request in ACC_MPC_STATE (790).

    Session architecture is the vendor's, frame-proven on its own drive logs
    (route 00000037, docs_site/op_byd_logs/7--12e_0) - the op_byd build never
    latches the EPS because it never produces a state the EPS rejects:

      activation the EPS's CruiseActivated bit is its session-PHASE flag
                 ("executing now"), raised only after it accepts a session.
                 The vendor streams Active=1 with c=0 for up to 2.9 s and
                 waits - our c-gated build deadlocked for two full drives
                 (route 0000000d: 7218 TX frames, all zero). Stream and
                 wait; the bounded request + hands-light arming keep the
                 waiting phase harmless (root cause 16, corrected).
      idle        Config=3, Active=0, lanes 0/0, request 0 streamed
                  unconditionally (their armed-standby posture)
      engage      3-frame ReqPrepare burst with lanes 2/2, then Active=1
                  ~50 ms later - the EPS ack is NOT waited for (EPS
                  LKAS_Prepared stayed 0 for the vendor's entire drive)
      in-session  request = model demand through the driver limit, full
                  authority: they reach -153 against a +166 driver yank and
                  193 absolute, ~8 sign flips in 27 s (smooth)
      dropout     Active->0 then IMMEDIATELY re-burst and re-activate within
                  60-80 ms (twice in the log, zero EPS complaint)
      disengage   request ramps 52->0 over ~80 ms with lanes still 2/2, lanes
                  drop to 0/0, then Active=0

    Every one of our ten TorqueFailed latches was a state the vendor cannot
    produce: armed-silence from yield-to-zero hesitation, an oscillating
    capped request stream (follow-flip kept the lateral loop open), or a
    dithering cold arm. So there is no defense stack here - full-authority
    output, instant exit-and-retry, and the EPS's own declarations
    (TorqueFailed / SteerErrorCode) as the only hard stops.
    """
    if self.frame % 2 != 0:
      return None

    drv = CS.out.steeringTorque
    demand = int(round(CC.actuators.torque * CarControllerParams.STEER_MAX))
    # Request envelope (root cause 16, corrected): the vendor's own drive
    # never exceeds 193 absolute - INCLUDING its waiting-for-activation
    # phase (up to 2.9 s at Active=1 while the EPS still reports
    # CruiseActivated=0, route 7--12e: 415 waiting frames). The uncapped
    # loop railed at +-300 in exactly that phase (open-loop windup), and the
    # residue on the bus at err=2 is what escalated to err=4 + TorqueFailed.
    demand = max(-CarControllerParams.STEER_MAX_REQUEST, min(CarControllerParams.STEER_MAX_REQUEST, demand))

    # Brake dead-session yield (drive route 4, 394-410 s: while the driver
    # brakes the EPS refuses to enter a lateral session - c=0/mt=0 across 16
    # s - but REMAIN_ACTIVE latActive keeps us armed, so we streamed +-200
    # requests at a rack the EPS would not move while the driver fought the
    # phantom torque by hand). This is NOT the armed-zero latch class (no
    # session was ever accepted, nothing is being held open): ramp the
    # request to zero for the brake+refused window, keep the session frame
    # alive, resume on release (c rose 0.5 s after release, same trace - the
    # EPS never lost the ability to follow, it just declines while braking).
    # Guard B is exempt here too: bounded dormancy is not a dead stream to
    # stand down, and its churn would punch holes in the 0x316 flow.
    brake_yield = bool(CS.out.brakePressed) and not CS.cruise_activated
    if brake_yield:
      demand = 0

    # Guard A - opposing-hand yield WHILE THE EPS IS NOT EXECUTING (root
    # cause 17, route 1f: the storm that latched was our demand pinned at
    # +-200 against a 120-229 opposing hand at c=0/mt=0 - we were fighting
    # the driver into an EPS that had refused the session). c=1 is NOT
    # guarded: the vendor out-authorities the driver at full authority in a
    # live session (-153 vs +166, route 7--12e) and standing down mid-fight
    # there would recreate the armed-zero-during-fight latch class that
    # 13b51bb5c4 removed. Output-level veto only - arming is never blocked.
    # 0.1 s entry debounce (the demand loop flips sign legitimately at ~1 Hz,
    # a single-frame opposition must not flap it), hysteresis exit - light
    # hand, same-direction demand, or the EPS taking the session over.
    if self.conflict:
      if CS.cruise_activated or abs(drv) < CarControllerParams.STEER_YIELD_DRV_RELEASE or demand * drv >= 0:
        self.conflict = False
    elif not CS.cruise_activated and demand * drv < 0 and abs(drv) > CarControllerParams.STEER_YIELD_OPPOSING_TORQUE:
      self.conflict_frames += 1
      if self.conflict_frames >= 5:
        self.conflict = True
    else:
      self.conflict_frames = 0

    # Guard B - dead-session timeout (root cause 17: 1f streamed Active=1 at
    # c=0 for ~45 s below 30 km/h; the EPS only ever waits 2.86 s for a
    # session it wants, and it WANTED none). CruiseActivated is the EPS's own
    # "executing" bit: an accepted session shows c=1 within the vendor's wait
    # band. Active=1 + c=0 past STEER_C0_WAIT_FRAMES is a stream into a
    # refusing EPS - exactly the residue that escalates err=2 -> 4. Stand
    # down (the ramp is under the armed-zero band by construction: the
    # silence backstop's measured 0.42 s exit is the same geometry), then
    # hold the retry burst STEER_C0_RETRY_HOLD_FRAMES, released early if the
    # EPS does activate. No assist is lost: at c=0 the EPS was not moving the
    # rack anyway (mt=0 across the whole 1f storm).
    if self.lkas_active and not CS.cruise_activated and not brake_yield:
      self.c0_frames += 1
    else:
      self.c0_frames = 0
    if self.c0_frames >= CarControllerParams.STEER_C0_WAIT_FRAMES:
      self.c0_hold_frames = CarControllerParams.STEER_C0_RETRY_HOLD_FRAMES
      self.c0_frames = 0
    if CS.cruise_activated:
      self.c0_hold_frames = 0
    elif self.c0_hold_frames > 0:
      self.c0_hold_frames -= 1

    # Guard C - torque-echo divergence (root cause 18, routes 28/2b 2026-10):
    # the EPS can abort torque mid-session (MainTorque -> 0, SteerWarning -> 1)
    # while still declaring the session active, and the old guard stack saw
    # nothing (err stayed 0, c stayed 1). Our loop then integrated error into
    # a 121->180 open-loop ramp and the EPS answered with err=4 + TorqueFailed
    # - the exact two "LKAS Fault" drives. The vendor's STEER_ERROR_MAX=46
    # check (request vs MainTorque echo) is the missing piece: past it for
    # 0.2 s is "we are demanding, the EPS is NOT delivering" - stand down and
    # hold the retry 3 s. Live-session steering tracks within ~20 counts of
    # lag (28/2b clean traces), so 46 has >2x margin. Scoped to c=1 (the EPS
    # declared it is executing): the vendor's own c=0 WAIT stream carries
    # requests up to 193 with mt legitimately 0 until acceptance (route
    # 7--12e, 415 frames) - clipping that would break the handshake, and it
    # is guard B's domain anyway.
    echo_err = abs(self.apply_torque_last - int(CS.out.steeringTorqueEps))
    if self.lkas_active and CS.cruise_activated and abs(self.apply_torque_last) > 15 and \
       echo_err > CarControllerParams.STEER_ERROR_MAX:
      self.echo_frames += 1
      if self.echo_frames >= CarControllerParams.STEER_ECHO_GUARD_FRAMES:
        self.echo_frames = 0
        self.echo_hold_frames = CarControllerParams.STEER_ECHO_RETRY_HOLD_FRAMES
    else:
      self.echo_frames = 0
    if self.echo_hold_frames > 0 and not CS.steer_warning:
      self.echo_hold_frames -= 1

    lkas_req_prepare = 0

    # The EPS's CruiseActivated bit (0x318 bit1) is its own session-PHASE
    # flag - "I am executing a session" - not a permission and not the
    # stock-ACC state. Proven in both directions: the vendor streams
    # Active=1 with c=0 for up to 2.9 s waiting for the EPS to activate
    # (route 7--12e), and our own c-gated build streamed pure idle for two
    # full drives (route 0000000d) because the EPS only raises the bit AFTER
    # accepting a session we never sent - a deadlock. So: stream the session
    # and let the EPS activate when it decides; what keeps the waiting phase
    # safe is the bounded request above. The old hands-light arm gate
    # (STEER_ARM_DRV_TORQUE, root cause 15) is GONE: routes 1d/1e 2026-09-30
    # measured it vetoing re-arms for 1.5-4.5 s after every brake release and
    # every cancel+re-SET while the driver was steering (drv 83-182), which
    # the driver read as "no assist until I press RES" - pressing RES only
    # worked because the hand leaves the wheel to reach the stalk (t=162.48:
    # re-armed with no button press). The vendor arms at any hands state and
    # was never latched doing it; the request envelope is the protection.

    # hard stops - declared by the driver (guard A) or the EPS itself (err
    # bits, SteerWarning, and the guard C echo divergence). The
    # vendor otherwise has none: it arms at zero request, steers parking
    # maneuvers at 37+ deg and 1-17 km/h, and out-authorities the driver
    # WHEN THE EPS IS EXECUTING (-153 vs +166 at c=1) - what 1f showed is
    # that past ~140 of OPPOSING hand at c=0 the same stream earns err=4.
    # The brake inhibit is GONE (user decision 2026-09-30 late: braking must
    # not exit lateral control - only main-off does). The EPS's own c-bit
    # semantics already handle the brake-cancel case: guard B bounds any
    # dead-session stream, so the old 0.08-0.12 s re-arm gap after every
    # brake release disappears entirely.
    allow = CC.latActive and not CS.out.standstill \
      and not CS.torque_failed and not CS.steer_error and not self.conflict \
      and self.c0_hold_frames == 0 \
      and not CS.steer_warning and self.echo_hold_frames == 0

    if self.lkas_active:
      if CS.torque_failed:
        # EPS-declared dead: hard cut, no ramp out of an EPS error (it has
        # already latched until ignition-off; there is nothing to be graceful
        # toward)
        new_torque = 0
        self.apply_torque_last = 0
        self.lkas_active = False
        self.silence_counter = 0
      elif allow:
        # armed-silence backstop (see values.py): exit to the retry burst
        # before the measured 0.48 s latch band when the demand loop itself
        # has gone quiet
        if abs(self.apply_torque_last) < 2 and abs(demand) < 6:
          self.silence_counter += 1
          if self.silence_counter >= CarControllerParams.STEER_ZERO_EXIT_FRAMES:
            allow = False
        else:
          self.silence_counter = 0
      if allow:
        new_torque = demand
      else:
        # stand-down: ramp the request to zero (vendor exit ramps 52->0 over
        # ~80 ms), keep Active=1 until it lands, then disarm. If OP is still
        # engaged and the EPS has not declared a fault, re-burst immediately
        # - the vendor re-arms a dropped session within 60-80 ms.
        new_torque = 0
        if self.apply_torque_last == 0:
          self.lkas_active = False
          self.silence_counter = 0
          if CC.latActive and not CS.torque_failed and not CS.steer_error:
            self.retry_burst = 3
    else:
      if allow:
        # engage/retry burst: 3 frames of ReqPrepare with lanes 2/2, request
        # 0, then activate. No ack wait - the burst itself is the handshake,
        # and no hands wait either (the route 0000000a / 909633 latches that
        # motivated the gate predate the session-architecture rewrite: its
        # rail and oscillation sources are structurally gone, and the vendor
        # arms at any hands state - see the comment block above).
        lkas_req_prepare = 1
        if self.retry_burst == 0:
          self.retry_burst = 3
        self.retry_burst -= 1
        if self.retry_burst == 0:
          self.lkas_active = True
      new_torque = 0

    self.apply_torque_last = apply_driver_steer_torque_limits(
      new_torque, self.apply_torque_last, drv, CarControllerParams)

    # 50 Hz; echo the stock camera's ACC_MPC_STATE so the SETME_* / MPC_State
    # fields match what the DiPilot ADAS domain expects (from-scratch frames
    # fault the camera). The session Config is 3 in EVERY state - idle
    # included - because that is the vendor's own idle posture (lanes 0/0
    # make it standby, not an armed-silent session).
    if CS.cam_lkas:
      return bydcan.create_lkas_request(
        self.packer, CS.cam_lkas, self.apply_torque_last, self.lkas_active,
        lkas_req_prepare, CarControllerParams.STEER_SESSION_CONFIG, (self.frame // 2) % 16)
    return None

  def _update_angle_lateral(self, CC, CS):
    """Experimental angle path: steer via the STEERING_MODULE_ADAS (482) angle request."""
    apply_angle = apply_std_steer_angle_limits(CC.actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw,
                                               CS.out.steeringAngleDeg, CC.latActive, CarControllerParams.ANGLE_LIMITS)

    # engage logic tied to the stock LKA button state
    if CS.lka_on:
      self.lkas_active = True
    if not CS.lka_on and not CS.out.cruiseState.enabled:
      self.lkas_active = False

    if CS.out.steeringTorqueEps > 15:
      apply_angle = CS.out.steeringAngleDeg

    lat_active = CC.latActive and self.lkas_active and not CS.out.standstill

    # 50 Hz
    if self.frame % 2 == 0:
      return bydcan.create_can_steer_command(
        self.packer, apply_angle, lat_active, CS.out.standstill, (self.frame // 2) % 16)
    return None

  def _update_longitudinal(self, CC, CS, can_sends):
    """OP longitudinal: transparent replacement of the radar's ACC frames on bus 0.

    Mirrors the vendor build's TX (route 00000037): all three messages at 50 Hz
    unconditionally - ACC_CMD is the radar frame with the acceleration fields
    overridden while engaged, ACC_HUD/ACC_AEB are pure echoes. The stock radar
    keeps owning the session on bus 2, so cruiseState and cancel semantics are
    untouched; the firmware blocks the stock frames bus2->bus0 while the
    LONGITUDINAL safety flag is set.

    Session gate: with the main-on arm latch (carstate) and the panda's
    matching acc_main_on widening, controls_allowed stays up through brake
    cancels and CANCEL standby - permission alone no longer implies a live
    session. The gate follows the vendor op_byd decompile: the radar's own
    bus-2 frame must show BOTH AccControlActive AND AccReqNotStandstill -
    the vendor arms exactly on AccReqNotStandstill ("ACC requesting control,
    not a standstill hold"), which additionally blocks the standstill-hold
    frames that the old AccControlActive-only gate let through. While the
    gate is down (standby / CANCEL / standstill / driver braking) OP
    re-broadcasts the radar frame verbatim instead of commanding.

    The SNG spoofed-RES auto-resume that used to sit next to this method is
    GONE (user decision 2026-10-05: "纵向要激活 ACC 才能控制，设置不应该
    控制"); the vendor gates its own resume injection behind the BYDSnG
    param, which we deliberately left unimplemented - resuming is the
    driver's stalk action.

    Vendor command pipeline (see bydcan.create_accel_command): the plan jerk
    (CC.actuators.jerk, carrot publishes it from long_plan.jTargetNow) drives
    the JerkUpper/Lower envelope, the pre-brake catch-up, the minimal-brake
    behavior bit and the standstill/resume bits. Continuity across the
    gate's yields is kept by accel_cmd_sent: the slew limit ramps from the
    value actually decoded back off our own wire."""
    if self.frame % 2 == 0:
      raw_cnt = (self.frame // 2) % 16
      lcs = CC.actuators.longControlState
      stopping = lcs == LongCtrlState.stopping
      starting = lcs == LongCtrlState.starting
      resume = CC.cruiseControl.resume or starting
      radar = CS.radar_acc_msg
      session_active = bool(radar.get("AccControlActive", 0)) and \
        bool(radar.get("AccReqNotStandstill", 0))
      long_active = CC.longActive and not CS.out.brakePressed and session_active
      # vendor: over the set speed (cluster scale) -> ask the VCU for at most
      # a coast/brake and raise the minimal-brake behavior bit. Latched with
      # hysteresis: the 2026-10-05 second drive (1154-1187 s) showed the bare
      # threshold chattering on 0.1-0.2 m/s cluster noise right at the cruise
      # target, and the old post-slew clamp snapped the wire between the
      # demand and 0.00 at ~2 Hz - the sustained 忽加忽刹 sawtooth. ±0.15
      # deadband spreads the cycle out; the clamp below is ramped by the slew.
      MB_ON, MB_OFF = 0.15, 0.15
      if self.min_brake_latched:
        if CS.out.vEgoCluster <= CS.out.cruiseState.speed - MB_OFF or CS.out.vEgoCluster <= 2.5:
          self.min_brake_latched = False
      elif CS.out.vEgoCluster >= CS.out.cruiseState.speed + MB_ON and CS.out.vEgoCluster > 2.5:
        self.min_brake_latched = True
      minimal_brake = self.min_brake_latched

      if long_active:
        demand = CC.actuators.accel
        # no set speed -> never accelerate. Routes 31/2d show the engage frame
        # is (AccControlActive=1, SetSpeed=0) - the radar ramps to its own
        # stored target while OP's plan has no initialized cruise speed, and
        # echoing its AccelCmd is what the driver reads as "速度没设置就一直
        # 往上加". Until SetSpeed lands (1-2 s later in every trace) command
        # hold-or-brake only; the positive half of our accel is clamped.
        if int(CS.adas_msg.get("SetSpeed", 0) if CS.adas_msg else 0) == 0:
          demand = min(demand, 0.0)
        # minimal-brake accel clamp lives HERE (ahead of the slew), not in
        # create_accel_command: crossing the set speed must ramp the demand
        # to coast/brake, not snap it there - bydcan still raises the
        # BrakeBehaviour=1 behavior bit from the same latched flag.
        if minimal_brake:
          demand = min(demand, 0.0)
        # vendor pre-brake catch-up, applied to the DEMAND so the slew limit
        # below ramps it instead of stepping it (bydcan once did this after
        # shaping - the 2026-10-05 roadtest showed why: carrot's plan jerk
        # (jTargetNow) dips below -0.25 on ordinary sign noise, and a moment
        # of powertrain aEgo overshoot then slammed the wire from +0.95 to
        # -0.10 in one frame, ~22 times per 90 s of following - the 顿挫
        # complaint). The guard demand <= 0.0 keeps the vendor's own shape:
        # its planner only ever emits deep negative jerk while already
        # commanding deceleration; a positive accel + negative jerk pair is
        # carrot model jitter, not an under-deceleration to punish.
        jerk_cmd = CC.actuators.jerk
        if (not stopping and not starting and jerk_cmd <= -0.25 and demand <= 0.0 and
                CS.out.aEgo - demand > 0.2):
          demand = min(demand, 0.2 * jerk_cmd)
        # slew limit against what is actually on the wire (which, through a
        # yield, was the radar's own command) - see ACCEL_SLEW_* above
        lo = self.accel_cmd_sent - ACCEL_SLEW_DOWN * 0.02
        hi = self.accel_cmd_sent + ACCEL_SLEW_UP * 0.02
        accel = max(lo, min(hi, demand))
      else:
        # echo path: transmit the radar's own frame unchanged
        accel = radar.get("AccelCmd", 0.0) if radar else 0.0

      sent = bydcan.create_accel_command(
        self.packer, accel, CC.enabled, long_active, resume, radar, raw_cnt,
        cruise_standstill=bool(CS.out.cruiseState.standstill), stopping=stopping,
        starting=starting, minimal_brake=minimal_brake, a_ego=CS.out.aEgo,
        jerk=CC.actuators.jerk)
      can_sends.append(sent)
      # the exact value the VCU will act on (post shaping + 0.05/-5
      # quantization, read back from our own frame) is the ramp reference
      self.accel_cmd_sent = float(sent[1][0]) * 0.05 - 5.0
      if CS.adas_msg:
        can_sends.append(bydcan.create_acc_hud_command(self.packer, CS.adas_msg, raw_cnt))
      if CS.aeb_msg:
        can_sends.append(bydcan.create_acc_aeb_command(self.packer, CS.aeb_msg, raw_cnt))

  def update(self, CC, CC_SP, CS, now_nanos):
    can_sends = []

    # Transmit the spoofed 0x316 at 50 Hz UNCONDITIONALLY (engaged or not).
    # The camera's own 0x316 is blocked from relaying bus2->bus0 by the
    # firmware fwd hook, so OP is the EPS's only 0x316 source - if we stop
    # transmitting while disengaged, the EPS LKAS subsystem starves and
    # faults the ADAS domain ('check multifunction video controller', EPS
    # SteerWarning latched in real-vehicle logs). Idle frames echo the
    # camera's fields with LKAS_Active=0 / torque 0. The old claim that
    # continuous TX faults the domain only applied to the pre-echo
    # from-scratch frames (missing SETME_* fields). Matches the
    # community-verified BYD_Files controller, which transmits every cycle
    # regardless of engagement.
    steer_send = None
    new_actuators = CC.actuators.as_builder()
    if USE_ANGLE_STEERING:
      if CC.enabled or CC.latActive:
        steer_send = self._update_angle_lateral(CC, CS)
        if steer_send is not None:
          new_actuators.steeringAngleDeg = self.apply_angle_last
    else:
      steer_send = self._update_torque_lateral(CC, CS)
      new_actuators.torque = self.apply_torque_last / CarControllerParams.STEER_MAX
      new_actuators.torqueOutputCan = float(self.apply_torque_last)

    if steer_send is not None:
      can_sends.append(steer_send)

    if self.CP.openpilotLongitudinalControl:
      self._update_longitudinal(CC, CS, can_sends)

    self.frame += 1
    return new_actuators, can_sends
