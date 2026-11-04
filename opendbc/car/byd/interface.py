import math

import numpy as np

from opendbc.car import get_safety_config, structs
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.byd.carcontroller import CarController
from opendbc.car.byd.carstate import CarState
from opendbc.car.byd.radar_interface import RadarInterface
from opendbc.car.byd.values import HUD_MULTIPLIER, USE_ANGLE_STEERING, BydSafetyFlags, CarControllerParams


def sig(val: float) -> float:
  # vendor byd/interface.py (cp_byd pyarmor_decrypted, interface.py.disasm.txt L183):
  # clamp to +-700 to keep exp() finite, sigmoid re-centered so sig(0)==0
  val = max(min(val, 700.0), -700.0)
  return 1.0 / (1.0 + math.exp(-val)) - 0.5


def non_linear_torque(lateral_accel_value: float, v_ego: float,
                      f1: float, f2: float, f3: float, f4: float, f5: float, f6: float, f7: float) -> float:
  # vendor L380 (the 9-arg form - non_linear_torque_old at L269 is dead code). Stack-traced
  # against docs/byd-lateral-vendor-model.md:
  #   torque = f2 * sig(lat * uf2 * hsf * f1 * hsf * (1 - uf)) + lat * uf2 * v_ego / 10
  # uf/uf2 are left/right asymmetric: Song Plus computes ~-0.47 left vs +0.42 right at
  # |lat|=1 m/s2 @ 20 m/s, i.e. a structural pre-compensation for the road-crown pull.
  unbalanced_factor = f5 if lateral_accel_value < 0 else f5 * f6
  unbalanced_factor_2 = f3 if lateral_accel_value < 0 else f7
  # kph ramp 1.0 -> 1.5 across [f4, 2*f4]; with the max(8 m/s) floor the siglin entry passes
  # in, this is pinned to 1.5 in normal driving (model intentionally frozen below ~29 km/h)
  high_speed_factor = 1.0 + min(0.5, max(0.0, (v_ego * 3.6 - f4) / f4))
  sigmoid_term = f2 * sig(lateral_accel_value * unbalanced_factor_2 * high_speed_factor * f1 *
                          high_speed_factor * (1.0 - unbalanced_factor))
  linear_term = lateral_accel_value * unbalanced_factor_2 * v_ego / 10.0
  return sigmoid_term + linear_term


_lats_sm = None


def _read_lat_state() -> tuple[bool, float]:
  # vendor equivalent: module-level sm = SubMaster(...) on 'controlsState', read via
  # sm['controlsState'].lateralControlState (disasm L1287+). Returns (is_torqueState, i).
  # Any failure mode (no msgq, first frame, schema drift) must degrade to (False, 0.0) -
  # the curve term goes inert, the model keeps steering.
  global _lats_sm
  if _lats_sm is None:
    try:
      from cereal.messaging import SubMaster
      _lats_sm = SubMaster(['controlsState'])
    except Exception:
      _lats_sm = False
  if not _lats_sm:
    return False, 0.0
  try:
    _lats_sm.update(0)
    lat_state = _lats_sm['controlsState'].lateralControlState
    if lat_state.which() == 'torqueState':
      return True, float(lat_state.torqueState.i)
  except Exception:
    pass
  return False, 0.0


class CarInterface(CarInterfaceBase):
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    # curve term inputs, refreshed once per loop in apply() (see _read_lat_state):
    # the sunnypilot CarControl schema has NO actuators.lateralControlState - the vendor
    # reaches the lateral torque state through a SubMaster on controlsState (disasm
    # L1287+), we do the same. Cached here so the siglin callback stays allocation-free
    # (it runs several times per frame) and so tests can drive it directly.
    self._lat_state_is_torque = False
    self._lat_torque_i = 0.0

  def update(self, can_packets):
    ret, ret_sp = super().update(can_packets)
    # T8 gap monitor rides the bus1 parser owned by CarState (see the note in
    # carstate init: deliberately outside can_parsers, canValid/canTimeout are
    # NOT affected). Fields are telemetry until desire_helper_byd gates on them.
    # Guarded: update() shares card's never-raise contract with apply() - a crash
    # here gaps 0x316 and latches EPS TorqueFailed. The extra parser must not be
    # able to kill the loop; worst case the gap fields stay False.
    try:
      ret.radarGapClosing, ret.radarGapNew = self.CS.update_radar_gap(can_packets, ret.vEgo)
      # stage-3 lane discrimination (0=unknown/1=own/2=adjacent); telemetry
      # for the road test, and the gate-3 consumer in desire_helper_byd
      ret.radarGapLane = self.CS.gap_mon.lane
    except Exception:
      pass
    return ret, ret_sp

  def apply(self, c: structs.CarControl, c_sp: structs.CarControlSP, now_nanos: int | None = None) -> tuple[structs.CarControl.Actuators, list]:
    # ABSOLUTE rule on BYD: apply()/card must never raise - a gap in the 0x316 stream
    # latches EPS TorqueFailed until the next ignition cycle. Everything below is soft.
    try:
      self._lat_state_is_torque, self._lat_torque_i = _read_lat_state()
    except Exception:
      self._lat_state_is_torque = False
      self._lat_torque_i = 0.0
    return super().apply(c, c_sp, now_nanos)

  @staticmethod
  def _get_params(ret: structs.CarParams, candidate, fingerprint, car_fw, alpha_long, is_release, docs) -> structs.CarParams:
    ret.brand = "byd"
    if USE_ANGLE_STEERING:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd, BydSafetyFlags.ANGLE_STEERING)]
      ret.steerControlType = structs.CarParams.SteerControlType.angle
      ret.steerActuatorDelay = 0.01  # DiPilot steering request is applied almost instantly
    else:
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.byd)]
      ret.steerControlType = structs.CarParams.SteerControlType.torque
      # vendor live carParams (route 00000037): steerActuatorDelay=0.3
      ret.steerActuatorDelay = 0.3
      # A档 (vendor configure_torque_tune, interface.py.disasm L1953 + L2347):
      # - 0.1 deg steering-angle deadzone (Song Plus group default; latcontrol_torque converts
      #   it to a lateral-accel deadzone via the vehicle model every frame)
      # - ki=0: the vendor lateral PID has NO integrator (kp=kf=1 are already the base defaults).
      #   Their steady-state trimming is done by the model shaping (B档) and, on top of that,
      #   the C档 error-enhancer subsystem - not by the loop integrator.
      CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning, steering_angle_deadzone_deg=0.1)
      # vendor parity: the vendor lateral PID has NO integrator (ki=0). This fork's
      # torque PID carries its KI in the controller (not CP.lateralTuning.torque), so
      # the brand gate that zeroes it lives in latcontrol_torque (main process).
    ret.steerLimitTimer = 0.5

    # Radar interface LIVE (2026-10-19, alpha-longitudinal verification round;
    # ported 2026-10-20): primary synthesized lead = continuous 0x109 b7 *
    # BydRadarDistScale (pinned 0.25 m/count) with the presence==7 gate and
    # median-filtered vRel; 0x351-354 slot path stays as the auto-lighting
    # future source (private-CAN tap), 0x32D HUD bars as the last-resort
    # fallback (published measured=False on this fork, see radar_interface
    # _update_hud). In stock-ACC mode radarState feeds nothing longitudinal
    # (the vendor's MPC keeps owning the session), so this flag is
    # behavior-neutral there; with AlphaLongitudinalEnabled the synthesized
    # lead reaches the MPC only through this fork's vision-association matcher
    # (farther-than-vision veto + strongly-visible fallback - a bad lead is
    # dropped, not trusted; V2/V3 road-test items, docs/byd-radar-010x.md).
    # The gap monitor (BydRadarGapGate) is unaffected (counts space, own parser).
    ret.radarUnavailable = False

    # longitudinal control is done by the stock ACC; openpilot does lateral
    # control only (unless AlphaLongitudinalEnabled). The spoofed RES-button
    # auto-resume was REMOVED 2026-10-05 (user rule: 纵向要激活 ACC 才能控制,
    # 设置不应该控制) - OP must never activate the stock session by itself,
    # resuming from a standstill is the driver's action on the stalk.
    # pcmCruise ties openpilot's engagement to the ACC main posture (like the
    # Geely port): cruiseState.enabled is the arm latch (debounced main-on +
    # one genuine session this drive, see carstate) - OP enables on its rising
    # edge and only drops it when the driver turns ACC off. Brake and session
    # standby deliberately do not disengage; the longitudinal output is gated
    # on the live radar session in the controller instead.
    ret.openpilotLongitudinalControl = False
    ret.pcmCruise = True
    # true: a standstill must not fire STOP_REQUIRED-style disengagements
    # (the main-on latch keeps OP alive through the stop; the driver resumes)
    ret.autoResumeSng = True

    ret.wheelSpeedFactor = HUD_MULTIPLIER

    ret.minEnableSpeed = -1
    # note: this fork's CarParams has no stoppingDecelRate (deprecated group,
    # unused by its LongControl) - the stop ramp lives in LongControl itself

    # OP longitudinal (AlphaLongitudinalEnabled param): transparent replacement
    # of the stock radar's ACC_CMD/ACC_HUD/ACC_AEB on bus 0. The stock radar
    # keeps owning the session - its frames on bus 2 stay alive and keep feeding
    # cruiseState/pcm_cruise_check - so the engage chain above is unchanged and
    # no reflash is needed to toggle: the firmware keys its extra TX whitelist,
    # accel checks and 0x32D/E/F forward block off the LONGITUDINAL safety flag.
    # Declare availability: without this flag selfdrived deletes
    # AlphaLongitudinalEnabled at every startup and exp_button locks the onroad
    # toggle ("stock ACC is used" message). Running the param on firmware
    # predating the longitudinal port (fw_base fee17db6) is instead caught by
    # the safetyTxBlocked watchdog (bydLongFirmwareMissing, M1).
    ret.alphaLongitudinalAvailable = True
    if alpha_long:
      ret.openpilotLongitudinalControl = True
      ret.safetyConfigs[0].safetyParam |= int(BydSafetyFlags.LONGITUDINAL)
      # vendor interface params (decrypted op_byd interface.py, 2026-10-05
      # full-pipeline decompile): the vendor sets these once for ALL BYD with
      # op-long. This fork's CarParams has no vEgoStopping/vEgoStarting/
      # startAccel/stoppingDecelRate/startingState (deprecated group, unused by
      # its LongControl - which also has no `starting` state; the standstill ->
      # go transition rides the PID state and cruiseControl.resume instead), so
      # only the actuator delay carries over.
      ret.longitudinalActuatorDelay = 0.4

    return ret

  @staticmethod
  def get_pid_accel_limits(CP, CP_SP, current_speed, cruise_speed):
    # vendor ACCEL_MIN/MAX (base class returns -3.5/2.0; BYD commands down to
    # -4.0 m/s2, see route 37 TX)
    return CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX

  # --- B档: vendor siglin lateral model (interface.py.disasm L1556/L1848, see
  # docs/byd-lateral-vendor-model.md). latcontrol_torque binds CI.torque_from_lateral_accel()
  # once at startup, so the selector below is the only hook needed. ---

  def torque_from_lateral_accel(self):
    if self.CP.carFingerprint in CarControllerParams.NON_LINEAR_TORQUE_PARAMS:
      return self.torque_from_lateral_accel_siglin
    return super().torque_from_lateral_accel()

  def get_adjusted_lateral_accel(self, lateral_accel_value: float,
                                 torque_params: structs.CarParams.LateralTorqueTuning,
                                 steer_mode: bool, roll_compensation: float,
                                 gravity_adjusted: bool) -> float:
    # vendor L1218. Two shapers on the commanded lateral accel, applied to every callback
    # input (setpoint, measurement, and feedforward):
    # 1) a gain around the latAccelFactor nominal (Song Plus: 2.5 -> exactly 1.0) plus
    #    +0.3 while the EPS is NOT in an active steering session (their STEER_MODE in (1,2);
    #    we approximate with the 0x318 CruiseActivated bit - see docs);
    # 2) a curve taper keyed on the PID integrator log (torqueState.i). With the vendor's
    #    ki=0 it is always 0 there; keeping it wired means it engages if the user raises ki
    #    or turns on the NNFF integrator on carrot.
    adjusted = lateral_accel_value
    if gravity_adjusted:
      adjusted += roll_compensation
    curve_offset = 0.0
    if self._lat_state_is_torque:
      curve_offset = float(np.interp(abs(adjusted), [0.0, 0.2], [0.2, 1.0])) * self._lat_torque_i
    lat_conv_factor = 1.0 - (torque_params.latAccelFactor - 2.5)
    if not steer_mode:
      lat_conv_factor += 0.3
    lat_conv_factor *= (1.0 - curve_offset) if adjusted > 0 else (1.0 + curve_offset)
    return adjusted * lat_conv_factor

  def torque_from_lateral_accel_siglin(self, lateral_acceleration: float,
                                       torque_params: structs.CarParams.LateralTorqueTuning) -> float:
    # This fork's torque callback contract is 2-arg (lateral_acceleration, torque_params):
    # latcontrol_torque adds the friction curve in lateral-accel space OUTSIDE the
    # callback, and roll/gravity shaping happens on its feedforward before the PID, so
    # neither is re-applied here (gravity_adjusted=False semantics). The remaining
    # vendor inputs are recovered from CS telemetry: v_ego (model speed floor) and
    # steer_mode (CruiseActivated bit).
    lateral_accel_value = lateral_acceleration
    # vendor floors the model speed at 8 m/s (the siglin entry, disasm L110 region): below
    # ~29 km/h the nonlinear terms stop changing with speed
    v_ego = max(8.0, self.CS.out.vEgo)
    params = CarControllerParams.NON_LINEAR_TORQUE_PARAMS.get(self.CP.carFingerprint)
    assert params is not None, 'The params are not defined'
    f1, f2, f3, f4, f5, f6, f7 = params
    # steer_mode: vendor's STEER_MODE msg (1 Hz) is absent from our dbc - CruiseActivated is
    # the closest live bit (True == EPS steering == their in(1,2)); default True so a missing
    # carstate attribute falls back to NO +0.3 boost, matching their in-session behavior
    steer_mode = bool(getattr(self.CS, "cruise_activated", True))
    adjusted = self.get_adjusted_lateral_accel(lateral_accel_value, torque_params,
                                               steer_mode, 0.0, False)
    return float(non_linear_torque(adjusted, v_ego, f1, f2, f3, f4, f5, f6, f7))
