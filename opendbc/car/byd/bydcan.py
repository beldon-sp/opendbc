import numpy as np

from opendbc.car.byd.values import CarControllerParams

_ACCEL_MIN = CarControllerParams.ACCEL_MIN
_ACCEL_MAX = CarControllerParams.ACCEL_MAX
_COMFORT_BAND_UPPER = CarControllerParams.COMFORT_BAND_UPPER
_COMFORT_BAND_LOWER = CarControllerParams.COMFORT_BAND_LOWER
_MIN_START_ACCEL = CarControllerParams.MIN_START_ACCEL


def byd_checksum(byte_key, dat):
  # not an actual known crc function, reverse engineered on Atto 3 (bukapilot);
  # TODO(Song Plus DM-i): verify against real vehicle CAN logs
  def byte_crc4_linear_inverse(byte_list):
    return (-1 * sum(byte_list) + 0x9) & 0xF

  second_bytes = [byte & 0xF for byte in dat]
  remainder = sum(second_bytes) >> 4
  second_bytes.append(byte_key >> 4)

  first_bytes = [byte >> 4 for byte in dat]
  first_bytes.append(byte_key & 0xF)

  return (((byte_crc4_linear_inverse(first_bytes) + (-1 * remainder + 5)) << 4) + byte_crc4_linear_inverse(second_bytes)) & 0xFF


def byd_checksum_xor(dat):
  # XOR checksum used on ACC_EPS_STATE (from the open BYD port)
  ret = 0
  for byte in dat:
    ret ^= byte
  return ret


# Fields echoed from the stock DiPilot camera's ACC_MPC_STATE so OP's spoofed
# frame keeps the same SETME_* / MPC_State / AutoFullBeam* values the ADAS
# domain expects. Building the message from scratch (leaving these as 0)
# faults the camera ('check multifunction video controller' + 'ACC restricted').
_ACC_MPC_STATE_ECHO_FIELDS = [
  "AutoFullBeamState", "LeftLaneState", "LKAS_Config", "SETME2_0x1",
  "MPC_State", "AutoFullBeam_OnOff", "LKAS_Output", "LKAS_Active",
  "SETME3_0x0", "TrafficSignRecognition_OnOff", "SETME4_0x0",
  "SETME5_0x1", "RightLaneState", "LKAS_State",
  "TrafficSignRecognition_Result", "LKAS_AlarmType", "SETME7_0x3",
]


def create_lkas_request(packer, cam_msg, apply_torque, lkas_active, lkas_req_prepare, lkas_config, raw_cnt):
  """50 Hz, spoofed ACC_MPC_STATE (790) carrying the LKAS torque request.

  Echoes the stock camera's ACC_MPC_STATE fields (SETME_*, MPC_State,
  AutoFullBeam*, LKAS_State) and overrides the torque request and lane
  states. Verified against real-vehicle CAN logs:
  - LeftLaneState/RightLaneState MUST be 2 or the EPS will not actuate LKA.
  - LKAS_Config MUST be 2 (LKA) whenever LeftLaneState=2 or LKAS_ReqPrepare=1:
    the stock camera idles at LKAS_Config=1 (ALARM), and EPS treats the
    combination Config=1 + LaneState=2 as illegal and latches SteerWarning
    (refusing to arm LKAS_Prepared) - which also drives the MPC's 'check
    multifunction video controller' fault via ACC_EPS_STATE.
  - LKAS_ReqPrepare=1 must be sent while engaged until the EPS reports
    LKAS_Prepared (ACC_EPS_STATE bit0).
  """
  values = {s: cam_msg[s] for s in _ACC_MPC_STATE_ECHO_FIELDS if s in cam_msg}
  values["ReqHandsOnSteeringWheel"] = 0
  values["LKAS_ReqPrepare"] = lkas_req_prepare
  values["LKAS_Config"] = lkas_config   # the vendor streams 3 in every state (idle included)
  values["Counter"] = raw_cnt

  if lkas_active:
    values.update({
      "LKAS_Output": apply_torque,   # steer torque request
      "LKAS_Active": 1,
      # The actuation gate is LKAS_Config=3 (ALARM_AND_LKA session), NOT the
      # state field: the vendor build (op_byd, route 00000037) steers with
      # exactly (State=1, MPC=0, Config=3, Active=1, lanes 2/2).
      "LKAS_State": 1,
      "LeftLaneState": 2,
      "RightLaneState": 2,
    })
  elif lkas_req_prepare:
    # engage/retry burst: the vendor's 3-frame prepare carries lanes 2/2 with
    # Active=0 and request 0 (route 00000037 t=16.78), then activates 50 ms
    # later without waiting for the EPS ack
    values.update({
      "LKAS_Output": 0,
      "LKAS_Active": 0,
      "LKAS_State": 1,
      "LeftLaneState": 2,
      "RightLaneState": 2,
    })
  else:
    values.update({
      "LKAS_Output": 0,
      "LKAS_Active": 0,
      # vendor standby: Config=3 with lanes 0/0 (no LKA target) - streamed
      # for minutes at a time on the vendor's own drive without EPS complaint
      "LKAS_State": 1,
      "LeftLaneState": 0,
      "RightLaneState": 0,
    })

  dat = packer.make_can_msg("ACC_MPC_STATE", 0, values)[1]
  crc = byd_checksum(0xAF, dat[:-1])
  values["CheckSum"] = crc
  return packer.make_can_msg("ACC_MPC_STATE", 0, values)


def send_buttons(packer, count):
  """Spoof ACC UP_RESETSPEED button press (BTN_AccUpDown_Cmd=3).

  UNUSED since 2026-10-05: the SNG auto-resume that called this could activate
  the stock ACC session while the driver had only set it ("设置不应该控制" -
  longitudinal must require the driver's own ACC activation). Kept as the
  frame reference for the fw's still-permitted standstill button window."""
  values = {
    "SETME_1": 1,
    "BTN_AccUpDown_Cmd": 3,
    "SETME2_1": 1,
    "Counter": count,
  }

  dat = packer.make_can_msg("PCM_BUTTONS", 0, values)[1]
  crc = byd_checksum(0xAF, dat[:-1])
  values["CheckSum"] = crc
  return packer.make_can_msg("PCM_BUTTONS", 0, values)


def create_fake_eps_state(packer, eps_state_msg, fake_torque, counter):
  """Spoof ACC_EPS_STATE (792) EPS feedback so the ADAS domain does not fault (no DTC).

  Not sent by default; only needed if the real EPS feedback is not visible to the
  ADAS domain with the harness in place."""
  values = {s: eps_state_msg[s] for s in [
    "LKAS_Prepared",
    "CruiseActivated",
    "TorqueFailed",
    "SETME1_0x1",
    "SteerWarning",
    "SteerErrorCode",
    "SETME3_0x1",
    "SETME4_0x3",
    "SETME5_0xFF",
    "SETME6_0xFFF",
  ]}
  values["MainTorque"] = fake_torque
  values["SteerDriverTorque"] = 0
  values["ReportHandsNotOnSteeringWheel"] = 0
  values["Counter"] = counter

  dat = packer.make_can_msg("ACC_EPS_STATE", 0, values)[1]
  values["CheckSum"] = byd_checksum_xor(dat[:-1])
  return packer.make_can_msg("ACC_EPS_STATE", 0, values)


def create_can_steer_command(packer, steer_angle, steer_req, is_standstill, raw_cnt):
  """50 Hz, spoofed STEERING_MODULE_ADAS (482) angle command. Experimental angle path only."""
  set_me_xe = 0xB
  if is_standstill:
    set_me_xe = 0xE

  values = {
    "STEER_REQ": steer_req,
    "STEER_REQ_ACTIVE_LOW": not steer_req,
    "STEER_ANGLE": steer_angle * 1.02,     # desired steer angle
    "SET_ME_X01": 0x1 if steer_req else 0,  # must be 0x1 to steer
    "SET_ME_XE": set_me_xe if steer_req else 0,  # 0xB faults lesser, maybe higher value faults lesser,
                                            # 0xB also seems to have the highest angle limit at high speed
    "COUNTER": raw_cnt,
    "SET_ME_FF": 0xFF,
    "SET_ME_F": 0xF,
    "SET_ME_1_1": 1,
    "SET_ME_1_2": 1,
  }

  dat = packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)[1]
  crc = byd_checksum(0xAF, dat[:-1])
  values["CHECKSUM"] = crc
  return packer.make_can_msg("STEERING_MODULE_ADAS", 0, values)


def create_accel_command(packer, accel, enabled, active, resume, radar_acc_msg, raw_cnt,
                         cruise_standstill=False, stopping=False, starting=False,
                         minimal_brake=False, a_ego=0.0, jerk=0.0):
  """50 Hz, transparent ACC_CMD (814) replacement for OP longitudinal control.

  Rebuilt to match the vendor op_byd decompile (bydcan.create_accel_command,
  see docs notes 2026-10-05): the frame is always echoed from the radar's own
  bus-2 ACC_CMD, and while OP is engaged the acceleration envelope fields are
  re-derived from the plan every command frame instead of sending fixed jerk
  constants - that jerk envelope is the vendor's longitudinal smoothness
  mechanism (the VCU executes our accel *at* the commanded jerk rate).

  Vendor formulae reproduced (radar signal names -> our dbc names):
    ACCEL_CMD        -> AccelCmd      (0.05,-5 quantization, raw 20..140)
    ACC_CONTROLLABLE_AND_ON -> AccControlActive
    ACC_REQ_NOT_STANDSTILL  -> AccReqNotStandstill
    STANDSTILL_RESUME -> ResumeFromStandstill, STANDSTILL_STATE -> StandstillState,
    BRAKE_BEHAVIOR -> BrakeBehaviour (0 free / 1 minimal-brake / 2 stopping),
    ESP_BEHAVIOR -> EspBehaviour, ACC_OVERRIDE_OR_STANDSTILL -> AccOverrideOrStandstill

  - should_send_resume (AND chain, vendor): a resume request only while the
    car holds a standstill that the radar no longer claims - it bumps the
    command to max(0.2, accel) to roll from stop.
  - should_send_standstill: hold-stop bits set only when not resuming and
    (long controller is stopping or the car reports cruise standstill).
  - catch-up: deep plan jerk (<=-0.25) while the car decelerates LESS than
    commanded -> accel = min(accel, 0.2*jerk), a bounded pre-brake - applied
    by the caller BEFORE the slew limiter (carcontroller), not here.
  - jerk budget: jerk = min(min(0.3*can_accel, (accel-a_ego)/0.1), plan jerk);
    envelope upper np.clip(jerk, 1, 12), lower np.clip(jerk, -4, -0.8);
    stopping overrides to 2 / -4.
  - minimal_brake (over target speed): BrakeBehaviour=1 (the accel
    floor clamp is applied by the caller ahead of the slew limiter).
  """
  if radar_acc_msg:
    # echo the radar's frame as the base (StandstillState / BrakeBehaviour /
    # EspBehaviour etc. are the radar's own values)
    values = {s: radar_acc_msg[s] for s in (
      "AccelCmd", "ComfortBandUpper", "ComfortBandLower", "SETME1_0x1",
      "JerkUpperLimit", "ResumeFromStandstill", "JerkLowerLimit",
      "StandstillState", "BrakeBehaviour", "AccReqNotStandstill",
      "AccControlActive", "AccOverrideOrStandstill", "EspBehaviour")}
  else:
    # no radar frame seen yet (ACC never on): synthesized idle frame, the
    # all-zero control-bits shape from route 37's byte5=0x00 population
    values = {
      "AccelCmd": 0.0, "ComfortBandUpper": 0.0, "ComfortBandLower": 0.0,
      "SETME1_0x1": 1, "JerkUpperLimit": 0.0, "ResumeFromStandstill": 0,
      "JerkLowerLimit": 0.0, "StandstillState": 0, "BrakeBehaviour": 0,
      "AccReqNotStandstill": 0, "AccControlActive": 0,
      "AccOverrideOrStandstill": 0, "EspBehaviour": 0,
    }

  if enabled and active:
    can_accel = float(np.clip(accel, _ACCEL_MIN, _ACCEL_MAX))
    radar_accel = radar_acc_msg.get("AccelCmd", 0.0) if radar_acc_msg else 0.0
    radar_standstill_state = bool(radar_acc_msg.get("StandstillState", 0)) if radar_acc_msg else False

    # vendor pre-brake catch-up (jerk-deep + car under-decelerating ->
    # accel = min(accel, 0.2*jerk)) MOVED to carcontroller._update_longitudinal,
    # ahead of the slew limiter, guarded by demand <= 0. The 2026-10-05
    # roadtest proved why: applied here it ran AFTER the slew and could step
    # the wire ±1 m/s^2 in one frame on carrot jerk sign noise.

    should_send_resume = resume and cruise_standstill and radar_accel > 0 and not radar_standstill_state
    should_send_standstill = (not should_send_resume) and (stopping or cruise_standstill)

    # vendor jerk budget: never demand a jerk the executed accel can't carry
    min_jerk = min(can_accel * 0.3, (can_accel - a_ego) / 0.1)
    jerk = min(min_jerk, jerk)
    jerk_upper = float(np.clip(jerk, 1, 12))
    jerk_lower = float(np.clip(jerk, -4, -0.8))
    if stopping and not should_send_resume:
      jerk_upper, jerk_lower = 2.0, -4.0

    brake_behavior = 0
    if not cruise_standstill:
      if minimal_brake:
        # accel floor clamp (demand -> <=0) moved to carcontroller, ahead of
        # the slew limiter - here we only raise the vendor behavior bit
        brake_behavior = 1
      if stopping:
        brake_behavior = 2
    if should_send_resume:
      can_accel = max(_MIN_START_ACCEL, can_accel)

    values.update({
      "AccelCmd": can_accel,
      "ComfortBandUpper": _COMFORT_BAND_UPPER if can_accel >= 0 else 0.0,
      "ComfortBandLower": _COMFORT_BAND_LOWER if can_accel >= 0 else 0.0,
      "JerkUpperLimit": jerk_upper,
      "JerkLowerLimit": jerk_lower,
      "AccControlActive": 1,
      "AccReqNotStandstill": 0 if should_send_standstill else 1,
      "AccOverrideOrStandstill": 1 if should_send_standstill else 0,
      "StandstillState": 1 if should_send_standstill else 0,
      "EspBehaviour": 1,
      "BrakeBehaviour": brake_behavior,
      "ResumeFromStandstill": 1 if should_send_resume else 0,
    })

  values["Counter"] = raw_cnt
  values["SETME2_0xF"] = 0xF

  dat = packer.make_can_msg("ACC_CMD", 0, values)[1]
  values["CheckSum"] = byd_checksum(0xAF, dat[:-1])
  return packer.make_can_msg("ACC_CMD", 0, values)


def create_acc_hud_command(packer, adas_msg, raw_cnt):
  """50 Hz, ACC_HUD_ADAS (813) relay: pure echo of the stock camera's HUD frame.

  The camera keeps streaming its HUD on bus 2 (it stays the session owner);
  OP re-broadcasts it onto bus 0 with only the counter/checksum re-stamped.
  Route 37 shows the vendor's HUD content matching the camera's state 1:1
  (AccState/AccOn1/Notify all follow, SETME3_0xFFF=0xFFF, Status=4)."""
  values = {s: adas_msg[s] for s in (
    "SetSpeed", "HasLead", "SetDistance", "LeadingDistance", "AEB", "FCW",
    "SETME1_0x1", "AccState", "AccOn1", "CloseWarning", "SETME2_0x1",
    "Notify", "Status", "SETME3_0xFFF")}
  values["Counter"] = raw_cnt
  values["SETME4_0xF"] = 0xF

  dat = packer.make_can_msg("ACC_HUD_ADAS", 0, values)[1]
  values["CheckSum"] = byd_checksum(0xAF, dat[:-1])
  return packer.make_can_msg("ACC_HUD_ADAS", 0, values)


def create_acc_aeb_command(packer, aeb_msg, raw_cnt):
  """50 Hz, ACC_AEB (815) heartbeat relay: static payload + our counter.

  Byte template from route 37: payload `05 80 02 0f ff ff` constant, counter
  in byte 6 low nibble, SETME4_0xF, byd_checksum. AEB itself stays with the
  stock radar."""
  values = {"PAYLOAD": aeb_msg["PAYLOAD"]}
  values["Counter"] = raw_cnt
  values["SETME4_0xF"] = 0xF

  dat = packer.make_can_msg("ACC_AEB", 0, values)[1]
  values["CheckSum"] = byd_checksum(0xAF, dat[:-1])
  return packer.make_can_msg("ACC_AEB", 0, values)
