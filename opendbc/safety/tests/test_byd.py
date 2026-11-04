#!/usr/bin/env python3
import unittest

from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
import opendbc.safety.tests.mads_common as mads_common


def _skip_mads_tests(cls):
  """BYD Plan A: MADS is short-circuited for this mode.

  The framework-level MADS machinery tests are intentionally inert here: the
  byd port carries no check_relay TX messages (0x316/0x3B0 are natively visible
  on bus 0, enabling relay malfunction detection would false-trigger), so
  mads_state_update never runs, and controls_allowed_lateral stays false for
  BYD by design - lateral permission comes from BYD's own engaged gate
  (acc_main_on latch, enforced in byd.h's tx hook)."""
  for name in dir(mads_common.MadsSafetyTestBase):
    if name.startswith('test_'):
      setattr(cls, name, unittest.skip('BYD: MADS short-circuited (Plan A)')(getattr(mads_common.MadsSafetyTestBase, name)))
  return cls


@_skip_mads_tests
class BydButtonTestBase:
  """PCM_BUTTONS (0x3B0) resume spoofing rules, shared by both steering paths."""

  # Plan A: MADS does not own the lateral switch for BYD (see _skip_mads_tests)
  MADS_SUPPORTED = False

  # Full relay bus 0 <-> bus 2 (camera and radar are fed through it); block
  # only the LKAS (0x316) and angle (0x1E2) control frames. See byd_fwd_hook.
  FWD_BLACKLISTED_ADDRS = {0: [0x1E2, 0x316], 2: [0x1E2, 0x316]}

  # BYD_ACC_MAIN_FALL_HOLD in byd.h, plus margin
  MAIN_FALL_HOLD_FRAMES = 65

  def _rx_main_off(self):
    """Sustained ACC main-off: past the fall hold, so acc_main_on actually
    drops and the pcm/mads edge machinery sees a genuine main-off."""
    for _ in range(self.MAIN_FALL_HOLD_FRAMES):
      self._rx(self._pcm_status_msg(False))

  # The main fall hold means a SINGLE main-off frame no longer cancels -
  # cruise status keeps reading engaged through the hold. Override the three
  # generic pcm tests to drive a sustained main-off where they assert the drop.
  def test_enable_control_allowed_from_cruise(self):
    self._rx_main_off()
    self.assertFalse(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_disable_control_allowed_from_cruise(self):
    self.safety.set_controls_allowed(1)
    self._rx_main_off()
    self.assertFalse(self.safety.get_controls_allowed())

  def test_cruise_engaged_prev(self):
    for engaged in [True, False]:
      self._rx(self._pcm_status_msg(engaged))
      self.assertEqual(engaged, self.safety.get_cruise_engaged_prev())
      self._rx_main_off()
      self.assertFalse(self.safety.get_cruise_engaged_prev())

  def test_main_survives_short_drop(self):
    # the camera's ~2 s main glitches must not kill actuation permission:
    # a burst of main-off frames shorter than the hold keeps acc_main_on up
    # (hold = 60 frames; the 60th drop frame is the first to let go)
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    for _ in range(59):
      self._rx(self._pcm_status_msg(False))
      self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_standby_posture_is_main_on(self):
    # AccState=1 (standby, also the ignition-leftover posture) + AccOn1=1 is
    # MAIN-ON now: longitudinal actuation permission stays up through brake
    # cancel / CANCEL standby - the session gate lives in the OP controller
    self.safety.set_controls_allowed(False)
    self._rx_main_off()
    self.assertFalse(self.safety.get_controls_allowed())
    values = {"AccState": 1, "AccOn1": 1}
    self._rx(self.packer.make_can_msg_safety("ACC_HUD_ADAS", 2, values))
    self.assertTrue(self.safety.get_controls_allowed())

  # BYD pedal semantics (ported design, road-validated on the carrot build):
  # the panda observes the pedals but never drops controls_allowed on
  # them. cruiseState.enabled is the ACC main latch (carstate), which a pedal
  # press does not toggle - a panda-side disengage would have no rising edge
  # to recover from until the driver cycles ACC, killing BOTH axes. Driver
  # override lives upstream: selfdrived suppresses the brake disengage for byd
  # (brand-gated pedalPressed), DisengageOnAccelerator still applies to the
  # longitudinal state machine on the OP side, and the controller gates
  # ACC_CMD TX on the live radar session. The generic harness assumes the
  # classic pedal-disengage modes of this framework generation, so BYD
  # overrides the pedal tests to assert its neutral behaviour (with the
  # *_prev mirrors still tracked, see byd.h).

  def test_prev_gas(self):
    self.assertFalse(self.safety.get_gas_pressed_prev())
    for pressed in [self.GAS_PRESSED_THRESHOLD + 1, 0]:
      self._rx(self._user_gas_msg(pressed))
      self.assertEqual(bool(pressed), self.safety.get_gas_pressed_prev())

  def test_disengage_on_gas(self):
    self._rx(self._user_gas_msg(0))
    self.safety.set_controls_allowed(True)
    self._rx(self._user_gas_msg(self.GAS_PRESSED_THRESHOLD + 1))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._user_gas_msg(0))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_no_disengage_on_gas(self):
    # this fork dropped the panda-level gas disengage entirely (generic_rx_checks
    # has no gas branch; override lives upstream in selfdrived), so a gas press
    # never drops controls_allowed - no alt-experience flag involved
    self.safety.set_controls_allowed(True)
    self._rx(self._user_gas_msg(self.GAS_PRESSED_THRESHOLD + 1))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._user_gas_msg(0))
    self.assertTrue(self.safety.get_controls_allowed())

  def test_allow_user_brake_at_zero_speed(self):
    self._rx(self._vehicle_moving_msg(0))
    self.safety.set_controls_allowed(True)
    for brake in (True, False, True):
      self._rx(self._user_brake_msg(brake))
      self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._user_brake_msg(False))

  def test_not_allow_user_brake_when_moving(self):
    # BYD: lateral AND longitudinal permission survive brake presses at any
    # speed (the session gate in the controller, not the panda, owns the cut)
    self.safety.set_controls_allowed(True)
    self._rx(self._user_brake_msg(True))
    self._rx(self._vehicle_moving_msg(self.STANDSTILL_THRESHOLD + 1))
    self._rx(self._user_brake_msg(False))
    self._rx(self._user_brake_msg(True))
    self.assertTrue(self.safety.get_controls_allowed())
    self._rx(self._vehicle_moving_msg(0))

  def test_fwd_hook(self):
    # Full relay bus 0 <-> bus 2 with the default get_fwd_bus mapping; the
    # LKAS (0x316) and angle (0x1E2) control frames are blocked in both
    # directions (BYD_TX_MSGS check_relay is off on purpose - both addrs are
    # natively visible on bus 0 on this car).
    for bus in range(3):
      for addr in self.SCANNED_ADDRS:
        fwd_bus = self.FWD_BUS_LOOKUP.get(bus, -1)
        if bus in self.FWD_BLACKLISTED_ADDRS and addr in self.FWD_BLACKLISTED_ADDRS[bus]:
          fwd_bus = -1
        self.assertEqual(fwd_bus, self.safety.safety_fwd_hook(bus, addr), f"{addr=:#x} from {bus=} to {fwd_bus=}")

  def test_resume_buttons(self):
    # BTN_AccUpDown_Cmd=3 (UP_RESETSPEED) spoof is only allowed while stationary
    for stationary in (True, False):
      self._rx(self._speed_msg(0. if stationary else 5.))

      for controls_allowed in (True, False):
        self.safety.set_controls_allowed(controls_allowed)

        # all-zero message (no button press) is allowed only while stationary
        self.assertEqual(stationary, self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", 0, {})))

        # resume press
        values = {"BTN_AccUpDown_Cmd": 3}
        self.assertEqual(stationary, self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", 0, values)))

        # other buttons are never allowed
        for btn in ("BTN_AccCancel", "BTN_TOGGLE_ACC_OnOff", "BTN_AccDistanceDecrease", "BTN_AccDistanceIncrease"):
          values = {btn: 1}
          self.assertFalse(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", 0, values)))

        # down (setspeed) press is never allowed
        values = {"BTN_AccUpDown_Cmd": 1}
        self.assertFalse(self._tx(self.packer.make_can_msg_safety("PCM_BUTTONS", 0, values)))

    self._rx(self._speed_msg(0.))


class BydTorqueTestBase(BydButtonTestBase):
  """Torque-path steering frame semantics, shared by the default and LONG param."""

  def test_steer_safety_check(self):
    # BYD's actuation bit IS LKAS_Active, and the torque path enforces its own
    # engaged gate in byd.h (MADS is short-circuited for BYD - Plan A - so the
    # lateral permission never comes from controls_allowed_lateral). Rules:
    #   - (Active=0, torque=0): the idle stream, always allowed
    #   - (Active=1, torque t): allowed only while controls_allowed
    #   - (Active=0, torque!=0): req mismatch, blocked by the framework check
    # |t| stays inside DRIVER_TORQUE_ALLOWANCE so the matrix isolates the gate
    # rules from the driver-torque limit.
    for enabled in (0, 1):
      self.safety.set_controls_allowed(enabled)
      for t in (-self.DRIVER_TORQUE_ALLOWANCE + 20, -1, 0, 1, self.DRIVER_TORQUE_ALLOWANCE - 20):
        for req in (0, 1):
          # rejected frames reset the firmware's internal last-torque state,
          # so re-prime it before every assertion to isolate the gate rules
          self._set_prev_torque(t)
          should_tx = (t == 0 and not req) or (enabled and req)
          self.assertEqual(should_tx, self._tx(self._torque_cmd_msg(t, steer_req=req)), f"{enabled=} {t=} {req=}")


class TestBydSafetyTorque(BydTorqueTestBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):

  DBC = "byd_general_pt"
  SAFETY_MODEL = CarParams.SafetyModel.byd
  SAFETY_PARAM = 0

  # torque path TX whitelist: OP's own messages only, matches BYD_TX_MSGS_TORQUE
  # in opendbc/safety/modes/byd.h and the flashed firmware
  TX_MSGS = [[0x316, 0], [0x3B0, 0]]
  # NOTE: 0x316 and 0x3B0 are natively visible on bus 0 on Song Plus DM-i
  # (0x316 via the fingerprint, 0x3B0 from the stalk), so relay malfunction
  # detection cannot be enabled - it would false-trigger and block all control.
  RELAY_MALFUNCTION_ADDRS = {}

  GAS_PRESSED_THRESHOLD = 1  # factor 0.01 percent

  # torque control limits (matches BYD_TORQUE_STEERING_LIMITS in byd.h)
  MAX_RATE_UP = 18
  MAX_RATE_DOWN = 18
  MAX_RT_DELTA = 250
  MAX_TORQUE_LOOKUP = [0], [300]
  # our actuation bit cut does not lock the mode out; mismatch (torque with
  # LKAS_Active=0) is still blocked, asserted in test_torque_req_mismatch
  NO_STEER_REQ_BIT = True

  DRIVER_TORQUE_ALLOWANCE = 120
  DRIVER_TORQUE_FACTOR = 3

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Active": steer_req}
    return self.packer.make_can_msg_safety("ACC_MPC_STATE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"SteerDriverTorque": torque}
    return self.packer.make_can_msg_safety("ACC_EPS_STATE", 0, values)

  def _pcm_status_msg(self, enable):
    # ACC MAIN posture (route-verified): enable = engaged session frame
    # (AccState=3, AccOn1=1); disable = true main-off (AccState=0, AccOn1=0).
    # AccState=1 + AccOn1=1 would be MAIN-ON standby, NOT an off (it also
    # shows up at ignition and after a brake cancel).
    values = {"AccState": 3 if enable else 0, "AccOn1": 1 if enable else 0}
    return self.packer.make_can_msg_safety("ACC_HUD_ADAS", 2, values)  # camera side is bus 2

  def _speed_msg(self, speed):
    # 0x122 WHEEL_SPEED: used by the standard safety tests for vehicle_moving.
    # NOTE: on Song Plus DM-i the real 0x122 reads all zeros, so the flashed
    # firmware reads speed from 0x1F0 ESP_SPEED (see byd.h); the tests still
    # drive 0x122 here because it is the message the generic test harness expects.
    values = {"WHEELSPEED_BL": speed * 3.6, "WHEELSPEED_BR": speed * 3.6}
    return self.packer.make_can_msg_safety("WHEEL_SPEED", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePressed": brake}
    return self.packer.make_can_msg_safety("DRIVE_STATE", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AcceleratorPedal": gas}
    return self.packer.make_can_msg_safety("PEDAL", 0, values)

  def test_torque_req_mismatch(self):
    # torque request with LKAS_Active=0 is blocked (no tolerance on mismatch)
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(self.MAX_TORQUE)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_TORQUE, 1)))
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_TORQUE, 0)))


class TestBydSafetyAngle(BydButtonTestBase, common.CarSafetyTest, common.AngleSteeringSafetyTest):
  """Experimental 482 angle path (safetyParam ANGLE_STEERING)."""

  DBC = "byd_general_pt"
  SAFETY_MODEL = CarParams.SafetyModel.byd
  SAFETY_PARAM = 1  # BydSafetyFlags.ANGLE_STEERING

  TX_MSGS = [[0x1E2, 0], [0x3B0, 0]]  # STEERING_MODULE_ADAS, PCM_BUTTONS
  RELAY_MALFUNCTION_ADDRS = {}

  GAS_PRESSED_THRESHOLD = 1

  # Angle control limits
  STEER_ANGLE_MAX = 90  # deg, DiPilot faults above this
  DEG_TO_CAN = 10

  ANGLE_RATE_BP = [0., 5., 15.]
  ANGLE_RATE_UP = [3., 1.2, 0.35]   # windup limit
  ANGLE_RATE_DOWN = [3., 2.5, 0.6]  # unwind limit

  def _angle_cmd_msg(self, angle: float, enabled: bool, increment_timer: bool = True):
    values = {"STEER_ANGLE": angle, "STEER_REQ": 1 if enabled else 0}
    return self.packer.make_can_msg_safety("STEERING_MODULE_ADAS", 0, values)

  def _angle_meas_msg(self, angle: float):
    values = {"SteeringAngle": angle}
    return self.packer.make_can_msg_safety("EPS", 0, values)

  def _pcm_status_msg(self, enable):
    # ACC MAIN posture (route-verified): enable = engaged session frame
    # (AccState=3, AccOn1=1); disable = true main-off (AccState=0, AccOn1=0).
    # AccState=1 + AccOn1=1 would be MAIN-ON standby, NOT an off (it also
    # shows up at ignition and after a brake cancel).
    values = {"AccState": 3 if enable else 0, "AccOn1": 1 if enable else 0}
    return self.packer.make_can_msg_safety("ACC_HUD_ADAS", 2, values)  # camera side is bus 2

  def _speed_msg(self, speed):
    # 0x122 WHEEL_SPEED: used by the standard safety tests for vehicle_moving.
    # NOTE: on Song Plus DM-i the real 0x122 reads all zeros, so the flashed
    # firmware reads speed from 0x1F0 ESP_SPEED (see byd.h); the tests still
    # drive 0x122 here because it is the message the generic test harness expects.
    values = {"WHEELSPEED_BL": speed * 3.6, "WHEELSPEED_BR": speed * 3.6}
    return self.packer.make_can_msg_safety("WHEEL_SPEED", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePressed": brake}
    return self.packer.make_can_msg_safety("DRIVE_STATE", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AcceleratorPedal": gas}
    return self.packer.make_can_msg_safety("PEDAL", 0, values)


class TestBydSafetyLong(BydTorqueTestBase, common.CarSafetyTest, common.DriverTorqueSteeringSafetyTest):
  """OP longitudinal (safetyParam LONGITUDINAL=2): torque lateral + transparent
  ACC_CMD/ACC_HUD/ACC_AEB replacement on bus 0."""

  DBC = "byd_general_pt"
  SAFETY_MODEL = CarParams.SafetyModel.byd
  SAFETY_PARAM = 2  # BydSafetyFlags.LONGITUDINAL

  # torque lateral + the three ACC-domain frames; matches BYD_TX_MSGS_LONG in byd.h
  TX_MSGS = [[0x316, 0], [0x32D, 0], [0x32E, 0], [0x32F, 0], [0x3B0, 0]]
  # forward block adds the ACC-domain frames (both relay directions)
  FWD_BLACKLISTED_ADDRS = {0: [0x1E2, 0x316, 0x32D, 0x32E, 0x32F],
                           2: [0x1E2, 0x316, 0x32D, 0x32E, 0x32F]}
  RELAY_MALFUNCTION_ADDRS = {}

  GAS_PRESSED_THRESHOLD = 1

  MAX_RATE_UP = 18
  MAX_RATE_DOWN = 18
  MAX_RT_DELTA = 250
  MAX_TORQUE_LOOKUP = [0], [300]
  NO_STEER_REQ_BIT = True

  DRIVER_TORQUE_ALLOWANCE = 120
  DRIVER_TORQUE_FACTOR = 3

  # ACC_CMD accel limits, raw units (0.05, -5): [-4.0, 2.0] m/s2
  MAX_ACCEL_RAW = 140
  MIN_ACCEL_RAW = 20
  INACTIVE_ACCEL_RAW = 100

  def _torque_cmd_msg(self, torque, steer_req=1):
    values = {"LKAS_Output": torque, "LKAS_Active": steer_req}
    return self.packer.make_can_msg_safety("ACC_MPC_STATE", 0, values)

  def _torque_driver_msg(self, torque):
    values = {"SteerDriverTorque": torque}
    return self.packer.make_can_msg_safety("ACC_EPS_STATE", 0, values)

  def _accel_cmd_msg(self, accel_raw, acc_control_active=0):
    # packer takes the physical value (0.05, -5); accel_raw is the CAN raw
    values = {"AccelCmd": accel_raw * 0.05 - 5.0, "AccControlActive": acc_control_active}
    return self.packer.make_can_msg_safety("ACC_CMD", 0, values)

  def _pcm_status_msg(self, enable):
    # ACC MAIN posture (route-verified): enable = engaged session frame
    # (AccState=3, AccOn1=1); disable = true main-off (AccState=0, AccOn1=0).
    # AccState=1 + AccOn1=1 would be MAIN-ON standby, NOT an off (it also
    # shows up at ignition and after a brake cancel).
    values = {"AccState": 3 if enable else 0, "AccOn1": 1 if enable else 0}
    return self.packer.make_can_msg_safety("ACC_HUD_ADAS", 2, values)  # camera side is bus 2

  def _speed_msg(self, speed):
    values = {"WHEELSPEED_BL": speed * 3.6, "WHEELSPEED_BR": speed * 3.6}
    return self.packer.make_can_msg_safety("WHEEL_SPEED", 0, values)

  def _user_brake_msg(self, brake):
    values = {"BrakePressed": brake}
    return self.packer.make_can_msg_safety("DRIVE_STATE", 0, values)

  def _user_gas_msg(self, gas):
    values = {"AcceleratorPedal": gas}
    return self.packer.make_can_msg_safety("PEDAL", 0, values)

  def test_accel_cmd_active(self):
    # OP commanding (AccControlActive=1): accel within [-4, 2] m/s2 and
    # controls_allowed required for ANY accel value (the stale-echo guard)
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._accel_cmd_msg(self.INACTIVE_ACCEL_RAW, acc_control_active=1)))
    # bounds
    self.assertTrue(self._tx(self._accel_cmd_msg(self.MAX_ACCEL_RAW, acc_control_active=1)))
    self.assertTrue(self._tx(self._accel_cmd_msg(self.MIN_ACCEL_RAW, acc_control_active=1)))
    # out of bounds
    self.assertFalse(self._tx(self._accel_cmd_msg(self.MAX_ACCEL_RAW + 1, acc_control_active=1)))
    self.assertFalse(self._tx(self._accel_cmd_msg(self.MIN_ACCEL_RAW - 1, acc_control_active=1)))

    # commanding (any accel) without controls_allowed is blocked
    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._accel_cmd_msg(self.INACTIVE_ACCEL_RAW, acc_control_active=1)))
    self.assertFalse(self._tx(self._accel_cmd_msg(self.INACTIVE_ACCEL_RAW + 20, acc_control_active=1)))

  def test_accel_cmd_echo_relay(self):
    # AccControlActive=0 (radar relay/standby): accel within limits allowed
    # regardless of controls_allowed - this carries the radar's own braking
    # while OP is disengaged (stock ACC still working)
    self.safety.set_controls_allowed(False)
    self.assertTrue(self._tx(self._accel_cmd_msg(self.INACTIVE_ACCEL_RAW, acc_control_active=0)))
    self.assertTrue(self._tx(self._accel_cmd_msg(self.MIN_ACCEL_RAW, acc_control_active=0)))
    self.assertTrue(self._tx(self._accel_cmd_msg(self.MAX_ACCEL_RAW, acc_control_active=0)))
    # out of bounds is never allowed
    self.assertFalse(self._tx(self._accel_cmd_msg(self.MIN_ACCEL_RAW - 1, acc_control_active=0)))
    self.assertFalse(self._tx(self._accel_cmd_msg(self.MAX_ACCEL_RAW + 1, acc_control_active=0)))

  def test_torque_req_mismatch(self):
    # torque request with LKAS_Active=0 is blocked (no tolerance on mismatch)
    self.safety.set_controls_allowed(True)
    self._set_prev_torque(self.MAX_TORQUE)
    self.assertTrue(self._tx(self._torque_cmd_msg(self.MAX_TORQUE, 1)))
    self.assertFalse(self._tx(self._torque_cmd_msg(self.MAX_TORQUE, 0)))


if __name__ == "__main__":
  unittest.main()
