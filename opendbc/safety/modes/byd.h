#pragma once

#include "opendbc/safety/declarations.h"

// BYD lateral control, two paths selected by safetyParam:
//
//  param 0 (default): TORQUE path. The EPS follows the LKAS request carried in
//    ACC_MPC_STATE (0x316): LKAS_Config=LKA, LKAS_Output=steer torque,
//    LKAS_Active=actuation. Stock ACC is used for longitudinal; PCM_BUTTONS
//    (0x3B0) UP_RESETSPEED is spoofed for auto-resume from standstill.
//    Message layouts are from byd_han_dmev_2020.dbc, extracted from a working
//    BYD device.
//
//  param 1: experimental ANGLE path (bukapilot Atto 3 style), spoofing
//    STEERING_MODULE_ADAS (0x1E2) with a desired angle. Song Plus DM-i does
//    not transmit 0x1E2, so this path is kept only as a fallback.
//
//  param 2 (LONGITUDINAL, combines with the default torque path): OP
//    longitudinal control via transparent replacement of the radar's ACC
//    frames on bus 0 - ACC_CMD (0x32E) is the radar frame with accel
//    overridden while engaged, ACC_HUD_ADAS (0x32D) / ACC_AEB (0x32F) are
//    relays. The stock radar stays the session owner (bus-2 frames keep
//    feeding cruiseState); this flag adds the 0x32D/E/F TX whitelist entries,
//    the 0x32E accel checks and the bus2->bus0 forward block for 0x32D/E/F.
//
// TODO(Song Plus DM-i): verify LKAS_Output scaling and driver torque
// thresholds from real vehicle CAN logs. 0x122 is confirmed to read all
// zeros on this car; the real speed comes from 0x1F0 ESP_SPEED. 0x122 is
// still parsed as a fallback so the standard safety test harness keeps
// working (it drives 0x122), but on the real car 0x1F0 overrides it.

#define BYD_PARAM_ANGLE_STEERING 1
#define BYD_PARAM_LONGITUDINAL 2

static bool byd_brake_pedal_pressed = false;

// ACC main fall hold (0x32D frames). The camera drops the main posture for
// ~2 s with no button event (route 11 t=156.4, route 12 t=200.3); hold
// through drops shorter than this so the edge machinery only sees genuine
// main-off. 60 frames at the measured 100 Hz 0x32D rate = 0.6 s -
// deliberately LONGER than OP's own 0.5 s available debounce (carstate
// AVAIL_FALL_DEBOUNCE_TIME) so OP always stops transmitting first; the
// opposite window (OP still sending, panda already refusing) would punch
// holes into the 0x316/0x32E streams.
#define BYD_ACC_MAIN_FALL_HOLD 60U
static unsigned int byd_acc_main_hold = 0U;  // 0x32D frames since main dropped

static void byd_rx_hook(const CANPacket_t *msg) {

  if (msg->bus == 0U) {
    // EPS: measured steering angle, factor 0.1 deg, little endian
    if (msg->addr == 0x11FU) {
      int angle_meas_new = to_signed((GET_BYTES_LE(msg, 0, 2) & 0xFFFFU), 16);
      update_sample(&angle_meas, angle_meas_new);
    }

    // WHEEL_SPEED: 0x122 reads all zeros on Song Plus DM-i (verified on real
    // vehicle logs). It is parsed here only so the standard safety test harness
    // (which drives 0x122) keeps working; on the real car the 0x1F0 branch
    // below overrides it every 50 ms.
    if (msg->addr == 0x122U) {
      uint16_t left_rear = ((msg->data[5] << 8) | msg->data[4]);
      uint16_t right_rear = ((msg->data[7] << 8) | msg->data[6]);
      vehicle_moving = (left_rear | right_rear) != 0U;
      UPDATE_VEHICLE_SPEED(((left_rear + right_rear) / 2.0) * 0.1 / 3.6);
    }

    // ESP_SPEED: real vehicle speed on Song Plus DM-i, byte 4, factor 1 km/h
    // (bus 0). Overrides the all-zero 0x122 reading on the real car.
    if (msg->addr == 0x1F0U) {
      uint16_t esp_speed_kph = msg->data[4];
      vehicle_moving = esp_speed_kph != 0U;
      UPDATE_VEHICLE_SPEED(esp_speed_kph / 3.6);
    }

    // PEDAL: analog gas and brake pedal, factor 0.01 (percent)
    if (msg->addr == 0x342U) {
      gas_pressed = msg->data[0] > 1U;
      byd_brake_pedal_pressed = msg->data[1] > 1U;
    }

    // DRIVE_STATE: brake switch
    if (msg->addr == 0x242U) {
      brake_pressed = GET_BIT(msg, 37U) || byd_brake_pedal_pressed;
    }

    // ACC_EPS_STATE: EPS feedback to the ADAS domain
    if (msg->addr == 0x318U) {
      int torque_driver_new = to_signed((((msg->data[4] & 0xFU) << 8) | msg->data[3]), 12);  // SteerDriverTorque 24|12
      update_sample(&torque_driver, torque_driver_new);

      int torque_meas_new = to_signed((((msg->data[2] & 0xFU) << 8) | msg->data[1]), 12);  // MainTorque 8|12
      update_sample(&torque_meas, torque_meas_new);
    }
  }

  // ACC_HUD_ADAS: stock ACC status
  // NOTE(Song Plus DM-i): the stock DiPilot camera sits on the intercepted
  // camera-side CAN (bus 2), so its messages are received on bus 2.
  if ((msg->addr == 0x32DU) && (msg->bus == 2U)) {
    // Two independent postures live in this frame (route 11/12 verified):
    //   main    - the stalk's ACC main switch: AccOn1 (data[2] bit 6) or
    //             AccState 1/2/3/5. Survives brake-cancel, CANCEL and the
    //             radar's bounce blips (AccOn1 stays 1 through all of them);
    //             drops only when ACC is turned off.
    //   session - the radar actually commanding: AccControlActive (0x32E)
    //             or AccState 2/3/5. Drops on brake (stock auto-resumes
    //             ~40 ms after release), CANCEL and the 60-110 ms flaps.
    // acc_main_on feeds pcm_cruise_check, and OP's whole lifecycle hangs on
    // it. It must be the MAIN posture, not the session: lateral permission
    // has to survive brake / CANCEL / bounce (the driver never asked anything
    // to stop) - the old {2,3,5} test made every brake press kill lateral
    // control with the session churn the car itself produces.
    uint8_t acc_state = ((msg->data[2] >> 3) & 0x7U);
    bool acc_on1 = GET_BIT(msg, 22U);
    bool raw_main = acc_on1 || (acc_state == 1U) || (acc_state == 2U) || (acc_state == 3U) || (acc_state == 5U);
    if (raw_main) {
      byd_acc_main_hold = 0U;
    } else {
      byd_acc_main_hold += 1U;
    }
    acc_main_on = raw_main || (byd_acc_main_hold < BYD_ACC_MAIN_FALL_HOLD);
    pcm_cruise_check(acc_main_on);
  }

  // Pedal state is OBSERVED here but BYD deliberately does not call
  // generic_rx_checks(): neither pedal may drop controls_allowed. OP's
  // cruiseState.enabled is the ACC main latch (carstate), which a pedal press
  // does not toggle - a panda-side disengage here would have no rising edge
  // to recover from until the driver cycles ACC, killing both axes. Driver
  // override lives upstream instead: selfdrived suppresses the brake
  // disengage for byd, DisengageOnAccelerator still applies to the
  // longitudinal state machine on the OP side, and the controller gates
  // ACC_CMD TX on the live radar session. We keep the *_prev mirrors
  // consistent so pandaState and the test harness see coherent pedal edges.
  gas_pressed_prev = gas_pressed;
  brake_pressed_prev = brake_pressed;
}

static bool byd_button_checks(const CANPacket_t *msg) {
  // PCM_BUTTONS: UP_RESETSPEED spoof (SNG auto-resume) is only allowed while
  // stationary; cancel/toggle/distance buttons are never allowed
  bool violation = false;
  if (msg->addr == 0x3B0U) {
    // BTN_AccUpDown_Cmd is motorola 4|2: bits 4 (msb) and 3 (lsb)
    uint8_t updown_cmd = ((msg->data[0] >> 3) & 0x3U);
    bool other_btns = GET_BIT(msg, 6U) || GET_BIT(msg, 8U) || GET_BIT(msg, 15U) || GET_BIT(msg, 16U);

    if (other_btns || vehicle_moving || (updown_cmd != 0U && updown_cmd != 3U)) {
      violation = true;
    }
  }
  return violation;
}

static bool byd_tx_hook(const CANPacket_t *msg) {
  bool tx = true;
  bool violation = false;

  if (GET_FLAG(current_safety_param, BYD_PARAM_ANGLE_STEERING)) {
    const AngleSteeringLimits BYD_STEERING_LIMITS = {
      .max_angle = 900,               // 90 deg, DiPilot faults above this
      .angle_deg_to_can = 10,
      .angle_rate_up_lookup = {
        {0., 5., 15.},
        {3., 1.2, 0.35}
      },
      .angle_rate_down_lookup = {
        {0., 5., 15.},
        {3., 2.5, 0.6}
      },
    };

    // STEERING_MODULE_ADAS: steering angle command (experimental angle path)
    if (msg->addr == 0x1E2U) {
      // STEER_ANGLE: factor -0.1, little endian, start bit 24
      int desired_angle = to_signed(((msg->data[4] << 8) | msg->data[3]), 16);
      bool steer_req = GET_BIT(msg, 21U);

      // no steering control allowed when openpilot is not engaged
      if (steer_req && !controls_allowed) {
        violation = true;
      }

      if (steer_angle_cmd_checks(desired_angle, steer_req, BYD_STEERING_LIMITS)) {
        violation = true;
      }
    }
  } else {
    const TorqueSteeringLimits BYD_TORQUE_STEERING_LIMITS = {
      .max_torque = 300,              // matches python STEER_MAX
      .max_rate_up = 18,              // per frame at 50 Hz: vendor firmware struct dump (ELF) = 18 (python STEER_DELTA_UP=16)
      .max_rate_down = 18,            // vendor struct = 18 both directions
      .max_rt_delta = 250,            // 250 ms realtime limit: binds at 12.5 frames x 16 ~ 200 achievable, never clips legitimate ramps
      .type = TorqueDriverLimited,

      // matches python STEER_DRIVER_ALLOWANCE / STEER_DRIVER_MULTIPLIER
      // (vendor firmware numbers, STEER_MAX=300/ALLOWANCE=120: its real
      // traffic reaches -153 against a +166 driver yank and 193 absolute -
      // under 68 the driver-limit clip zeroes opposing requests past
      // |drv| ~135, which is exactly the armed-silence the EPS latches on)
      .driver_torque_allowance = 120,
      .driver_torque_multiplier = 3,

      .max_torque_error = 350,
    };

    // ACC_MPC_STATE: LKAS torque command (default torque path)
    if (msg->addr == 0x316U) {
      // LKAS_Output 16|11, signed
      int desired_torque = to_signed(((msg->data[2] | ((msg->data[3] & 0x7U) << 8)) & 0x7FFU), 11);
      bool lka_active = GET_BIT(msg, 28U);  // LKAS_Active

      // BYD gate: only the idle frame (LKAS_Active=0, torque=0) may stream
      // while disengaged - the controller transmits 0x316 at 50 Hz
      // unconditionally and this bit is the actuation switch. NOTE(port):
      // this fork's steer_torque_cmd_checks gates its rate/driver limit checks
      // on (controls_allowed || controls_allowed_lateral); with MADS
      // short-circuited for BYD (aol_allowed bypass, Plan A) the lateral
      // permission never comes from MADS, so BYD enforces the engaged gate
      // itself here instead of relying on it.
      if (!controls_allowed && (lka_active || (desired_torque != 0))) {
        violation = true;
      }

      if (steer_torque_cmd_checks(desired_torque, lka_active, BYD_TORQUE_STEERING_LIMITS)) {
        violation = true;
      }
    }
  }

  // OP longitudinal (transparent ACC replacement, param flag LONGITUDINAL):
  // ACC_CMD (0x32E) re-broadcasts the stock radar's frame with the accel
  // fields overridden while engaged. The echo-relay semantics differ from the
  // hyundai-style gate: with OP disengaged the frame is the RADAR's own
  // command (its bus-2 frames stay alive and OP relays them), so a nonzero
  // accel with AccControlActive=1 is legitimate stock-ACC braking, not an OP
  // injection. Rules:
  //   - AccControlActive=1: requires controls_allowed (OP is commanding) and
  //     accel within raw limits. Covers the stale-echo hazard too: a phantom
  //     engaged radar cannot inject accel while OP is off.
  //   - AccControlActive=0: relay/standby frame - accel within raw limits.
  //   - raw 100 (0.0 m/s2, the inactive value) is always allowed.
  if (GET_FLAG(current_safety_param, BYD_PARAM_LONGITUDINAL) && (msg->addr == 0x32EU)) {
    // AccelCmd 0|8 (0.05, -5): raw [20, 140] = [-4.0, 2.0] m/s2, raw 100 = 0.0
    int desired_accel_raw = (int)msg->data[0] - 100;
    bool acc_control_active = GET_BIT(msg, 44U);

    bool violation = false;
    if (acc_control_active) {
      violation |= !controls_allowed;
    }
    // same semantics as helpers.h's safety_max_limit_check(val, 40, -80)
    violation |= (desired_accel_raw > 40) || (desired_accel_raw < -80);

    if (violation) {
      tx = false;
    }
  }

  if (byd_button_checks(msg)) {
    violation = true;
  }

  if (violation) {
    tx = false;
  }

  return tx;
}

static bool byd_fwd_hook(int bus_num, int addr) {
  // Full relay bus 0 <-> bus 2: the stock camera AND front radar are fed
  // through it (a narrow white-list was tried on the car and starved the
  // radar - 'check front millimeter-wave radar'). Block only the LKAS/angle
  // control frames: the camera's own 0x316 must NOT reach the EPS (OP
  // replaces it - the controller transmits 0x316 at 50 Hz unconditionally,
  // idle echo frames while disengaged, or the EPS's LKAS subsystem starves
  // and faults the ADAS domain). Matches the community-verified BYD_Files
  // firmware policy; 0x32E (ACC_CMD) must keep flowing for stock ACC.
  //
  // NOTE(port): this fork's fwd hook contract is "return true to block";
  // the 0 <-> 2 destination mapping is the default get_fwd_bus() behavior in
  // safety.h, so only the block set is expressed here. Blocking is
  // intentionally DIRECTION-AGNOSTIC (both relay directions): it also stops
  // OP's own bus-0 TX from looping back onto bus 2, where it would collide
  // with the stock sender of the same address.
  (void)bus_num;
  bool block_msg = (addr == 0x1E2U) || (addr == 0x316U);

  // OP longitudinal: block the ACC domain's own frames from relaying onto
  // bus 0 - OP re-broadcasts 0x32D/0x32E/0x32F itself (transparent
  // replacement, vendor route 37: the stock frames must not collide with
  // OP's on bus 0). Bus-2 RX is unaffected, so cruiseState/pcm_cruise_check
  // keep seeing the radar's frames. Toggling the param (no LONG flag) puts
  // the relay back to stock-ACC pass-through with no reflash.
  if (GET_FLAG(current_safety_param, BYD_PARAM_LONGITUDINAL)) {
    block_msg |= (addr == 0x32DU) || (addr == 0x32EU) || (addr == 0x32FU);
  }

  return block_msg;
}

static safety_config byd_init(uint16_t param) {
  // TX whitelist matches the flashed firmware: OP's own messages only.
  // Card-level CAN forwarding is disabled; the firmware relay
  // (get_fwd_bus: bus 0 <-> bus 2) carries bus 0 <-> bus 2 traffic.
  static const CanMsg BYD_TX_MSGS_TORQUE[] = {
    {0x316, 0, 8, .check_relay = false},  // ACC_MPC_STATE (LKAS torque request)
    {0x3B0, 0, 8, .check_relay = false},  // PCM_BUTTONS (SNG auto-resume)
  };

  static const CanMsg BYD_TX_MSGS_ANGLE[] = {
    {0x1E2, 0, 8, .check_relay = false},  // STEERING_MODULE_ADAS (experimental angle path)
    {0x3B0, 0, 8, .check_relay = false},  // PCM_BUTTONS (SNG auto-resume)
  };

  // OP longitudinal (torque lateral + transparent ACC replacement)
  static const CanMsg BYD_TX_MSGS_LONG[] = {
    {0x316, 0, 8, .check_relay = false},  // ACC_MPC_STATE (LKAS torque request)
    {0x32D, 0, 8, .check_relay = false},  // ACC_HUD_ADAS relay (camera HUD echo)
    {0x32E, 0, 8, .check_relay = false},  // ACC_CMD (radar frame echo + OP accel override)
    {0x32F, 0, 8, .check_relay = false},  // ACC_AEB heartbeat relay
    {0x3B0, 0, 8, .check_relay = false},  // PCM_BUTTONS (SNG auto-resume)
  };

  // NOTE(port): this fork runs the quality-flag check on every RxCheck
  // (valid_quality_flag defaults to ignore_quality_flag when the mode has no
  // get_quality_flag_valid hook), so every entry below sets it to true to
  // reproduce carrot's no-quality-check behavior.
  static RxCheck byd_rx_checks_torque[] = {
    {.msg = {{0x11F, 0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // EPS
    {.msg = {{0x122, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // WHEEL_SPEED (all zeros on Song Plus, liveness only)
    {.msg = {{0x1F0, 0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ESP_SPEED (real vehicle speed)
    {.msg = {{0x242, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // DRIVE_STATE
    {.msg = {{0x342, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // PEDAL
    {.msg = {{0x32D, 2, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ACC_HUD_ADAS (camera side, bus 2)
    {.msg = {{0x318, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ACC_EPS_STATE
  };

  static RxCheck byd_rx_checks_angle[] = {
    {.msg = {{0x11F, 0, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // EPS
    {.msg = {{0x122, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // WHEEL_SPEED (all zeros on Song Plus, liveness only)
    {.msg = {{0x1F0, 0, 8, 20U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ESP_SPEED (real vehicle speed)
    {.msg = {{0x242, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // DRIVE_STATE
    {.msg = {{0x342, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // PEDAL
    {.msg = {{0x32D, 2, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},   // ACC_HUD_ADAS (camera side, bus 2)
  };

  safety_config ret;
  if (GET_FLAG(param, BYD_PARAM_ANGLE_STEERING)) {
    SET_TX_MSGS(BYD_TX_MSGS_ANGLE, ret);
    SET_RX_CHECKS(byd_rx_checks_angle, ret);
  } else if (GET_FLAG(param, BYD_PARAM_LONGITUDINAL)) {
    SET_TX_MSGS(BYD_TX_MSGS_LONG, ret);
    SET_RX_CHECKS(byd_rx_checks_torque, ret);
  } else {
    SET_TX_MSGS(BYD_TX_MSGS_TORQUE, ret);
    SET_RX_CHECKS(byd_rx_checks_torque, ret);
  }
  // BYD's cruiseState.enabled is the ACC main latch (carstate): brake and
  // session drops deliberately do NOT disengage - see the note in byd_rx_hook.
  ret.disable_pedal_disengage = true;
  return ret;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
};
