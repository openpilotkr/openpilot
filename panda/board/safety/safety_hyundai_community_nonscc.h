#include "safety_hyundai_common.h"

const int HYUNDAI_COMMUNITY_MAX_ACCEL = 150;        // 1.5 m/s2
const int HYUNDAI_COMMUNITY_MIN_ACCEL = -300;       // -3.0 m/s2

const int HYUNDAI_COMMUNITY_ISO_MAX_ACCEL = 200;        // 2.0 m/s2
const int HYUNDAI_COMMUNITY_ISO_MIN_ACCEL = -400;       // -3.5 m/s2

bool aeb_cmd_act = false;
int prev_desired_accel = 0;
int decel_not_ramping = 0;

const CanMsg HYUNDAI_COMMUNITY_NONSCC_TX_MSGS[] = {
  {0x340, 0, 8}, {0x340, 1, 8}, // LKAS11 Bus 0, 1
  {0x4F1, 0, 4}, {0x4F1, 1, 4}, {0x4F1, 2, 4}, // CLU11 Bus 0, 1, 2
  {0x485, 0, 4}, // LFAHDA_MFC Bus 0
  {0x251, 2, 8},  // MDPS12, Bus 2
  {0x420, 0, 8}, //   SCC11,  Bus 0
  {0x421, 0, 8}, //   SCC12,  Bus 0
  {0x50A, 0, 8}, //   SCC13,  Bus 0
  {0x389, 0, 8},  //   SCC14,  Bus 0
  {0x4A2, 0, 8},  //   4a2SCC, Bus 0
  {0x316, 1, 8}, // EMS11, Bus 1
  {0x483, 0, 8}, //   FCA12,  Bus 0
  {0x38D, 0, 8},  //   FCA11,  Bus 0
  {0x7D0, 0, 8},  // SCC_DIAG, Bus 0
};

// for non SCC hyundai vehicles
AddrCheckStruct hyundai_community_nonscc_addr_checks[] = {
  {.msg = {{0x386, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
  {.msg = {{0x394, 0, 8, .expected_timestep = 10000U}, { 0 }, { 0 }}},
};

#define HYUNDAI_COMMUNITY_NONSCC_ADDR_CHECK_LEN (sizeof(hyundai_community_nonscc_addr_checks) / sizeof(hyundai_community_nonscc_addr_checks[0]))
addr_checks hyundai_community_nonscc_rx_checks = {hyundai_community_nonscc_addr_checks, HYUNDAI_COMMUNITY_NONSCC_ADDR_CHECK_LEN};

static int hyundai_community_nonscc_rx_hook(CANPacket_t *to_push) {

  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);

  bool valid = addr_safety_check(to_push, &hyundai_community_nonscc_rx_checks,
                            hyundai_get_checksum, hyundai_compute_checksum,
                            hyundai_get_counter, NULL);

  if (!valid){
    puth(addr);
  }

  if (valid && (bus == 0)) {
    if (addr == 0x251) {
      int torque_driver_new = ((GET_BYTES(to_push, 0, 4) & 0x7ffU) * 0.79) - 808; // scale down new driver torque signal to match previous one
      // update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // engage for non ACC car
    if (addr == 0x4F1) {
      int cruise_button = GET_BYTE(to_push, 0) & 0x7U;
      // enable on res+ or set- buttons press
      if (!controls_allowed && (cruise_button == 1 || cruise_button == 2)) {
        hyundai_common_cruise_state_check_alt(1);
      }
      // disable on cancel press
      if (cruise_button == 4) {
        controls_allowed = 0;
      }
    }

    // sample wheel speed, averaging opposite corners
    if (addr == 0x386) {
      uint32_t front_left_speed = GET_BYTES(to_push, 0, 2) & 0x3FFFU;
      uint32_t rear_right_speed = GET_BYTES(to_push, 6, 2) & 0x3FFFU;
      vehicle_moving = (front_left_speed > HYUNDAI_STANDSTILL_THRSLD) || (rear_right_speed > HYUNDAI_STANDSTILL_THRSLD);
    }

    gas_pressed = brake_pressed = false;

    generic_rx_checks((addr == 0x340));
  }

    // monitor AEB active command to bypass panda accel safety, don't block AEB
  if ((addr == 0x421) && (bus == 2)){
    aeb_cmd_act = (GET_BYTES(to_push, 0, 6) & 0x40U) != 0;
  }

  return valid;
}

static int hyundai_community_nonscc_tx_hook(CANPacket_t *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  tx = msg_allowed(to_send, HYUNDAI_COMMUNITY_NONSCC_TX_MSGS, sizeof(HYUNDAI_COMMUNITY_NONSCC_TX_MSGS)/sizeof(HYUNDAI_COMMUNITY_NONSCC_TX_MSGS[0]));

  // ACCEL: safety check

  if ((addr == 0X421) && (bus == 0) && (!aeb_cmd_act) && vehicle_moving) {
    int desired_accel = (GET_BYTE(to_send, 3) | ((GET_BYTE(to_send, 4) & 0x7) << 8)) - 1024;
    prev_desired_accel = desired_accel;
    if (!controls_allowed) {
        if (((desired_accel < -10) && (prev_desired_accel >= desired_accel))||  //staying in braking or braking more
            ((desired_accel > 10) && (prev_desired_accel <= desired_accel)))     //staying in gas or accelerating more
        {
           decel_not_ramping +=1;
        }
        else
        {
           decel_not_ramping =0;
        }
        if (decel_not_ramping > 5) {  // allow 5 loops
            tx = 0;
        }
    }
    if (controls_allowed) {
      bool vio = (alternative_experience & ALT_EXP_RAISE_LONGITUDINAL_LIMITS_TO_ISO_MAX)?
          max_limit_check(desired_accel, HYUNDAI_COMMUNITY_ISO_MAX_ACCEL, HYUNDAI_COMMUNITY_ISO_MIN_ACCEL) :
          max_limit_check(desired_accel, HYUNDAI_COMMUNITY_MAX_ACCEL, HYUNDAI_COMMUNITY_MIN_ACCEL);
      if (vio) {
        tx = 0;
      }
    }
  }

  // LKA STEER: safety check
  if (addr == 0x340) {
    OP_LKAS_live = 20;
    int desired_torque = ((GET_BYTES(to_send, 0, 4) >> 16) & 0x7ffU) - 1024U;
    uint32_t ts = microsecond_timer_get();
    bool violation = 0;

    if (controls_allowed) {

      // *** global torque limit check ***
      bool torque_check = 0;
      violation |= torque_check = max_limit_check(desired_torque, HYUNDAI_MAX_STEER, -HYUNDAI_MAX_STEER);

      // *** torque rate limit check ***
      bool torque_rate_check = 0;
      violation |= torque_rate_check = driver_limit_check(desired_torque, desired_torque_last, &torque_driver,
        HYUNDAI_MAX_STEER, HYUNDAI_MAX_RATE_UP, HYUNDAI_MAX_RATE_DOWN,
        HYUNDAI_DRIVER_TORQUE_ALLOWANCE, HYUNDAI_DRIVER_TORQUE_FACTOR);

      // used next time
      desired_torque_last = desired_torque;

      // *** torque real time rate limit check ***
      bool torque_rt_check = 0;
      violation |= torque_rt_check = rt_rate_limit_check(desired_torque, rt_torque_last, HYUNDAI_MAX_RT_DELTA);

      // every RT_INTERVAL set the new limits
      uint32_t ts_elapsed = get_ts_elapsed(ts, ts_last);
      if (ts_elapsed > HYUNDAI_RT_INTERVAL) {
        rt_torque_last = desired_torque;
        ts_last = ts;
      }
    }

    // no torque if controls is not allowed
    if (!controls_allowed && (desired_torque != 0)) {
      violation = 1;
    }

    // reset to 0 if either controls is not allowed or there's a violation
    if (violation || !controls_allowed) { // a reset worsen the issue of Panda blocking some valid LKAS messages
      desired_torque_last = 0;
      rt_torque_last = 0;
      ts_last = ts;
    }

    if (violation) {
      tx = 0;
    }
  }

  // UDS: Only tester present ("\x02\x3E\x80\x00\x00\x00\x00\x00") allowed on diagnostics address
  if (addr == 0x7D0) {
    if ((GET_BYTES(to_send, 0, 4) != 0x00803E02U) || (GET_BYTES(to_send, 4, 4) != 0x0U)) {
      tx = 0;
    }
  }

  // 1 allows the message through
  return tx;
}

static int hyundai_community_nonscc_fwd_hook(int bus_num, int addr) {

  int bus_fwd = -1;

  // forward cam to ccan and viceversa, except lkas cmd
  if (bus_num == 0) {
    bus_fwd = 2;
  }

  return bus_fwd;
}

static const addr_checks* hyundai_community_nonscc_init(uint16_t param) {
  hyundai_common_init(param);

  hyundai_community_nonscc_rx_checks = (addr_checks){hyundai_community_nonscc_addr_checks, HYUNDAI_COMMUNITY_NONSCC_ADDR_CHECK_LEN};
  return &hyundai_community_nonscc_rx_checks;
}

const safety_hooks hyundai_community_nonscc_hooks = {
  .init = hyundai_community_nonscc_init,
  .rx = hyundai_community_nonscc_rx_hook,
  .tx = hyundai_community_nonscc_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_community_nonscc_fwd_hook,
};
