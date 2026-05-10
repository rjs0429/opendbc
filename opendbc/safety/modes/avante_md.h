#pragma once

#include "opendbc/safety/declarations.h"

#define AVANTE_MD_VSM1 0x164U
#define AVANTE_MD_VSM2 0x165U
#define AVANTE_MD_TCS5 0x1F1U
#define AVANTE_MD_SAS1 0x2B0U
#define AVANTE_MD_EMS6 0x260U
#define AVANTE_MD_CLU2 0x690U

#define AVANTE_MD_VEHICLE_BUS 0U
#define AVANTE_MD_EPS_BUS 2U
#define AVANTE_MD_VSM1_STALE_US 200000U
#define AVANTE_MD_VALID_SAS_STAT 7U
#define AVANTE_MD_MAX_STEERING_TORQUE 1000
#define AVANTE_MD_MAX_STEERING_EPS_TORQUE 1000

static bool vehicle_vsm1_non_normal = false;
static bool vehicle_vsm1_seen = false;
static uint32_t vehicle_vsm1_last_time = 0U;
static bool eps_vsm2_non_normal = false;
static bool eps_vsm2_seen = false;
static uint32_t eps_vsm2_last_time = 0U;
static bool vehicle_sas1_invalid = false;
static bool vehicle_sas1_seen = false;
static uint32_t vehicle_sas1_last_time = 0U;
static bool eco_button_prev = false;
static bool eco_first_press_pending = false;
static uint32_t eco_first_press_time = 0U;

static void avante_md_disengage_controls(void) {
  controls_allowed = false;
  desired_torque_last = 0;
  rt_torque_last = 0;
  eco_first_press_pending = false;
}

static uint8_t avante_md_vsm_checksum(const CANPacket_t *msg) {
  uint8_t checksum = 0U;
  for (uint8_t i = 0U; i < 7U; i++) {
    checksum ^= msg->data[i];
  }
  return checksum;
}

static bool avante_md_vehicle_vsm1_stale(uint32_t now) {
  return vehicle_vsm1_seen &&
         (safety_get_ts_elapsed(now, vehicle_vsm1_last_time) > AVANTE_MD_VSM1_STALE_US);
}

static bool avante_md_eps_vsm2_stale(uint32_t now) {
  return eps_vsm2_seen &&
         (safety_get_ts_elapsed(now, eps_vsm2_last_time) > AVANTE_MD_VSM1_STALE_US);
}

static bool avante_md_vehicle_sas1_stale(uint32_t now) {
  return vehicle_sas1_seen &&
         (safety_get_ts_elapsed(now, vehicle_sas1_last_time) > AVANTE_MD_VSM1_STALE_US);
}

static bool avante_md_update_control_prereq_stale(void) {
  uint32_t now = microsecond_timer_get();
  bool stale = avante_md_vehicle_vsm1_stale(now) ||
               avante_md_eps_vsm2_stale(now) ||
               avante_md_vehicle_sas1_stale(now);
  if (stale) {
    avante_md_disengage_controls();
  }
  return stale;
}

static bool avante_md_control_prereqs_normal(void) {
  bool stale = avante_md_update_control_prereq_stale();
  return vehicle_vsm1_seen && !vehicle_vsm1_non_normal &&
         eps_vsm2_seen && !eps_vsm2_non_normal &&
         vehicle_sas1_seen && !vehicle_sas1_invalid &&
         !stale;
}

static void avante_md_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == (unsigned char)AVANTE_MD_VEHICLE_BUS) {
    if (msg->addr == AVANTE_MD_TCS5) {
      uint16_t wheel_fl = (uint16_t)msg->data[2] | (((uint16_t)msg->data[3] & 0xFU) << 8U);
      uint16_t wheel_fr = ((uint16_t)msg->data[3] >> 4U) | ((uint16_t)msg->data[4] << 4U);
      uint16_t wheel_rl = (uint16_t)msg->data[5] | (((uint16_t)msg->data[6] & 0xFU) << 8U);
      uint16_t wheel_rr = ((uint16_t)msg->data[6] >> 4U) | ((uint16_t)msg->data[7] << 4U);
      uint32_t wheel_sum = (uint32_t)wheel_fl + (uint32_t)wheel_fr + (uint32_t)wheel_rl + (uint32_t)wheel_rr;
      uint32_t wheel_avg = wheel_sum / 4U;
      float speed_ms = (float)wheel_avg;
      speed_ms *= 0.125F;
      speed_ms /= 3.6F;

      UPDATE_VEHICLE_SPEED(speed_ms);
      vehicle_moving = speed_ms > 0.1F;
    }

    if (msg->addr == AVANTE_MD_VSM1) {
      uint32_t now = microsecond_timer_get();
      if (avante_md_vehicle_vsm1_stale(now)) {
        avante_md_disengage_controls();
      }

      vehicle_vsm1_seen = true;
      vehicle_vsm1_last_time = now;
      bool checksum_ok = msg->data[7] == avante_md_vsm_checksum(msg);
      bool non_normal_new = !checksum_ok ||
                            (msg->data[0] != 0x00U) ||
                            (msg->data[1] != 0x08U) ||
                            (msg->data[2] != 0x00U) ||
                            (msg->data[3] != 0x00U) ||
                            (msg->data[4] != 0x00U) ||
                            (msg->data[5] != 0x00U) ||
                            ((msg->data[6] & 0xF0U) != 0x00U);
      if (non_normal_new && !vehicle_vsm1_non_normal) {
        avante_md_disengage_controls();
      }
      vehicle_vsm1_non_normal = non_normal_new;
    }

    if (msg->addr == AVANTE_MD_EMS6) {
      gas_pressed = ((msg->data[7] >> 6) & 0x3U) != 0U;
    }

    if (msg->addr == AVANTE_MD_CLU2) {
      uint32_t now = microsecond_timer_get();
      bool control_ready = avante_md_control_prereqs_normal();

      if (!control_ready || (eco_first_press_pending && (safety_get_ts_elapsed(now, eco_first_press_time) > 1000000U))) {
        eco_first_press_pending = false;
      }

      bool eco_pressed = (msg->data[6] & 0x1U) != 0U;
      if (eco_pressed && !eco_button_prev) {
        if (control_ready) {
          if (!eco_first_press_pending) {
            eco_first_press_pending = true;
            eco_first_press_time = now;
          } else {
            controls_allowed = !controls_allowed;
            if (!controls_allowed) {
              avante_md_disengage_controls();
            }
            eco_first_press_pending = false;
          }
        }
      }
      eco_button_prev = eco_pressed;
    }
  }

  if (msg->bus == (unsigned char)AVANTE_MD_EPS_BUS) {
    if (msg->addr == AVANTE_MD_VSM2) {
      uint32_t now = microsecond_timer_get();
      if (avante_md_eps_vsm2_stale(now)) {
        avante_md_disengage_controls();
      }

      eps_vsm2_seen = true;
      eps_vsm2_last_time = now;
      uint16_t torque_driver_raw = (((uint16_t)msg->data[1] & 0xFU) << 8U) | (uint16_t)msg->data[0];
      int torque_driver_new = (int)torque_driver_raw - 2048;
      uint16_t torque_eps_raw = (((uint16_t)msg->data[2]) << 4U) | (((uint16_t)msg->data[1] >> 4U) & 0xFU);
      int torque_eps_new = (int)torque_eps_raw - 2048;
      bool vsm2_fault = ((msg->data[3] & 0x3U) != 0U);
      bool torque_valid = (torque_driver_new <= AVANTE_MD_MAX_STEERING_TORQUE) &&
                          (torque_driver_new >= -AVANTE_MD_MAX_STEERING_TORQUE) &&
                          (torque_eps_new <= AVANTE_MD_MAX_STEERING_EPS_TORQUE) &&
                          (torque_eps_new >= -AVANTE_MD_MAX_STEERING_EPS_TORQUE);
      bool non_normal_new = vsm2_fault || !torque_valid;
      if (non_normal_new && !eps_vsm2_non_normal) {
        avante_md_disengage_controls();
      }
      eps_vsm2_non_normal = non_normal_new;
      update_sample(&torque_driver, torque_driver_new);
    }
    
    if (msg->addr == AVANTE_MD_SAS1) {
      uint32_t now = microsecond_timer_get();
      if (avante_md_vehicle_sas1_stale(now)) {
        avante_md_disengage_controls();
      }

      vehicle_sas1_seen = true;
      vehicle_sas1_last_time = now;
      bool invalid_new = msg->data[3] != AVANTE_MD_VALID_SAS_STAT;
      if (invalid_new && !vehicle_sas1_invalid) {
        avante_md_disengage_controls();
      }
      vehicle_sas1_invalid = invalid_new;
    }
  }
}

static bool avante_md_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits AVANTE_MD_STEERING_LIMITS = {
    .max_torque = 800,
    .max_rate_up = 10,
    .max_rate_down = 25,
    .max_rt_delta = 300,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance = 150,
    .type = TorqueDriverLimited,
  };

  bool tx = true;

  if ((msg->addr == AVANTE_MD_VSM1) && (msg->bus == (unsigned char)AVANTE_MD_EPS_BUS)) {
    uint16_t desired_torque_raw = (((uint16_t)msg->data[1] & 0xFU) << 8U) | (uint16_t)msg->data[0];
    int desired_torque = (int)desired_torque_raw - 2048;
    bool checksum_valid = msg->data[7] == avante_md_vsm_checksum(msg);

    if (!avante_md_control_prereqs_normal()) {
      tx = false;
    } else {
      bool steer_req = (msg->data[1] & 0x10U) != 0U;
      int ctr_mode = (msg->data[1] >> 5) & 0x7U;
      bool def_flag = (msg->data[2] & 0x1U) != 0U;

      bool violation = !checksum_valid || def_flag;
      violation |= steer_req ? (ctr_mode != 2) : (ctr_mode != 0);
      violation |= steer_torque_cmd_checks(desired_torque, steer_req, AVANTE_MD_STEERING_LIMITS);

      if (violation) {
        tx = false;
      }
    }
  }

  return tx;
}

static bool avante_md_fwd_hook(int bus_num, int addr) {
  bool block = false;
  if (addr == (int)AVANTE_MD_VSM1) {
    if (bus_num == (int)AVANTE_MD_VEHICLE_BUS) {
      block = avante_md_control_prereqs_normal();
    } else {
      block = true;
    }
  }
  return block;
}

static safety_config avante_md_init(uint16_t param) {
  static const CanMsg AVANTE_MD_TX_MSGS[] = {
    {AVANTE_MD_VSM1, AVANTE_MD_EPS_BUS, 8, .check_relay = true, .disable_static_blocking = true},
  };

  static RxCheck avante_md_rx_checks[] = {
    {.msg = {{AVANTE_MD_VSM1, AVANTE_MD_VEHICLE_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{AVANTE_MD_VSM2, AVANTE_MD_EPS_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{AVANTE_MD_TCS5, AVANTE_MD_VEHICLE_BUS, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{AVANTE_MD_SAS1, AVANTE_MD_EPS_BUS, 5, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{AVANTE_MD_EMS6, AVANTE_MD_VEHICLE_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{AVANTE_MD_CLU2, AVANTE_MD_VEHICLE_BUS, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  vehicle_vsm1_non_normal = false;
  vehicle_vsm1_seen = false;
  vehicle_vsm1_last_time = 0U;
  eps_vsm2_non_normal = false;
  eps_vsm2_seen = false;
  eps_vsm2_last_time = 0U;
  vehicle_sas1_invalid = false;
  vehicle_sas1_seen = false;
  vehicle_sas1_last_time = 0U;
  eco_button_prev = false;
  eco_first_press_pending = false;
  eco_first_press_time = 0U;

  SAFETY_UNUSED(param);
  return BUILD_SAFETY_CFG(avante_md_rx_checks, AVANTE_MD_TX_MSGS);
}

const safety_hooks avante_md_hooks = {
  .init = avante_md_init,
  .rx = avante_md_rx_hook,
  .tx = avante_md_tx_hook,
  .fwd = avante_md_fwd_hook,
};
