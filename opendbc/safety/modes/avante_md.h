#pragma once

#include "opendbc/safety/declarations.h"

#define AVANTE_MD_VSM1 0x164U
#define AVANTE_MD_VSM2 0x165U
#define AVANTE_MD_TCS5 0x1F1U
#define AVANTE_MD_EMS6 0x260U
#define AVANTE_MD_TCU2 0x440U
#define AVANTE_MD_CLU1 0x4F0U

#define AVANTE_MD_VEHICLE_BUS 0U
#define AVANTE_MD_EPS_BUS 2U

static bool vehicle_vsm1_non_normal = false;

static uint8_t avante_md_vsm_checksum(const CANPacket_t *msg) {
  uint8_t checksum = 0U;
  for (uint8_t i = 0U; i < 7U; i++) {
    checksum ^= msg->data[i];
  }
  return checksum;
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
        controls_allowed = false;
        desired_torque_last = 0;
        rt_torque_last = 0;
      }
      vehicle_vsm1_non_normal = non_normal_new;
    }

    if (msg->addr == AVANTE_MD_VSM2) {
      uint16_t torque_driver_raw = (((uint16_t)msg->data[1] & 0xFU) << 8U) | (uint16_t)msg->data[0];
      int torque_driver_new = (int)torque_driver_raw - 2048;
      update_sample(&torque_driver, torque_driver_new);
    }

    if (msg->addr == AVANTE_MD_EMS6) {
      gas_pressed = ((msg->data[7] >> 6) & 0x3U) != 0U;
    }

    if (msg->addr == AVANTE_MD_TCU2) {
      brake_pressed = ((msg->data[3] >> 2) & 0x3U) != 0U;
    }

    if (msg->addr == AVANTE_MD_CLU1) {
      pcm_cruise_check(GET_BIT(msg, 24U));
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

    if (vehicle_vsm1_non_normal) {
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
      block = !vehicle_vsm1_non_normal;
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
    {.msg = {{AVANTE_MD_VSM2, AVANTE_MD_VEHICLE_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{AVANTE_MD_TCS5, AVANTE_MD_VEHICLE_BUS, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{AVANTE_MD_EMS6, AVANTE_MD_VEHICLE_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{AVANTE_MD_TCU2, AVANTE_MD_VEHICLE_BUS, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{AVANTE_MD_CLU1, AVANTE_MD_VEHICLE_BUS, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  vehicle_vsm1_non_normal = false;

  SAFETY_UNUSED(param);
  return BUILD_SAFETY_CFG(avante_md_rx_checks, AVANTE_MD_TX_MSGS);
}

const safety_hooks avante_md_hooks = {
  .init = avante_md_init,
  .rx = avante_md_rx_hook,
  .tx = avante_md_tx_hook,
  .fwd = avante_md_fwd_hook,
};
