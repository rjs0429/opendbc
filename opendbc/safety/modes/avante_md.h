#pragma once

#include "opendbc/safety/declarations.h"

// ── Message IDs ──────────────────────────────────────────────────────────────
#define AVANTE_MD_TCS1    0x153U
#define AVANTE_MD_VSM1    0x164U
#define AVANTE_MD_VSM2    0x165U
#define AVANTE_MD_TCS5    0x1F1U
#define AVANTE_MD_ESP2    0x220U
#define AVANTE_MD_SAS1    0x2B0U
#define AVANTE_MD_TCU1    0x43FU
#define AVANTE_MD_TCU2    0x440U
#define AVANTE_MD_WHL_PUL 0x4B1U
#define AVANTE_MD_CLU1    0x4F0U
#define AVANTE_MD_MDPS1   0x5E4U
#define AVANTE_MD_CLU2    0x690U

// ── Bus IDs ──────────────────────────────────────────────────────────────────
#define AVANTE_MD_VEHICLE_BUS 0U
#define AVANTE_MD_EPS_BUS     2U

// ── Timing thresholds (microseconds) ─────────────────────────────────────────
#define AVANTE_MD_STALE_FAST_US     100000U
#define AVANTE_MD_STALE_10HZ_US     250000U
#define AVANTE_MD_VSM1_TX_MIN_US    7000U
#define AVANTE_MD_OP_VSM1_RECENT_US 30000U
#define AVANTE_MD_STABILIZE_US      100000U

// ── Signal value constants ────────────────────────────────────────────────────
#define AVANTE_MD_VALID_SAS_STAT 7U
#define AVANTE_MD_TCU1_DRIVE     5U
#define AVANTE_MD_TCU2_GEAR_MIN  1U
#define AVANTE_MD_TCU2_GEAR_MAX  6U
#define AVANTE_MD_TCU2_GEAR_R    14U

// ── Steering torque safety limits (0.01 Nm units) ────────────────────────────
// max 8.0 Nm, rate_up 0.1 Nm, rate_down 0.25 Nm, rt_delta 3.0 Nm, allowance 1.5 Nm
#define AVANTE_MD_MAX_STEER_TORQUE  800
#define AVANTE_MD_MAX_RATE_UP        10
#define AVANTE_MD_MAX_RATE_DOWN      25
#define AVANTE_MD_MAX_RT_DELTA      300
#define AVANTE_MD_DRIVER_ALLOWANCE  150

// ── RX message state tracking ─────────────────────────────────────────────────
typedef struct {
  bool     seen;
  uint32_t last_time;
} AvanteMdRxState;

static AvanteMdRxState avante_md_tcs1_state    = {false, 0U};
static AvanteMdRxState avante_md_vsm1_state    = {false, 0U};
static AvanteMdRxState avante_md_tcs5_state    = {false, 0U};
static AvanteMdRxState avante_md_esp2_state    = {false, 0U};
static AvanteMdRxState avante_md_whl_pul_state = {false, 0U};
static AvanteMdRxState avante_md_clu1_state    = {false, 0U};
static AvanteMdRxState avante_md_clu2_state    = {false, 0U};
static AvanteMdRxState avante_md_tcu1_state    = {false, 0U};
static AvanteMdRxState avante_md_tcu2_state    = {false, 0U};
static AvanteMdRxState avante_md_vsm2_state    = {false, 0U};
static AvanteMdRxState avante_md_sas1_state    = {false, 0U};
static AvanteMdRxState avante_md_mdps1_state   = {false, 0U};

// ── Derived vehicle state flags ───────────────────────────────────────────────
static bool avante_md_vsm1_normal          = false;
static bool avante_md_vsm2_normal          = false;
static bool avante_md_sas1_valid           = false;
static bool avante_md_tcu1_drive           = false;
static bool avante_md_tcu2_drive           = false;
static bool avante_md_doors_closed         = false;
static bool avante_md_seatbelt_latched     = false;
static bool avante_md_parking_brake_off    = false;

// ── Openpilot VSM1 TX tracking ────────────────────────────────────────────────
static bool     avante_md_vsm1_tx_seen      = false;
static uint32_t avante_md_vsm1_tx_last_time = 0U;

// ── TX-block stabilization tracking ──────────────────────────────────────────
static bool     avante_md_block_seen      = false;
static uint32_t avante_md_block_last_time = 0U;

// ── RX state helpers ──────────────────────────────────────────────────────────

static void avante_md_reset_rx_state(AvanteMdRxState *state) {
  state->seen      = false;
  state->last_time = 0U;
}

static void avante_md_update_rx_state(AvanteMdRxState *state, uint32_t now) {
  state->seen      = true;
  state->last_time = now;
}

static bool avante_md_rx_state_stale(const AvanteMdRxState *state, uint32_t now, uint32_t stale_us) {
  return !state->seen ||
         (safety_get_ts_elapsed(now, state->last_time) > stale_us);
}

// Checks rx_msg_safety_check status for a given addr/bus pair.
static bool avante_md_rx_check_valid(int addr, unsigned int bus) {
  bool valid = false;
  for (int i = 0; i < current_safety_config.rx_checks_len; i++) {
    const RxCheck *check = &current_safety_config.rx_checks[i];
    if (check->status.msg_seen) {
      const CanMsgCheck *msg_check = &check->msg[check->status.index];
      if ((msg_check->addr == addr) && (msg_check->bus == bus)) {
        valid = check->status.valid_checksum &&
                check->status.valid_quality_flag &&
                (check->status.wrong_counters < MAX_WRONG_COUNTERS);
        break;
      }
    }
  }
  return valid;
}

// ── Checksum computation ──────────────────────────────────────────────────────

static uint8_t avante_md_get_counter(const CANPacket_t *msg) {
  uint8_t counter = 0U;
  if ((msg->addr == AVANTE_MD_VSM1) || (msg->addr == AVANTE_MD_VSM2)) {
    counter = msg->data[6] & 0xFU;
  } else if (msg->addr == AVANTE_MD_SAS1) {
    counter = msg->data[4] & 0xFU;
  } else if (msg->addr == AVANTE_MD_TCU2) {
    counter = (msg->data[1] >> 4U) & 0x3U;
  } else if (msg->addr == AVANTE_MD_CLU1) {
    counter = (msg->data[2] >> 1U) & 0x7FU;
  } else if (msg->addr == AVANTE_MD_CLU2) {
    counter = (msg->data[6] >> 2U) & 0xFU;
  } else {
    // no counter for this message
  }
  return counter;
}

static uint32_t avante_md_get_checksum(const CANPacket_t *msg) {
  uint32_t checksum = 0U;
  if ((msg->addr == AVANTE_MD_TCS1) || (msg->addr == AVANTE_MD_WHL_PUL) ||
      (msg->addr == AVANTE_MD_VSM1) || (msg->addr == AVANTE_MD_VSM2)) {
    checksum = (uint32_t)msg->data[7];
  } else if (msg->addr == AVANTE_MD_SAS1) {
    checksum = (uint32_t)(msg->data[4] >> 4U);
  } else if (msg->addr == AVANTE_MD_TCU2) {
    checksum = (uint32_t)((msg->data[1] >> 6U) & 0x3U);
  } else {
    // no checksum for this message
  }
  return checksum;
}

static uint32_t avante_md_compute_checksum(const CANPacket_t *msg) {
  uint32_t checksum = 0U;

  if ((msg->addr == AVANTE_MD_TCS1) || (msg->addr == AVANTE_MD_WHL_PUL)) {
    // Sum of bytes 0..6, truncated to 8 bits
    for (uint8_t i = 0U; i < 7U; i++) {
      checksum += (uint32_t)msg->data[i];
    }
    checksum &= 0xFFU;

  } else if ((msg->addr == AVANTE_MD_VSM1) || (msg->addr == AVANTE_MD_VSM2)) {
    // XOR of bytes 0..6
    for (uint8_t i = 0U; i < 7U; i++) {
      checksum ^= (uint32_t)msg->data[i];
    }

  } else if (msg->addr == AVANTE_MD_SAS1) {
    // All payload nibbles XOR; byte4 low nibble is MsgCount (included)
    uint8_t cs = 0U;
    for (uint8_t i = 0U; i < 4U; i++) {
      cs ^= (msg->data[i] >> 4U) ^ (msg->data[i] & 0xFU);
    }
    cs ^= (msg->data[4] & 0xFU);
    checksum = ((uint32_t)cs) & 0xFU;

  } else if (msg->addr == AVANTE_MD_TCU2) {
    // 2-bit checksum: (alive + gear_offset) & 0x3
    // Valid gears: 0, 1-6, 14 (R).  Non-valid gears return sentinel that never matches.
    uint8_t gear  = msg->data[1] & 0xFU;
    uint8_t alive = (msg->data[1] >> 4U) & 0x3U;
    uint8_t gear_offset = 0U;
    bool valid_gear = true;

    if (gear == 0U) {
      gear_offset = 0U;
    } else if ((gear >= AVANTE_MD_TCU2_GEAR_MIN) && (gear <= AVANTE_MD_TCU2_GEAR_MAX)) {
      gear_offset = ((gear - 1U) % 3U) + 1U;
    } else if (gear == AVANTE_MD_TCU2_GEAR_R) {
      gear_offset = 1U;
    } else {
      // Non-normal gear value: fail checksum to trigger safety block
      valid_gear = false;
    }

    if (valid_gear) {
      checksum = (((uint32_t)alive) + ((uint32_t)gear_offset)) & 0x3U;
    } else {
      checksum = 0xFFU;
    }

  } else {
    // No checksum defined for this message
  }

  return checksum;
}

// ── Vehicle signal predicates ──────────────────────────────────────────────────

// VSM1 RX normal state: idle EPS frame (data[0..5] = 0x00 0x08 0x00 0x00 0x00 0x00,
// data[6] upper nibble = 0x0, checksum matches).
static bool avante_md_vsm1_normal_state(const CANPacket_t *msg) {
  return (msg->data[0] == 0x00U) &&
         (msg->data[1] == 0x08U) &&
         (msg->data[2] == 0x00U) &&
         (msg->data[3] == 0x00U) &&
         (msg->data[4] == 0x00U) &&
         (msg->data[5] == 0x00U) &&
         ((msg->data[6] & 0xF0U) == 0x00U) &&
         (avante_md_get_checksum(msg) == avante_md_compute_checksum(msg));
}

// VSM2: fault-free when CF_Mdps_Def == 0 and CF_Mdps_SErr == 0 (byte3 bits 0-1).
static bool avante_md_vsm2_fault_free(const CANPacket_t *msg) {
  return (msg->data[3] & 0x3U) == 0U;
}

// SAS1: valid when SAS_Stat == AVANTE_MD_VALID_SAS_STAT (byte3).
static bool avante_md_sas1_status_valid(const CANPacket_t *msg) {
  return msg->data[3] == AVANTE_MD_VALID_SAS_STAT;
}

// TCU1: D range when CUR_GR == 5 (byte1 bits 0-3).
static bool avante_md_tcu1_in_drive(const CANPacket_t *msg) {
  return (msg->data[1] & 0xFU) == AVANTE_MD_TCU1_DRIVE;
}

// TCU2: D range when CUR_GR in 1..6 (byte1 bits 0-3).
static bool avante_md_tcu2_in_drive(const CANPacket_t *msg) {
  uint8_t gear = msg->data[1] & 0xFU;
  return (gear >= AVANTE_MD_TCU2_GEAR_MIN) && (gear <= AVANTE_MD_TCU2_GEAR_MAX);
}

// CLU1: parking brake released when CF_Clu_ParkBrakeSw == 0 (byte0 bit7).
static bool avante_md_clu1_parking_brake_off(const CANPacket_t *msg) {
  return (msg->data[0] & 0x80U) == 0U;
}

// CLU2: both doors closed when CF_Clu_DrvDrSw == 0 and CF_Clu_AstDrSw == 0.
// DrvDrSw: byte0 bits 6-7.  AstDrSw: byte6 bits 6-7.
static bool avante_md_clu2_doors_closed(const CANPacket_t *msg) {
  bool driver_door_open    = ((msg->data[0] >> 6U) & 0x3U) != 0U;
  bool passenger_door_open = ((msg->data[6] >> 6U) & 0x3U) != 0U;
  return !driver_door_open && !passenger_door_open;
}

// CLU2: driver seatbelt latched when CF_Clu_DrvSeatBeltSw == 0 (byte2 bits 0-1).
static bool avante_md_clu2_seatbelt_latched(const CANPacket_t *msg) {
  return (msg->data[2] & 0x3U) == 0U;
}

// Torque value encoded in VSM1/VSM2: 12-bit field, offset 2048.
static int avante_md_get_vsm_torque(const CANPacket_t *msg) {
  uint16_t raw = (((uint16_t)msg->data[1] & 0xFU) << 8U) | (uint16_t)msg->data[0];
  return (int)raw - 2048;
}

// ── Readiness predicate helpers ───────────────────────────────────────────────

// Returns true when all Vehicle bus messages are fresh AND rx_check passes for
// VSM1/TCU2 (checksum+counter) and CLU1/CLU2 (counter).
static bool avante_md_vehicle_bus_ready(uint32_t now) {
  bool messages_fresh =
      !avante_md_rx_state_stale(&avante_md_tcs1_state,    now, AVANTE_MD_STALE_FAST_US) &&
      !avante_md_rx_state_stale(&avante_md_vsm1_state,    now, AVANTE_MD_STALE_FAST_US) &&
      !avante_md_rx_state_stale(&avante_md_tcs5_state,    now, AVANTE_MD_STALE_FAST_US) &&
      !avante_md_rx_state_stale(&avante_md_esp2_state,    now, AVANTE_MD_STALE_FAST_US) &&
      !avante_md_rx_state_stale(&avante_md_whl_pul_state, now, AVANTE_MD_STALE_FAST_US) &&
      !avante_md_rx_state_stale(&avante_md_clu1_state,    now, AVANTE_MD_STALE_FAST_US) &&
      !avante_md_rx_state_stale(&avante_md_clu2_state,    now, AVANTE_MD_STALE_10HZ_US) &&
      !avante_md_rx_state_stale(&avante_md_tcu1_state,    now, AVANTE_MD_STALE_FAST_US) &&
      !avante_md_rx_state_stale(&avante_md_tcu2_state,    now, AVANTE_MD_STALE_FAST_US);

  bool safety_checks_pass =
      avante_md_rx_check_valid((int)AVANTE_MD_VSM1, AVANTE_MD_VEHICLE_BUS) &&
      avante_md_rx_check_valid((int)AVANTE_MD_TCU2, AVANTE_MD_VEHICLE_BUS) &&
      avante_md_rx_check_valid((int)AVANTE_MD_CLU1, AVANTE_MD_VEHICLE_BUS) &&
      avante_md_rx_check_valid((int)AVANTE_MD_CLU2, AVANTE_MD_VEHICLE_BUS);

  return messages_fresh && safety_checks_pass;
}

// Returns true when all EPS bus messages are fresh AND rx_check passes for
// VSM2/SAS1 (checksum+counter).
static bool avante_md_eps_bus_ready(uint32_t now) {
  bool messages_fresh =
      !avante_md_rx_state_stale(&avante_md_vsm2_state,  now, AVANTE_MD_STALE_FAST_US) &&
      !avante_md_rx_state_stale(&avante_md_sas1_state,  now, AVANTE_MD_STALE_FAST_US) &&
      !avante_md_rx_state_stale(&avante_md_mdps1_state, now, AVANTE_MD_STALE_10HZ_US);

  bool safety_checks_pass =
      avante_md_rx_check_valid((int)AVANTE_MD_VSM2, AVANTE_MD_EPS_BUS) &&
      avante_md_rx_check_valid((int)AVANTE_MD_SAS1, AVANTE_MD_EPS_BUS);

  return messages_fresh && safety_checks_pass;
}

// Returns true when all Vehicle bus content conditions are normal.
static bool avante_md_vehicle_state_normal(void) {
  return avante_md_vsm1_normal       &&
         avante_md_tcu1_drive        &&
         avante_md_tcu2_drive        &&
         avante_md_doors_closed      &&
         avante_md_seatbelt_latched  &&
         avante_md_parking_brake_off;
}

// Returns true when all EPS bus content conditions are normal.
static bool avante_md_eps_state_normal(void) {
  return avante_md_vsm2_normal && avante_md_sas1_valid;
}

// ── Controls-allowed auto-management ─────────────────────────────────────────

static void avante_md_disengage_controls(void) {
  controls_allowed   = false;
  desired_torque_last = 0;
  rt_torque_last     = 0;
}

// Evaluates all TX-unblock prerequisites (condition 1: vehicle/EPS state).
// Automatically sets controls_allowed when ready AND 100ms have elapsed since
// the last TX-blocking condition was observed; disengages when not ready.
// Returns true only when both ready and stabilized.
static bool avante_md_control_prereqs_normal(void) {
  uint32_t now   = microsecond_timer_get();
  bool     ready = avante_md_vehicle_bus_ready(now)  &&
                   avante_md_eps_bus_ready(now)       &&
                   avante_md_vehicle_state_normal()   &&
                   avante_md_eps_state_normal();

  if (!ready) {
    avante_md_block_last_time = now;
    avante_md_block_seen      = true;
    avante_md_disengage_controls();
  } else {
    bool stabilized = !avante_md_block_seen ||
                      (safety_get_ts_elapsed(now, avante_md_block_last_time) >=
                       AVANTE_MD_STABILIZE_US);
    if (stabilized) {
      controls_allowed = true;
    }
  }

  bool stabilized_now = !avante_md_block_seen ||
                        (safety_get_ts_elapsed(now, avante_md_block_last_time) >=
                         AVANTE_MD_STABILIZE_US);
  return ready && stabilized_now;
}

// ── TX VSM1 message validity checks (condition 2) ────────────────────────────

static bool avante_md_vsm1_tx_checksum_valid(const CANPacket_t *msg) {
  return avante_md_get_checksum(msg) == avante_md_compute_checksum(msg);
}

// CF_Esc_Def must be 0 (byte2 bit0).
static bool avante_md_vsm1_tx_defect_clear(const CANPacket_t *msg) {
  return (msg->data[2] & 0x1U) == 0U;
}

// CF_Esc_Act (steer_req) = byte1 bit4.  CF_Esc_CtrMode = byte1 bits5-7.
// Valid: (act=1 -> mode=2) or (act=0 -> mode=0).
static bool avante_md_vsm1_tx_control_mode_valid(const CANPacket_t *msg) {
  bool    steer_req = (msg->data[1] & 0x10U) != 0U;
  uint8_t ctr_mode  = (msg->data[1] >> 5U) & 0x7U;
  return (steer_req && (ctr_mode == 2U)) || (!steer_req && (ctr_mode == 0U));
}

// Minimum inter-TX interval: 7 ms.
static bool avante_md_vsm1_tx_interval_ok(uint32_t now) {
  uint32_t elapsed = safety_get_ts_elapsed(now, avante_md_vsm1_tx_last_time);
  return !avante_md_vsm1_tx_seen || (elapsed >= AVANTE_MD_VSM1_TX_MIN_US);
}

// Steering torque safety limits.
static bool avante_md_vsm1_tx_torque_valid(const CANPacket_t *msg) {
  static const TorqueSteeringLimits AVANTE_MD_STEERING_LIMITS = {
    .max_torque             = AVANTE_MD_MAX_STEER_TORQUE,
    .max_rate_up            = AVANTE_MD_MAX_RATE_UP,
    .max_rate_down          = AVANTE_MD_MAX_RATE_DOWN,
    .max_rt_delta           = AVANTE_MD_MAX_RT_DELTA,
    .driver_torque_multiplier = 1,
    .driver_torque_allowance  = AVANTE_MD_DRIVER_ALLOWANCE,
    .type                   = TorqueDriverLimited,
  };

  int  desired_torque = avante_md_get_vsm_torque(msg);
  bool steer_req      = (msg->data[1] & 0x10U) != 0U;
  return !steer_torque_cmd_checks(desired_torque, steer_req, AVANTE_MD_STEERING_LIMITS);
}

// Aggregated TX message validity (condition 2).
static bool avante_md_vsm1_tx_msg_valid(const CANPacket_t *msg, uint32_t now) {
  return avante_md_vsm1_tx_checksum_valid(msg)    &&
         avante_md_vsm1_tx_defect_clear(msg)      &&
         avante_md_vsm1_tx_control_mode_valid(msg) &&
         avante_md_vsm1_tx_interval_ok(now)       &&
         avante_md_vsm1_tx_torque_valid(msg);
}

// ── Speed update from TCS5 ────────────────────────────────────────────────────

static void avante_md_update_speed_from_tcs5(const CANPacket_t *msg) {
  uint16_t wheel_fl  = (uint16_t)msg->data[2] | (((uint16_t)msg->data[3] & 0xFU) << 8U);
  uint16_t wheel_fr  = ((uint16_t)msg->data[3] >> 4U) | ((uint16_t)msg->data[4] << 4U);
  uint16_t wheel_rl  = (uint16_t)msg->data[5] | (((uint16_t)msg->data[6] & 0xFU) << 8U);
  uint16_t wheel_rr  = ((uint16_t)msg->data[6] >> 4U) | ((uint16_t)msg->data[7] << 4U);
  uint32_t wheel_sum = (uint32_t)wheel_fl + (uint32_t)wheel_fr +
                       (uint32_t)wheel_rl + (uint32_t)wheel_rr;
  float wheel_avg = (float)wheel_sum / 4.0F;
  float speed_ms = wheel_avg * 0.125F / 3.6F;
  UPDATE_VEHICLE_SPEED(speed_ms);
  vehicle_moving = speed_ms > 0.1F;
}

// ── Safety hooks ─────────────────────────────────────────────────────────────

static void avante_md_rx_hook(const CANPacket_t *msg) {
  uint32_t now = microsecond_timer_get();

  if (msg->bus == (unsigned char)AVANTE_MD_VEHICLE_BUS) {
    if (msg->addr == AVANTE_MD_TCS1) {
      avante_md_update_rx_state(&avante_md_tcs1_state, now);
    }

    if (msg->addr == AVANTE_MD_VSM1) {
      avante_md_update_rx_state(&avante_md_vsm1_state, now);
      avante_md_vsm1_normal = avante_md_vsm1_normal_state(msg);
    }

    if (msg->addr == AVANTE_MD_TCS5) {
      avante_md_update_rx_state(&avante_md_tcs5_state, now);
      avante_md_update_speed_from_tcs5(msg);
    }

    if (msg->addr == AVANTE_MD_ESP2) {
      avante_md_update_rx_state(&avante_md_esp2_state, now);
    }

    if (msg->addr == AVANTE_MD_WHL_PUL) {
      avante_md_update_rx_state(&avante_md_whl_pul_state, now);
    }

    if (msg->addr == AVANTE_MD_CLU1) {
      avante_md_update_rx_state(&avante_md_clu1_state, now);
      avante_md_parking_brake_off = avante_md_clu1_parking_brake_off(msg);
    }

    if (msg->addr == AVANTE_MD_CLU2) {
      avante_md_update_rx_state(&avante_md_clu2_state, now);
      avante_md_doors_closed     = avante_md_clu2_doors_closed(msg);
      avante_md_seatbelt_latched = avante_md_clu2_seatbelt_latched(msg);
    }

    if (msg->addr == AVANTE_MD_TCU1) {
      avante_md_update_rx_state(&avante_md_tcu1_state, now);
      avante_md_tcu1_drive = avante_md_tcu1_in_drive(msg);
    }

    if (msg->addr == AVANTE_MD_TCU2) {
      avante_md_update_rx_state(&avante_md_tcu2_state, now);
      avante_md_tcu2_drive = avante_md_tcu2_in_drive(msg);
    }
  }

  if (msg->bus == (unsigned char)AVANTE_MD_EPS_BUS) {
    if (msg->addr == AVANTE_MD_VSM2) {
      avante_md_update_rx_state(&avante_md_vsm2_state, now);
      avante_md_vsm2_normal = avante_md_vsm2_fault_free(msg);
      update_sample(&torque_driver, avante_md_get_vsm_torque(msg));
    }

    if (msg->addr == AVANTE_MD_SAS1) {
      avante_md_update_rx_state(&avante_md_sas1_state, now);
      avante_md_sas1_valid = avante_md_sas1_status_valid(msg);
    }

    if (msg->addr == AVANTE_MD_MDPS1) {
      avante_md_update_rx_state(&avante_md_mdps1_state, now);
    }
  }

  // Auto-manage controls_allowed on every accepted prerequisite RX. This treats
  // the initial unseen/stale prerequisite state as a real blocking condition, so
  // the first full normal prerequisite set still needs the 100 ms stabilization
  // window before controls are allowed.
  (void)avante_md_control_prereqs_normal();
}

static bool avante_md_tx_hook(const CANPacket_t *msg) {
  bool tx = true;

  if ((msg->addr == AVANTE_MD_VSM1) &&
      (msg->bus  == (unsigned char)AVANTE_MD_EPS_BUS)) {
    uint32_t now = microsecond_timer_get();

    // Fresh prerequisite check at TX time catches stale messages not yet
    // detected by the rx_hook (e.g., if a message went stale since last RX).
    bool prereqs_ok = avante_md_control_prereqs_normal();
    tx = prereqs_ok && avante_md_vsm1_tx_msg_valid(msg, now);

    if (tx) {
      avante_md_vsm1_tx_seen      = true;
      avante_md_vsm1_tx_last_time = now;
    }
  }

  return tx;
}

// Returns true when openpilot sent VSM1 within the recent window (30 ms).
static bool avante_md_op_vsm1_recent(uint32_t now) {
  return avante_md_vsm1_tx_seen &&
         (safety_get_ts_elapsed(now, avante_md_vsm1_tx_last_time) <=
          AVANTE_MD_OP_VSM1_RECENT_US);
}

static bool avante_md_fwd_hook(int bus_num, int addr) {
  bool block = false;

  if (addr == (int)AVANTE_MD_VSM1) {
    if (bus_num == (int)AVANTE_MD_VEHICLE_BUS) {
      // Block forwarding when: prereqs normal AND openpilot sent VSM1 recently.
      // Allow forwarding otherwise so the MDPS receives the stock VSM1.
      uint32_t now = microsecond_timer_get();
      block = avante_md_control_prereqs_normal() &&
              avante_md_op_vsm1_recent(now);
    } else if (bus_num == (int)AVANTE_MD_EPS_BUS) {
      // MDPS VSM1 feedback is always blocked from reaching bus 0.
      block = true;
    } else {
      // no action
    }
  }

  return block;
}

// ── State reset and init ──────────────────────────────────────────────────────

static void avante_md_reset_state(void) {
  avante_md_reset_rx_state(&avante_md_tcs1_state);
  avante_md_reset_rx_state(&avante_md_vsm1_state);
  avante_md_reset_rx_state(&avante_md_tcs5_state);
  avante_md_reset_rx_state(&avante_md_esp2_state);
  avante_md_reset_rx_state(&avante_md_whl_pul_state);
  avante_md_reset_rx_state(&avante_md_clu1_state);
  avante_md_reset_rx_state(&avante_md_clu2_state);
  avante_md_reset_rx_state(&avante_md_tcu1_state);
  avante_md_reset_rx_state(&avante_md_tcu2_state);
  avante_md_reset_rx_state(&avante_md_vsm2_state);
  avante_md_reset_rx_state(&avante_md_sas1_state);
  avante_md_reset_rx_state(&avante_md_mdps1_state);

  avante_md_vsm1_normal       = false;
  avante_md_vsm2_normal       = false;
  avante_md_sas1_valid        = false;
  avante_md_tcu1_drive        = false;
  avante_md_tcu2_drive        = false;
  avante_md_doors_closed      = false;
  avante_md_seatbelt_latched  = false;
  avante_md_parking_brake_off = false;

  avante_md_vsm1_tx_seen      = false;
  avante_md_vsm1_tx_last_time = 0U;

  avante_md_block_seen      = false;
  avante_md_block_last_time = 0U;
}

static safety_config avante_md_init(uint16_t param) {
  static const CanMsg AVANTE_MD_TX_MSGS[] = {
    {AVANTE_MD_VSM1, AVANTE_MD_EPS_BUS, 8,
     .check_relay = true, .disable_static_blocking = true},
  };

  static RxCheck avante_md_rx_checks[] = {
    // Vehicle bus — staleness only (checksum verified by framework, counter not required)
    {.msg = {{AVANTE_MD_TCS1, AVANTE_MD_VEHICLE_BUS, 8, 100U,
              .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    // Vehicle bus — checksum + counter required (VSM1)
    {.msg = {{AVANTE_MD_VSM1, AVANTE_MD_VEHICLE_BUS, 8, 100U,
              .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    // Vehicle bus — staleness only
    {.msg = {{AVANTE_MD_TCS5, AVANTE_MD_VEHICLE_BUS, 8, 50U,
              .ignore_checksum = true, .ignore_counter = true,
              .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{AVANTE_MD_ESP2, AVANTE_MD_VEHICLE_BUS, 8, 100U,
              .ignore_checksum = true, .ignore_counter = true,
              .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{AVANTE_MD_WHL_PUL, AVANTE_MD_VEHICLE_BUS, 8, 100U,
              .ignore_counter = true, .ignore_quality_flag = true}, {0}, {0}}},
    // Vehicle bus — counter required (CLU1, CLU2)
    {.msg = {{AVANTE_MD_CLU1, AVANTE_MD_VEHICLE_BUS, 8, 50U,
              .ignore_checksum = true, .max_counter = 127U,
              .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{AVANTE_MD_CLU2, AVANTE_MD_VEHICLE_BUS, 8, 10U,
              .ignore_checksum = true, .max_counter = 15U,
              .ignore_quality_flag = true}, {0}, {0}}},
    // Vehicle bus — staleness only (TCU1)
    {.msg = {{AVANTE_MD_TCU1, AVANTE_MD_VEHICLE_BUS, 8, 100U,
              .ignore_checksum = true, .ignore_counter = true,
              .ignore_quality_flag = true}, {0}, {0}}},
    // Vehicle bus — checksum + counter required (TCU2)
    {.msg = {{AVANTE_MD_TCU2, AVANTE_MD_VEHICLE_BUS, 8, 100U,
              .max_counter = 3U, .ignore_quality_flag = true}, {0}, {0}}},
    // EPS bus — checksum + counter required (VSM2, SAS1)
    {.msg = {{AVANTE_MD_VSM2, AVANTE_MD_EPS_BUS, 8, 100U,
              .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    {.msg = {{AVANTE_MD_SAS1, AVANTE_MD_EPS_BUS, 5, 100U,
              .max_counter = 15U, .ignore_quality_flag = true}, {0}, {0}}},
    // EPS bus — staleness only (MDPS1)
    {.msg = {{AVANTE_MD_MDPS1, AVANTE_MD_EPS_BUS, 3, 100U,
              .ignore_checksum = true, .ignore_counter = true,
              .ignore_quality_flag = true}, {0}, {0}}},
  };

  avante_md_reset_state();

  SAFETY_UNUSED(param);
  return BUILD_SAFETY_CFG(avante_md_rx_checks, AVANTE_MD_TX_MSGS);
}

const safety_hooks avante_md_hooks = {
  .init             = avante_md_init,
  .rx               = avante_md_rx_hook,
  .tx               = avante_md_tx_hook,
  .fwd              = avante_md_fwd_hook,
  .get_counter      = avante_md_get_counter,
  .get_checksum     = avante_md_get_checksum,
  .compute_checksum = avante_md_compute_checksum,
};
