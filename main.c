/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "nordic_common.h"
#include "sdk_config.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_button.h"
#include "app_error.h"
#include "app_pwm.h"
#include "app_timer.h"
#include "app_uart.h"
#include "app_util_platform.h"

#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_cus.h"
#include "ble_cus_c.h"
#include "ble_err.h"
#include "ble_fs.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "boards.h"
#include "bsp_btn_ble.h"

#include "dht.h"

#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_ble_scan.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_timer.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_uart.h"

#include "peer_manager_handler.h"
#include "pin_mapping.h"

#define DEVICE_NAME "RAPIDO"

/////////////////// Instances of Other Modules ///////////////////////////////
APP_PWM_INSTANCE(BLE_PWM1, 1); // Create the instance "BLE_PWM" using TIMER1.

APP_TIMER_DEF(m_app_timer);

/////////////// static variables //////////////////////////
static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

static uint8_t m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;           /**< Advertising handle used to identify an advertising set. */
static uint8_t m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];            /**< Buffer for storing an encoded advertising set. */
static uint8_t m_enc_scan_response_data[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; /**< Buffer for storing an encoded scan data. */
static nrf_ppi_channel_t m_ppi_channel, ppi_channel_0, ppi_channel_1, ppi_channel_2, ppi_channel_3,
    ppi_channel_4, ppi_channel_5, ppi_channel_6, ppi_channel_7; // ppi channels

static bool ble_pwm1_enable_flag = false;

uint32_t pwm1_frequency = 0;

uint8_t status_ack[8] = {'X', 'X', 'X', 'X', 'X', 'X', 'X', 'X'};

#define SAMPLES_IN_BUFFER 1

/////////////// static functions Decleration ////////////////
static void init_ble_pwm1(uint32_t pin, uint32_t freq, float dutyCycle);

static void deinit_ble_pwm1();

#define CONN_INTERVAL_DEFAULT (uint16_t)(MSEC_TO_UNITS(7.5, UNIT_1_25_MS)) /**< Default connection interval used at connection establishment by central side. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Minimum acceptable connection interval (7.5 mseconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(500, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.5 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(20000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000)   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                        /**< Number of attempts before giving up the connection parameter negotiation. */

#define TIMER_1_TIMEOUT_US_1 200
#define TIMER_1_TIMEOUT_US_2 400
#define TIMER_1_TIMEOUT_US_3 600
#define TIMER_1_TIMEOUT_US_4 800
#define TIMER_ADC_TIMEOUT_US 200

//================================================================
/**  GPIO pin setup  **/


// GPIO pins for real mode (EYSHSNZWZ)
int OUTPUT_LED            = 4;
int OUTPUT_PUMP_LEFT_LV1  = 6;  static bool pump_left_lv1_flag  = false;
int OUTPUT_PUMP_LEFT_LV2  = 8;  static bool pump_left_lv2_flag  = false;
int OUTPUT_PUMP_LEFT_LV3  = 7;  static bool pump_left_lv3_flag  = false;
int OUTPUT_PUMP_RIGHT_LV1 = 5;  static bool pump_right_lv1_flag = false;
int OUTPUT_PUMP_RIGHT_LV2 = 18; static bool pump_right_lv2_flag = false;
int OUTPUT_PUMP_RIGHT_LV3 = 20; static bool pump_right_lv3_flag = false;

/*
// GPIO pins for test mode (nRF52840-DK)
int OUTPUT_LED            = 27;
int OUTPUT_PUMP_LEFT_LV1  = 26;  static bool pump_left_lv1_flag  = false;
int OUTPUT_PUMP_LEFT_LV2  = 4;   static bool pump_left_lv2_flag  = false;
int OUTPUT_PUMP_LEFT_LV3  = 28;  static bool pump_left_lv3_flag  = false;
int OUTPUT_PUMP_RIGHT_LV1 = 29;  static bool pump_right_lv1_flag = false;
int OUTPUT_PUMP_RIGHT_LV2 = 30;  static bool pump_right_lv2_flag = false;
int OUTPUT_PUMP_RIGHT_LV3 = 31;  static bool pump_right_lv3_flag = false;
*/
//================================================================

//================================================================
/**  Frequency & Duty cycle  **/
#define FREQ1 5
#define FREQ2 10
#define FREQ3 20
#define FREQ4 40

#define DUTY1 5
#define DUTY2 10 
#define DUTY3 20
#define DUTY4 40
//================================================================

/**  Variables required for pump actuation  **/
static bool app_timer_flag       = false;

uint16_t electrolysis_timer_sec  = 0;

uint32_t pump_actuation_time     = 0;

#define PUMP_INTERVAL APP_TIMER_TICKS(1000)  // Tick every seconds
//================================================================

#define LESC_MITM_NC 0

/** @brief The maximum number of peripheral and central links combined. */
#define NRF_BLE_LINK_COUNT (NRF_SDH_BLE_PERIPHERAL_LINK_COUNT + NRF_SDH_BLE_CENTRAL_LINK_COUNT)

#define APP_BLE_CONN_CFG_TAG 1 /**< Tag that identifies the SoftDevice BLE configuration. */

#define CENTRAL_SCANNING_LED BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED BSP_BOARD_LED_1
#define PERIPHERAL_ADVERTISING_LED BSP_BOARD_LED_2
#define PERIPHERAL_CONNECTED_LED BSP_BOARD_LED_3

#define SCAN_DURATION 0x0000
#define APP_ADV_INTERVAL 64    /**< Duration of the scanning in units of 10 milliseconds. If set to 0x0000, scanning continues until it is explicitly disabled. */
#define APP_ADV_DURATION 18000 /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define SEC_PARAMS_BOND 1 /**< Perform bonding. */
#if LESC_MITM_NC
#define SEC_PARAMS_MITM 1                                        /**< Man In The Middle protection required. */
#define SEC_PARAMS_IO_CAPABILITIES BLE_GAP_IO_CAPS_DISPLAY_YESNO /**< Display Yes/No to force Numeric Comparison. */
#else
#define SEC_PARAMS_MITM 0                               /**< Man In The Middle protection required. */
#define SEC_PARAMS_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O caps. */
#endif
#define SEC_PARAMS_LESC 0          /**< LE Secure Connections pairing required. */
#define SEC_PARAMS_KEYPRESS 0      /**< Keypress notifications not required. */
#define SEC_PARAMS_OOB 0           /**< Out Of Band data not available. */
#define SEC_PARAMS_MIN_KEY_SIZE 7  /**< Minimum encryption key size in octets. */
#define SEC_PARAMS_MAX_KEY_SIZE 16 /**< Maximum encryption key size in octets. */

// #define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                           /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
// #define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                          /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3      /**< Number of attempts before giving up the connection parameter negotiation. */
#define UART_INTERVAL APP_TIMER_TICKS(2000) /**< Uart measurement interval (ticks). */
#define UART_TX_BUF_SIZE 64                 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 32                 /**< UART RX buffer size. */

#define APP_BLE_OBSERVER_PRIO 3

typedef struct {
  bool is_connected;
  ble_gap_addr_t address;
} conn_peer_t;

NRF_BLE_GQ_DEF(m_ble_gatt_queue, /**< BLE GATT Queue instance. */
    NRF_SDH_BLE_CENTRAL_LINK_COUNT,
    NRF_BLE_GQ_QUEUE_SIZE);

BLE_CUS_DEF(m_cus);                                    /**< Heart Rate Service instance. */
BLE_CUS_C_DEF(m_cus_c);                                /**< Structure used to identify the Heart Rate client module. */
NRF_BLE_GATT_DEF(m_gatt);                              /**< GATT module instance. */
NRF_BLE_QWRS_DEF(m_qwr, NRF_SDH_BLE_TOTAL_LINK_COUNT); /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                    /**< Advertising module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc);                       /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                              /**< Scanning Module instance. */

static uint16_t m_conn_handle_cus_c = BLE_CONN_HANDLE_INVALID;                        /**< Connection handle for the HRS central application. */
static volatile uint16_t m_conn_handle_num_comp_central = BLE_CONN_HANDLE_INVALID;    /**< Connection handle for the central that needs a numeric comparison button press. */
static volatile uint16_t m_conn_handle_num_comp_peripheral = BLE_CONN_HANDLE_INVALID; /**< Connection handle for the peripheral that needs a numeric comparison button press. */

static conn_peer_t m_connected_peers[NRF_BLE_LINK_COUNT]; /**< Array of connected peers. */

uint8_t uart_ble_data[2][6]; // 2 arrays of 3 analog values
uint8_t uart_array_check = 0x00;
static char *roles_str[] = {
    "INVALID_ROLE",
    "PERIPHERAL",
    "CENTRAL",
};

static const char m_target_periph_name[] = "Nordic_Template";

static ble_uuid_t m_adv_uuids[] = {{CUSTOM_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}};


// cus_c_error_handler()
static void cus_c_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}


// conn_params_error_handler()
static void conn_params_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}


// scan_start()
static void scan_start(void) {
  ret_code_t err_code;

  err_code = nrf_ble_scan_start(&m_scan);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_INFO("Scanning Starting\n");
}


// scan_stop()
static void scan_stop(void) {
  ret_code_t err_code;

  nrf_ble_scan_stop();
  NRF_LOG_INFO("Scanning Stopped\n");
}


// adv_scan_start()
static void adv_scan_start(void) {
  ret_code_t err_code;

  // Start advertising.
  err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_INFO("Advertising");
}


// pm_evt_handler()
static void pm_evt_handler(pm_evt_t const *p_evt) {
  pm_handler_on_pm_evt(p_evt);
  pm_handler_flash_clean(p_evt);

  switch (p_evt->evt_id) {
  case PM_EVT_PEERS_DELETE_SUCCEEDED:
    adv_scan_start();
    break;

  default:
    break;
  }
}


// filter_settings_change()
static void filter_settings_change(void) {
  ret_code_t err_code;

  err_code = nrf_ble_scan_all_filter_remove(&m_scan);
  APP_ERROR_CHECK(err_code);

  if (strlen(m_target_periph_name) != 0) {
    err_code = nrf_ble_scan_filter_set(&m_scan,
        SCAN_NAME_FILTER,
        m_target_periph_name);
    APP_ERROR_CHECK(err_code);
  }
}


// cus_c_evt_handler()
static void cus_c_evt_handler(ble_cus_c_t *p_cus_c, ble_cus_c_evt_t *p_cus_c_evt) {
  ret_code_t err_code;

  switch (p_cus_c_evt->evt_type) {
  case BLE_CUS_C_EVT_DISCOVERY_COMPLETE: {
    if (m_conn_handle_cus_c == BLE_CONN_HANDLE_INVALID) {
      ret_code_t err_code;

      m_conn_handle_cus_c = p_cus_c_evt->conn_handle;

      // We do not want to connect to two peripherals offering the same service, so when
      // a UUID is matched, we check whether we are not already connected to a peer which
      // offers the same service
      filter_settings_change();

      err_code = ble_cus_c_handles_assign(p_cus_c,
          m_conn_handle_cus_c,
          &p_cus_c_evt->params.peer_db);
      APP_ERROR_CHECK(err_code);

      // Heart rate service discovered. Enable notification of Heart Rate Measurement.
      err_code = ble_cus_c_data_char_notif_enable(p_cus_c);
      APP_ERROR_CHECK(err_code);
    }
  } break; // BLE_cus_C_EVT_DISCOVERY_COMPLETE

  case BLE_CUS_C_EVT_DATA_NOTIFICATION: {
    NRF_LOG_INFO("CENTRAL: Received Data len %d. Data is\n", p_cus_c_evt->params.data_char.length);
    NRF_LOG_HEXDUMP_INFO(p_cus_c_evt->params.data_char.p_data, p_cus_c_evt->params.data_char.length);

    for (uint8_t i = 0; i < p_cus_c_evt->params.data_char.length; i++) {
      uart_ble_data[uart_array_check][i] = p_cus_c_evt->params.data_char.p_data[i];
      NRF_LOG_INFO("uart_ble_data[%d][%d] =  %d\n", uart_array_check, i, uart_ble_data[uart_array_check][i]);
    }

    uart_array_check ^= 0x01; // choose the altermative arrays next time
  } break;

  default:
    break;
  }
}


// is_already_connected()
static bool is_already_connected(ble_gap_addr_t const *p_connected_adr) {
  for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++) {
    if (m_connected_peers[i].is_connected) {
      if (m_connected_peers[i].address.addr_type == p_connected_adr->addr_type) {
        if (memcmp(m_connected_peers[i].address.addr,
                p_connected_adr->addr,
                sizeof(m_connected_peers[i].address.addr)) == 0) {
          return true;
        }
      }
    }
  }
  return false;
}


// on_match_request()
static void on_match_request(uint16_t conn_handle, uint8_t role) {
  // Mark the appropriate conn_handle as pending. The rest is handled on button press.
  NRF_LOG_INFO("Press Button 1 to confirm, Button 2 to reject");
  if (role == BLE_GAP_ROLE_CENTRAL) {
    m_conn_handle_num_comp_central = conn_handle;
  } else if (role == BLE_GAP_ROLE_PERIPH) {
    m_conn_handle_num_comp_peripheral = conn_handle;
  }
}


// multi_qwr_conn_handle_assign()
static void multi_qwr_conn_handle_assign(uint16_t conn_handle) {
  for (uint32_t i = 0; i < NRF_BLE_LINK_COUNT; i++) {
    if (m_qwr[i].conn_handle == BLE_CONN_HANDLE_INVALID) {
      ret_code_t err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr[i], conn_handle);
      APP_ERROR_CHECK(err_code);
      break;
    }
  }
}


// on_ble_evt()
static void on_ble_evt(uint16_t conn_handle, ble_evt_t const *p_ble_evt) {
  char passkey[BLE_GAP_PASSKEY_LEN + 1];
  uint16_t role = ble_conn_state_role(conn_handle);

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    m_connected_peers[conn_handle].is_connected = true;
    m_connected_peers[conn_handle].address = p_ble_evt->evt.gap_evt.params.connected.peer_addr;
    multi_qwr_conn_handle_assign(conn_handle);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    memset(&m_connected_peers[conn_handle], 0x00, sizeof(m_connected_peers[0]));
    break;

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    NRF_LOG_INFO("%s: BLE_GAP_EVT_SEC_PARAMS_REQUEST", nrf_log_push(roles_str[role]));
    break;

  case BLE_GAP_EVT_PASSKEY_DISPLAY:
    memcpy(passkey, p_ble_evt->evt.gap_evt.params.passkey_display.passkey, BLE_GAP_PASSKEY_LEN);
    passkey[BLE_GAP_PASSKEY_LEN] = 0x00;

    NRF_LOG_INFO("%s: BLE_GAP_EVT_PASSKEY_DISPLAY: passkey=%s match_req=%d",
        nrf_log_push(roles_str[role]),
        nrf_log_push(passkey),
        p_ble_evt->evt.gap_evt.params.passkey_display.match_request);

    if (p_ble_evt->evt.gap_evt.params.passkey_display.match_request) {
      on_match_request(conn_handle, role);
    }
    break;

  case BLE_GAP_EVT_AUTH_KEY_REQUEST:
    NRF_LOG_INFO("%s: BLE_GAP_EVT_AUTH_KEY_REQUEST", nrf_log_push(roles_str[role]));
    break;

  case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
    NRF_LOG_INFO("%s: BLE_GAP_EVT_LESC_DHKEY_REQUEST", nrf_log_push(roles_str[role]));
    break;

  case BLE_GAP_EVT_AUTH_STATUS:
    NRF_LOG_INFO("%s: BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
        nrf_log_push(roles_str[role]),
        p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
        p_ble_evt->evt.gap_evt.params.auth_status.bonded,
        p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
        *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
        *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
    NRF_LOG_DEBUG("PHY update request.");
    ble_gap_phys_t const phys = {
        .rx_phys = BLE_GAP_PHY_AUTO,
        .tx_phys = BLE_GAP_PHY_AUTO,
    };
    ret_code_t err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
  } break;

  default:
    break;
  }
}


// on_ble_central_evt()
static void on_ble_central_evt(ble_evt_t const *p_ble_evt) {
  ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;
  ret_code_t err_code;

  switch (p_ble_evt->header.evt_id) {
  // Upon connection, check which peripheral is connected (HR or RSC), initiate DB
  //  discovery, update LEDs status, and resume scanning, if necessary.
  case BLE_GAP_EVT_CONNECTED: {
    NRF_LOG_INFO("CENTRAL: Connected, handle: %d.", p_gap_evt->conn_handle);
    // If no Heart Rate Sensor is currently connected, try to find them on this peripheral.
    if (m_conn_handle_cus_c == BLE_CONN_HANDLE_INVALID) {
      NRF_LOG_INFO("CENTRAL: Searching for cus on conn_handle 0x%x", p_gap_evt->conn_handle);

      err_code = ble_db_discovery_start(&m_db_disc, p_gap_evt->conn_handle);
      APP_ERROR_CHECK(err_code);
    }
  } break; // BLE_GAP_EVT_CONNECTED

  // Upon disconnection, reset the connection handle of the peer that disconnected, update
  // the status of LEDs, and start scanning again.
  case BLE_GAP_EVT_DISCONNECTED: {
    NRF_LOG_INFO("CENTRAL: Disconnected, handle: %d, reason: 0x%x",
        p_gap_evt->conn_handle,
        p_gap_evt->params.disconnected.reason);

    if (p_gap_evt->conn_handle == m_conn_handle_cus_c) {
      ble_uuid_t target_uuid = {.uuid = CUSTOM_SERVICE_UUID, .type = BLE_UUID_TYPE_VENDOR_BEGIN};
      m_conn_handle_cus_c = BLE_CONN_HANDLE_INVALID;

      err_code = nrf_ble_scan_filter_set(&m_scan,
          SCAN_UUID_FILTER,
          &target_uuid);
      APP_ERROR_CHECK(err_code);
    }
  } break; // BLE_GAP_EVT_DISCONNECTED

  case BLE_GAP_EVT_TIMEOUT: {
    // Timeout for scanning is not specified, so only connection attemps can time out.
    if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN) {
      NRF_LOG_DEBUG("CENTRAL: Connection Request timed out.");
    }
  } break;

  case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST: {
    // Accept parameters requested by peer.
    err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
        &p_gap_evt->params.conn_param_update_request.conn_params);
    APP_ERROR_CHECK(err_code);
  } break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    NRF_LOG_DEBUG("CENTRAL: GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    NRF_LOG_DEBUG("CENTRAL: GATT Server Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  default:
    break;
  }
}


// on_ble_peripheral_evt()
static void on_ble_peripheral_evt(ble_evt_t const *p_ble_evt) {
  ret_code_t err_code;
  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    NRF_LOG_INFO("PERIPHERAL: Connected, handle %d.", p_ble_evt->evt.gap_evt.conn_handle);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    NRF_LOG_INFO("PERIPHERAL: Disconnected, handle %d, reason 0x%x.",
        p_ble_evt->evt.gap_evt.conn_handle,
        p_ble_evt->evt.gap_evt.params.disconnected.reason);

    // LED indication will be changed when advertising starts.
    break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    NRF_LOG_DEBUG("PERIPHERAL: GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    NRF_LOG_DEBUG("PERIPHERAL: GATT Server Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  default:
    break;
  }
}


// on_adv_evt()
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
  switch (ble_adv_evt) {
  case BLE_ADV_EVT_FAST:
    break;

  case BLE_ADV_EVT_IDLE: {
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
  } break;

  default:
    break;
  }
}


// ble_evt_handler()
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
  uint16_t conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
  uint16_t role = ble_conn_state_role(conn_handle);

  if ((p_ble_evt->header.evt_id == BLE_GAP_EVT_CONNECTED) && (is_already_connected(&p_ble_evt->evt.gap_evt.params.connected.peer_addr))) {
    NRF_LOG_INFO("%s: Already connected to this device as %s (handle: %d), disconnecting.",
        (role == BLE_GAP_ROLE_PERIPH) ? "PERIPHERAL" : "CENTRAL",
        (role == BLE_GAP_ROLE_PERIPH) ? "CENTRAL" : "PERIPHERAL",
        conn_handle);

    (void)sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);

    // Do not process the event further.
    return;
  }

  on_ble_evt(conn_handle, p_ble_evt);

  if (role == BLE_GAP_ROLE_PERIPH) {
    // Manages peripheral LEDs.
    on_ble_peripheral_evt(p_ble_evt);
  } else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT)) {
    on_ble_central_evt(p_ble_evt);
  }
}


// cus_c_init()
static void cus_c_init(void) {
  ret_code_t err_code;
  ble_cus_c_init_t cus_c_init_obj;

  cus_c_init_obj.evt_handler = cus_c_evt_handler;
  cus_c_init_obj.error_handler = cus_c_error_handler;
  cus_c_init_obj.p_gatt_queue = &m_ble_gatt_queue;

  err_code = ble_cus_c_init(&m_cus_c, &cus_c_init_obj);
  APP_ERROR_CHECK(err_code);
}


// ble_stack_init()
static void ble_stack_init(void) {
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack by using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


// peer_manager_init()
static void peer_manager_init(void) {
  ble_gap_sec_params_t sec_params;
  ret_code_t err_code;

  err_code = pm_init();
  APP_ERROR_CHECK(err_code);

  memset(&sec_params, 0, sizeof(ble_gap_sec_params_t));

  // Security parameters to be used for all security procedures.
  sec_params.bond = SEC_PARAMS_BOND;
  sec_params.mitm = SEC_PARAMS_MITM;
  sec_params.lesc = SEC_PARAMS_LESC;
  sec_params.keypress = SEC_PARAMS_KEYPRESS;
  sec_params.io_caps = SEC_PARAMS_IO_CAPABILITIES;
  sec_params.oob = SEC_PARAMS_OOB;
  sec_params.min_key_size = SEC_PARAMS_MIN_KEY_SIZE;
  sec_params.max_key_size = SEC_PARAMS_MAX_KEY_SIZE;
  sec_params.kdist_own.enc = 1;
  sec_params.kdist_own.id = 1;
  sec_params.kdist_peer.enc = 1;
  sec_params.kdist_peer.id = 1;

  err_code = pm_sec_params_set(&sec_params);
  APP_ERROR_CHECK(err_code);

  err_code = pm_register(pm_evt_handler);
  APP_ERROR_CHECK(err_code);
}


// delete_bonds()
static void delete_bonds(void) {
  ret_code_t err_code;

  NRF_LOG_INFO("Erase bonds!");

  err_code = pm_peers_delete();
  APP_ERROR_CHECK(err_code);
}


// buttons_leds_init()
static void buttons_leds_init(bool *p_erase_bonds) {
  ret_code_t err_code;
  bsp_event_t startup_event;

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


// gap_params_init()
static void gap_params_init(void) {
  ret_code_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
      (const uint8_t *)DEVICE_NAME,
      strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}


// gatt_init()
static void gatt_init(void) {
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
  APP_ERROR_CHECK(err_code);
}


// conn_params_init()
static void conn_params_init(void) {
  ret_code_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID; // Start upon connection.
  cp_init.disconnect_on_fail = true;
  cp_init.evt_handler = NULL; // Ignore events.
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}


// void_db_disc_handler()
static void db_disc_handler(ble_db_discovery_evt_t *p_evt) {
  ble_cus_on_db_disc_evt(&m_cus_c, p_evt);
}


// db_discovery_init()
static void db_discovery_init(void) {
  ble_db_discovery_init_t db_init;

  memset(&db_init, 0, sizeof(db_init));

  db_init.evt_handler = db_disc_handler;
  db_init.p_gatt_queue = &m_ble_gatt_queue;

  ret_code_t err_code = ble_db_discovery_init(&db_init);
  APP_ERROR_CHECK(err_code);
}


// ble_pwm_ready_callback()
static void ble_pwm_ready_callback(uint32_t pwm_id) {
}


// freq_to_period_us()
static uint32_t freq_to_period_us(uint32_t freq) {
  return (uint32_t)((1.0 / (float)freq) * 1000000);
}


// init_ble_pwm1()
static void init_ble_pwm1(uint32_t pin, uint32_t freq, float dutyCycle) {

  ret_code_t err_code;

  app_pwm_config_t out_cfg = APP_PWM_DEFAULT_CONFIG_1CH(freq_to_period_us(freq), pin); // period = 1/freq

  // Switch the polarity of the second channel.
  out_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;

  // Initialize and enable PWM.
  err_code = app_pwm_init(&BLE_PWM1, &out_cfg, ble_pwm_ready_callback);
  APP_ERROR_CHECK(err_code);
  app_pwm_enable(&BLE_PWM1);
  ble_pwm1_enable_flag = true;
  pwm1_frequency = freq;
  for (uint8_t i = 0; i < out_cfg.num_of_channels; i++) {
    APP_ERROR_CHECK(app_pwm_channel_duty_set(&BLE_PWM1, i, dutyCycle));
  }
}


// deinit_ble_pwm1()
static void deinit_ble_pwm1() {
  app_pwm_disable(&BLE_PWM1);

  APP_ERROR_CHECK(app_pwm_uninit(&BLE_PWM1));
  ble_pwm1_enable_flag = false;
  pwm1_frequency = 0;
}


// pump_all_off() 
static void pump_all_off() {
  if (app_timer_flag) {
    ret_code_t err_code = app_timer_stop(m_app_timer);
    app_timer_flag = false;
    electrolysis_timer_sec = 0;
    pump_actuation_time = 0;
  }

  if (pump_left_lv1_flag) {  // Turn off left pump (Lv.1)
    nrf_gpio_cfg_output(OUTPUT_PUMP_LEFT_LV1);
    nrf_gpio_pin_clear(OUTPUT_PUMP_LEFT_LV1);
    pump_left_lv1_flag = false;
  }

  if (pump_left_lv2_flag) {  // Turn off left pump (Lv.2)
    nrf_gpio_cfg_output(OUTPUT_PUMP_LEFT_LV2);
    nrf_gpio_pin_clear(OUTPUT_PUMP_LEFT_LV2);
    pump_left_lv2_flag = false;
  }

  if (pump_left_lv3_flag) {  // Turn off left pump (Lv.3) 
    nrf_gpio_cfg_output(OUTPUT_PUMP_LEFT_LV3);
    nrf_gpio_pin_clear(OUTPUT_PUMP_LEFT_LV3);
    pump_left_lv3_flag = false;
  }

  if (pump_right_lv1_flag) {  // Turn off right pump (Lv.1)
    nrf_gpio_cfg_output(OUTPUT_PUMP_RIGHT_LV1);
    nrf_gpio_pin_clear(OUTPUT_PUMP_RIGHT_LV1);
    pump_right_lv1_flag = false;
  }

  if (pump_right_lv2_flag) {  // Turn off right pump (Lv.2)
    nrf_gpio_cfg_output(OUTPUT_PUMP_RIGHT_LV2);
    nrf_gpio_pin_clear(OUTPUT_PUMP_RIGHT_LV2);
    pump_right_lv2_flag = false;
  }

  if (pump_right_lv3_flag) {  // Turn off right pump (Lv.3)
    nrf_gpio_cfg_output(OUTPUT_PUMP_RIGHT_LV3);
    nrf_gpio_pin_clear(OUTPUT_PUMP_RIGHT_LV3);
    pump_right_lv3_flag = false;
  }

  // Force full deinitialization of peripheral pins
  nrf_gpio_cfg_input(OUTPUT_PUMP_LEFT_LV1, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(OUTPUT_PUMP_LEFT_LV2, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(OUTPUT_PUMP_LEFT_LV3, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(OUTPUT_PUMP_RIGHT_LV1, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(OUTPUT_PUMP_RIGHT_LV2, NRF_GPIO_PIN_NOPULL);
  nrf_gpio_cfg_input(OUTPUT_PUMP_RIGHT_LV3, NRF_GPIO_PIN_NOPULL);
}


// on_cus_evt()
static void on_cus_evt(ble_cus_t *p_cus_service, ble_cus_evt_t *p_evt, uint8_t *p_data, uint8_t length) {
  ret_code_t err_code;
  uint8_t data[length];

  switch (p_evt->evt_type) {
  case BLE_CUS_EVT_NOTIFICATION_ENABLED:
    break;

  case BLE_CUS_EVT_NOTIFICATION_DISABLED:
    break;

  case BLE_CUS_EVT_CONNECTED:
    break;

  case BLE_CUS_EVT_DISCONNECTED:
    break;

  case BLE_CUS_EVT_WRITE:
    memcpy(data, p_data, length);
    // 'attr_char_value.max_len' has been modified in "ble_cus.c". 
    NRF_LOG_INFO("Data is %s and length is %d", (uint32_t)data, length);

    if (data[0] == 'a') {  // set PWM: LED, 5Hz, 10ms duty
      if (ble_pwm1_enable_flag) {  // If LED was already operating, turn off the LED.
        deinit_ble_pwm1();
        NRF_LOG_INFO("LED was already operating. Turning off the LED...");
      
      } else {
        init_ble_pwm1(OUTPUT_LED, FREQ1, DUTY1);
        NRF_LOG_INFO("Operate LED in 5 Hz...");
      }

      ble_cus_custom_value_update(&m_cus, status_ack, sizeof(status_ack));

    } else if (data[0] == 'b') {  // set PWM: LED, 10Hz, 10ms duty
      if (ble_pwm1_enable_flag) {  // If LED was already operating, turn off the LED.
        deinit_ble_pwm1();
        NRF_LOG_INFO("LED was already operating. Turning off the LED...");
      
      } else {
        init_ble_pwm1(OUTPUT_LED, FREQ2, DUTY2);
        NRF_LOG_INFO("Operate LED in 10 Hz...");
      }

      ble_cus_custom_value_update(&m_cus, status_ack, sizeof(status_ack));

    } else if (data[0] == 'c') {  // set PWM: LED, 20Hz, 10ms duty
      if (ble_pwm1_enable_flag) {  // If LED was already operating, turn off the LED.
        deinit_ble_pwm1();
        NRF_LOG_INFO("LED was already operating. Turning off the LED...");
      
      } else {
        init_ble_pwm1(OUTPUT_LED, FREQ3, DUTY3);
        NRF_LOG_INFO("Operate LED in 20 Hz...");
      }

      ble_cus_custom_value_update(&m_cus, status_ack, sizeof(status_ack));
    
    } else if (data[0] == 'd') {  // set PWM: LED, 40Hz, 10ms duty
      if (ble_pwm1_enable_flag) {  // If LED was already operating, turn off the LED.
        deinit_ble_pwm1();
        NRF_LOG_INFO("LED was already operating. Turning off the LED...");
      
      } else {
        init_ble_pwm1(OUTPUT_LED, FREQ4, DUTY4);
        NRF_LOG_INFO("Operate LED in 40 Hz...");
      }

      ble_cus_custom_value_update(&m_cus, status_ack, sizeof(status_ack));
    
    } else if (data[0] == 'e') { 
      if ((data[1] == 'F') && (data[3] == 'f') && (data[4] == 'P') && (data[6] == 'p') && (data[7] == 'N') && (data[10] == 'n')) {
        // Pump actuation: data = "eF_fP_pN__n"
        // F: Target brain    -> 1(=Left) / 2(=Right) / 3(=Both)
        // P: Flow rate level -> 1(=Lv.1) / 2(=Lv.2) / 3(=Lv.3)
        // N: Actuation time  -> 01-99 sec
        if (app_timer_flag) {  // If pump was already actuating, turn off the pump.
          pump_all_off();
          NRF_LOG_INFO("Pump was already actuating. Turning off the pump(s)...");
      
        } else if ((data[2] == '1') && (data[5] == '1')) {  // Condition #1. Actuate left pump (Lv.1)
          pump_actuation_time = 10*(data[8] - '0') + 1*(data[9] - '0');
          NRF_LOG_INFO("Left pump (Lv.1) actuation time is %d sec.", pump_actuation_time);

          if (pump_actuation_time == 0) {  // If pump actuation time input is 0, turn off the pump.
            pump_all_off();
            NRF_LOG_INFO("Turning off the pump(s)...");
        
          } else if ((pump_actuation_time > 0) && (pump_actuation_time < 100)) {  // Actuation time: 01-99 sec
            electrolysis_timer_sec = 0;
            err_code = app_timer_start(m_app_timer, PUMP_INTERVAL, NULL);
            app_timer_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_LEFT_LV1);
            nrf_gpio_pin_set(OUTPUT_PUMP_LEFT_LV1);
            pump_left_lv1_flag = true;
          }
      
        } else if ((data[2] == '1') && (data[5] == '2')) {  // Condition #2. Actuate left pump (Lv.2)
          pump_actuation_time = 10*(data[8] - '0') + 1*(data[9] - '0');
          NRF_LOG_INFO("Left pump (Lv.2) actuation time is %d sec.", pump_actuation_time);

          if (pump_actuation_time == 0) {  // If pump actuation time input is 0, turn off the pump.
            pump_all_off();
            NRF_LOG_INFO("Turning off the pump(s)...");
        
          } else if ((pump_actuation_time > 0) && (pump_actuation_time < 100)) {  // Actuation time: 01-99 sec
            electrolysis_timer_sec = 0;
            err_code = app_timer_start(m_app_timer, PUMP_INTERVAL, NULL);
            app_timer_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_LEFT_LV2);
            nrf_gpio_pin_set(OUTPUT_PUMP_LEFT_LV2);
            pump_left_lv2_flag = true;
          }

        } else if ((data[2] == '1') && (data[5] == '3')) {  // Condition #3. Actuate left pump (Lv.3)
          pump_actuation_time = 10*(data[8] - '0') + 1*(data[9] - '0');
          NRF_LOG_INFO("Left pump (Lv.3) actuation time is %d sec.", pump_actuation_time);

          if (pump_actuation_time == 0) {  // If pump actuation time input is 0, turn off the pump.
            pump_all_off();
            NRF_LOG_INFO("Turning off the pump(s)...");
        
          } else if ((pump_actuation_time > 0) && (pump_actuation_time < 100)) {  // Actuation time: 01-99 sec
            electrolysis_timer_sec = 0;
            err_code = app_timer_start(m_app_timer, PUMP_INTERVAL, NULL);
            app_timer_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_LEFT_LV3);
            nrf_gpio_pin_set(OUTPUT_PUMP_LEFT_LV3);
            pump_left_lv3_flag = true;
          }

        } else if ((data[2] == '2') && (data[5] == '1')) {  // Condition #4. Actuate right pump (Lv.1)
          pump_actuation_time = 10*(data[8] - '0') + 1*(data[9] - '0');
          NRF_LOG_INFO("Right pump (Lv.1) actuation time is %d sec.", pump_actuation_time);

          if (pump_actuation_time == 0) {  // If pump actuation time input is 0, turn off the pump.
            pump_all_off();
            NRF_LOG_INFO("Turning off the pump(s)...");
        
          } else if ((pump_actuation_time > 0) && (pump_actuation_time < 100)) {  // Actuation time: 01-99 sec
            electrolysis_timer_sec = 0;
            err_code = app_timer_start(m_app_timer, PUMP_INTERVAL, NULL);
            app_timer_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_RIGHT_LV1);
            nrf_gpio_pin_set(OUTPUT_PUMP_RIGHT_LV1);
            pump_right_lv1_flag = true;
          }

        } else if ((data[2] == '2') && (data[5] == '2')) {  // Condition #5. Actuate right pump (Lv.2)
          pump_actuation_time = 10*(data[8] - '0') + 1*(data[9] - '0');
          NRF_LOG_INFO("Right pump (Lv.2) actuation time is %d sec.", pump_actuation_time);

          if (pump_actuation_time == 0) {  // If pump actuation time input is 0, turn off the pump.
            pump_all_off();
            NRF_LOG_INFO("Turning off the pump(s)...");
        
          } else if ((pump_actuation_time > 0) && (pump_actuation_time < 100)) {  // Actuation time: 01-99 sec
            electrolysis_timer_sec = 0;
            err_code = app_timer_start(m_app_timer, PUMP_INTERVAL, NULL);
            app_timer_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_RIGHT_LV2);
            nrf_gpio_pin_set(OUTPUT_PUMP_RIGHT_LV2);
            pump_right_lv2_flag = true;
          }

        } else if ((data[2] == '2') && (data[5] == '3')) {  // Condition #6. Actuate right pump (Lv.3)
          pump_actuation_time = 10*(data[8] - '0') + 1*(data[9] - '0');
          NRF_LOG_INFO("Right pump (Lv.3) actuation time is %d sec.", pump_actuation_time);

          if (pump_actuation_time == 0) {  // If pump actuation time input is 0, turn off the pump.
            pump_all_off();
            NRF_LOG_INFO("Turning off the pump(s)...");
        
          } else if ((pump_actuation_time > 0) && (pump_actuation_time < 100)) {  // Actuation time: 01-99 sec
            electrolysis_timer_sec = 0;
            err_code = app_timer_start(m_app_timer, PUMP_INTERVAL, NULL);
            app_timer_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_RIGHT_LV3);
            nrf_gpio_pin_set(OUTPUT_PUMP_RIGHT_LV3);
            pump_right_lv3_flag = true;
          }

        } else if ((data[2] == '3') && (data[5] == '1')) {  // Condition #7. Actuate both pumps (Lv.1)
          pump_actuation_time = 10*(data[8] - '0') + 1*(data[9] - '0');
          NRF_LOG_INFO("Both pumps (Lv.1) actuation time is %d sec.", pump_actuation_time);

          if (pump_actuation_time == 0) {  // If pump actuation time input is 0, turn off the pump.
            pump_all_off();
            NRF_LOG_INFO("Turning off the pump(s)...");
        
          } else if ((pump_actuation_time > 0) && (pump_actuation_time < 100)) {  // Actuation time: 01-99 sec
            electrolysis_timer_sec = 0;
            err_code = app_timer_start(m_app_timer, PUMP_INTERVAL, NULL);
            app_timer_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_LEFT_LV1);
            nrf_gpio_pin_set(OUTPUT_PUMP_LEFT_LV1);
            pump_left_lv1_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_RIGHT_LV1);
            nrf_gpio_pin_set(OUTPUT_PUMP_RIGHT_LV1);
            pump_right_lv1_flag = true;
          }

        } else if ((data[2] == '3') && (data[5] == '2')) {  // Condition #8. Actuate both pumps (Lv.2)
          pump_actuation_time = 10*(data[8] - '0') + 1*(data[9] - '0');
          NRF_LOG_INFO("Both pumps (Lv.2) actuation time is %d sec.", pump_actuation_time);

          if (pump_actuation_time == 0) {  // If pump actuation time input is 0, turn off the pump.
            pump_all_off();
            NRF_LOG_INFO("Turning off the pump(s)...");
        
          } else if ((pump_actuation_time > 0) && (pump_actuation_time < 100)) {  // Actuation time: 01-99 sec
            electrolysis_timer_sec = 0;
            err_code = app_timer_start(m_app_timer, PUMP_INTERVAL, NULL);
            app_timer_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_LEFT_LV2);
            nrf_gpio_pin_set(OUTPUT_PUMP_LEFT_LV2);
            pump_left_lv2_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_RIGHT_LV2);
            nrf_gpio_pin_set(OUTPUT_PUMP_RIGHT_LV2);
            pump_right_lv2_flag = true;
          }

        } else if ((data[2] == '3') && (data[5] == '3')) {  // Condition #9. Actuate both pumps (Lv.3)
          pump_actuation_time = 10*(data[8] - '0') + 1*(data[9] - '0');
          NRF_LOG_INFO("Both pumps (Lv.3) actuation time is %d sec.", pump_actuation_time);

          if (pump_actuation_time == 0) {  // If pump actuation time input is 0, turn off the pump.
            pump_all_off();
            NRF_LOG_INFO("Turning off the pump(s)...");
        
          } else if ((pump_actuation_time > 0) && (pump_actuation_time < 100)) {  // Actuation time: 01-99 sec
            electrolysis_timer_sec = 0;
            err_code = app_timer_start(m_app_timer, PUMP_INTERVAL, NULL);
            app_timer_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_LEFT_LV3);
            nrf_gpio_pin_set(OUTPUT_PUMP_LEFT_LV3);
            pump_left_lv3_flag = true;

            nrf_gpio_cfg_output(OUTPUT_PUMP_RIGHT_LV3);
            nrf_gpio_pin_set(OUTPUT_PUMP_RIGHT_LV3);
            pump_right_lv3_flag = true;
          }
        }
      
      } else {  // If pump actuation command is abnormal, turn off the pump.
        pump_all_off();
        NRF_LOG_INFO("Non-identified command. Turning off the pump(s)...");
      }

      ble_cus_custom_value_update(&m_cus, status_ack, sizeof(status_ack));

    } else if (data[0] == 'q') {  // Turn off the LED
      if (ble_pwm1_enable_flag) {
        deinit_ble_pwm1();
      }
      NRF_LOG_INFO("Turning off the LED...");

      ble_cus_custom_value_update(&m_cus, status_ack, sizeof(status_ack));

    } else if (data[0] == 'y') {  // Turn off the pumps
      pump_all_off();
      NRF_LOG_INFO("Turning off the pump(s)...");

      ble_cus_custom_value_update(&m_cus, status_ack, sizeof(status_ack));

    } else if (data[0] == 'r') {  // Reset
      sd_nvic_SystemReset();

    } else if (data[0] == 'z') {  // Restart into bootloader
      err_code = sd_power_gpregret_clr(0, 0xffffffff);
      VERIFY_SUCCESS(err_code);

      err_code = sd_power_gpregret_set(0, 0xB1);
      VERIFY_SUCCESS(err_code);

      // Signal that DFU mode is to be enter to the power management module
      nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_DFU);
    }
    break;

  default:
    break;
  }
}


// services_init()
static void services_init(void) {
  ret_code_t err_code;
  ble_cus_init_t cus_init = {0};

  // Initialize CUS Service init structure to zero.
  cus_init.evt_handler = on_cus_evt;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.cccd_write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cus_init.custom_value_char_attr_md.write_perm);

  err_code = ble_cus_init(&m_cus, &cus_init);
  APP_ERROR_CHECK(err_code);
}


// advertising_init()
static void advertising_init(void) {
  ret_code_t err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = true;
  init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.srdata.uuids_complete.p_uuids = m_adv_uuids;

  init.config.ble_adv_fast_enabled = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


// saadc_event_handler()
void saadc_event_handler(nrf_drv_saadc_evt_t const *p_event) {

}


// saadc_init()
int saadc_init1() {
  // AIN0 =                           P0.02 (EYSHSNZWZ)
  // AIN1 = P0.03 (nRF52840-DK; A0) = P0.03 (EYSHSNZWZ)
  // AIN2 = P0.04 (nRF52840-DK; A1) = P0.04 (EYSHSNZWZ)
  // AIN3 =                           P0.05 (EYSHSNZWZ)
  // AIN4 = P0.28 (nRF52840-DK; A2) = P0.28 (EYSHSNZWZ)
  // AIN5 = P0.29 (nRF52840-DK; A3)
  // AIN6 = P0.30 (nRF52840-DK; A4)
  // AIN7 = P0.31 (nRF52840-DK; A5)
  int ret = nrf_drv_saadc_init(NULL, saadc_event_handler);
  if (ret)
    return ret;
  nrf_saadc_channel_config_t config =
      NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN1); // AIN1 = P0.03 (PIN 9 - TAIYOYUDEN)
  ret = nrf_drv_saadc_channel_init(0, &config);
  return ret;
}


// saadc_measure1()
int saadc_measure1() {
  nrf_saadc_value_t value;
  nrf_drv_saadc_sample_convert(0, &value);
  return value;
}


// timer_handler()
void timer_handler(nrf_timer_event_t event_type, void *p_context) {
  electrolysis_timer_sec = electrolysis_timer_sec + 1;
}


// log_init()
static void log_init(void) {
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}


// timer_init()
static void timer_init(void) {
  ret_code_t err_code;

  /* timer driver initialization */
  nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
}


// power_management_init()
static void power_management_init(void) {
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}


// application_timers_start()
static void application_timers_start(void) {

}


// advertising_start()
static void advertising_start(bool erase_bonds) {
  if (erase_bonds == true) {
    delete_bonds();
    // Advertising is started by PM_EVT_PEERS_DELETED_SUCEEDED event
  } else {
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

    APP_ERROR_CHECK(err_code);
  }
}


// app_timer_handler() 
static void app_timer_handler(void * p_context) {
  electrolysis_timer_sec = electrolysis_timer_sec + 1;

  if (electrolysis_timer_sec == pump_actuation_time) {
    pump_all_off();
  }
}


// app_timers_init()
static void app_timers_init(void) {
  // Initialize timer module, making it use the scheduler
  ret_code_t err_code;

  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);
  
  err_code = app_timer_create(&m_app_timer, APP_TIMER_MODE_REPEATED, app_timer_handler);
  APP_ERROR_CHECK(err_code);
}


// idle_state_handler()
static void idle_state_handle(void) {
  ret_code_t err_code;

  if (NRF_LOG_PROCESS() == false) {
    nrf_pwr_mgmt_run();
  }
}


// main()
int main(void) {
  bool erase_bonds;

  // Initialize.
  log_init(); //
  timer_init();
  buttons_leds_init(&erase_bonds);
  app_timers_init();
  power_management_init();
  ble_stack_init();
  gap_params_init();
  gatt_init();
  services_init();
  advertising_init();
  conn_params_init();
  peer_manager_init();

  // Start execution.
  NRF_LOG_INFO("Template example started.");
  application_timers_start();

  db_discovery_init();
  cus_c_init();

  // advertising_start(erase_bonds);
  // saadc_init0();
  saadc_init1();

  // Start execution.
  NRF_LOG_INFO("LE Secure Connections example started.");

  if (erase_bonds == true) {
    delete_bonds();
    // Scanning and advertising is started by PM_EVT_PEERS_DELETE_SUCEEDED.
  } else {
    adv_scan_start();  // "Advertising"
  }

  // Enter main loop.
  for (;;) {
    idle_state_handle();
  }
}
