/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
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
 * 5. Any software provided in binary form under this license must not be
 * reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */
// Board/nrf6310/ble/ble_app_hrs_rtx/main.c
/**
 *
 * @brief Heart Rate Service Sample Application with RTX main file.
 *
 * This file contains the source code for a sample application using RTX and the
 * Heart Rate service (and also Battery and Device Information services).
 * This application uses the @ref srvlib_conn_params module.
 */

/* clang-format off */
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_error.h"
#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dis.h"
#include "ble_hci.h"
#include "ble_hrs.h"
#include "ble_srv_common.h"
#include "ble_workout_data.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "lis2dh12.h"
#include "math.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_saadc.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "nrf_sdh_soc.h"
#include "nrf_twi_mngr.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

/* clang-format off */
#define DEVICE_NAME                         "RepXcel"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "Garmin"                                /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                    18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(100, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.10 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(300, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.30 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      5000                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       30000                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

#define VELOCITY_QUEUE_LENGTH               1                                       /**< Size of velocity queue */
#define WORKOUT_DATA_NOTIFICATION_INTERVAL  400                                     /**< Notification update interval */

#define TWI_INSTANCE_ID                     0                                       /**< I2C driver instance */
#define MAX_PENDING_TRANSACTIONS            32                                      /**< Maximal number of pending I2C transactions */
#define ACCEl_BUFFER_SIZE                   17                                      /**< Buffer size */
#define ACCEL_PERIOD                        1.0f/200.0f                             /**< Accel sampling period */
#define ACCEL_VALUE_MINIMUM_MMPSS           150.0f                                  /**< Values below threshold will be treated as negligible acceleration*/
#define ACCEL_SAMPLES_TO_READ               5                                       /**< Number of samples to use for velocity when valid sample is detected*/
#define REP_VELOCITY_VALUE_MINIMUM_MMPS     35.0f                                   /**< Minimum velocity for rep tracking*/
#define REP_VELOCITY_SAMPLES_TILL_MOVING    24                                      /**< Number of samples for a valid rep */

#define MG_TO_MMPSS(MG)                     (MG) * 9.81f                            /**< Converts from mg to mm/s^2 (centimeters per second)*/

#define LIS2DH12_INT1                       25                                      /**< nRF52 Pin for LIS2DH12 INT1 */
#define LIS2DH12_INT2                       26                                      /**< nRF52 Pin for LIS2DH12 INT2 */
#define LIS2DH12_SDA                        29                                      /**< nRF52 Pin for LIS2DH12 SDA */
#define LIS2DH12_SCL                        30                                      /**< nRF52 Pin for LIS2DH12 SCL */

#define BUTTON_SLEEPWAKE                    0                                       /**< ID of the button used to sleep/wake the application. */
#define BUTTON_RESET                        1                                       /**< ID of the button used to reset the application. */

#ifdef BOARD_D52DK1
    #define BATTERY_CHARGE_INDICATION       12                                      /**< ID of charge indication LED */
    #define LED_GREEN                       20

#else
    #define BATTERY_CHARGE_INDICATION       4                                       /**< ID of charge indication LED */
    #define LED_GREEN                       11
#endif

/**@brief Macro to convert the result of ADC conversion in millivolts.
 *
 * @param[in]  ADC_VALUE   ADC result.
 *
 * @retval     Result converted to millivolts.
 */
#define ADC_RESULT_IN_MILLI_VOLTS(ADC_VALUE)\
        ((((ADC_VALUE) * ADC_REF_VOLTAGE_IN_MILLIVOLTS) / ADC_RES_10BIT) * ADC_PRE_SCALING_COMPENSATION)

#define MILLIVOLTS_TO_PERCENT(MV)\
        MV >= BATTERY_MAX_VOLTAGE_MILLIVOLTS ? 100 : \
        MV <= BATTERY_MIN_VOLTAGE_MILLIVOLTS ? 0 : \
        (int)(100 - (100 * (float)(BATTERY_MAX_VOLTAGE_MILLIVOLTS - MV) / (float)(BATTERY_MAX_VOLTAGE_MILLIVOLTS - BATTERY_MIN_VOLTAGE_MILLIVOLTS)))

#define ADC_REF_VOLTAGE_IN_MILLIVOLTS   600                                         /**< Reference voltage (in milli volts) used by ADC while doing conversion. */
#define ADC_PRE_SCALING_COMPENSATION    6                                           /**< The ADC is configured to use VDD with 1/3 prescaling as input. And hence the result of conversion is to be multiplied by 3 to get the actual value of the battery voltage.*/
#define ADC_RES_10BIT                   1024                                        /**< Maximum digital value for 10-bit ADC conversion. */
#define BATTERY_MAX_VOLTAGE_MILLIVOLTS  2050                                        /**< VBAT_MEAS Reading for 100% battery */
#define BATTERY_MIN_VOLTAGE_MILLIVOLTS  1650                                        /**< VBAT_MEAS Reading for 0% battery */
#define BATTERY_LEVEL_MEAS_INTERVAL     1000                                        /**< Battery level measurement interval (ms). */

typedef enum                                                                        /**< Device states for rep veloicty state machine */
{
    REST,                                                                           /**< Device not in motion, rep velocity not updated */
    BEGIN_MOVING,                                                                   /**< Device in motion, rep velocity not updated */
    MOVING,                                                                         /**< Device in motion, rep veloicty is updated */
} device_state_t;

BLE_BAS_DEF(m_bas);                                                                 /**< Battery service instance. */
BLE_HRS_DEF(m_hrs);                                                                 /**< Heart rate service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */
NRF_TWI_MNGR_DEF(m_nrf_twi_mngr, MAX_PENDING_TRANSACTIONS, TWI_INSTANCE_ID);        /**< TWI manager instance. */
NRF_TWI_SENSOR_DEF(m_nrf_twi_sensor, &m_nrf_twi_mngr, ACCEl_BUFFER_SIZE);           /**< TWI sensor instance. */
LIS2DH12_INSTANCE_DEF(m_lis2dh12, &m_nrf_twi_sensor, LIS2DH12_BASE_ADDRESS_HIGH);   /**< LIS2DH12 accel instance. */
BLE_WORKOUT_DATA_DEF(m_workout_data);

static uint16_t         m_conn_handle = BLE_CONN_HANDLE_INVALID;                    /**< Handle of the current connection. */
static QueueHandle_t    m_velocity_queue = NULL; 

static workout_data_t   m_rep_velocity_mmps;                                        /**< Device state for rep velocity tracking*/  
static nrf_saadc_value_t adc_buf[2];                                                /**< SAADC buffer */

static ble_uuid_t m_adv_uuids[] = {                                                 /**< Universally unique service identifiers. */
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
};

static ble_uuid_t m_sr_uuids[] = {
    {CUSTOM_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}                               /**< Universally unique service identifiers. */
};

static nrf_ppi_channel_t m_ppi_channel;

static TimerHandle_t m_battery_timer;                                               /**< Definition of battery timer. */
static TimerHandle_t m_ble_workout_data_notif_timer;                                /**< Definition of notification timer. */

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                                /**< Definition of Logger thread. */
#endif
static TaskHandle_t m_accel_thread;                                                 /**< Definition of Accel thread. */
static TaskHandle_t m_rep_velocity_thread;                                          /**< Definition of Per Rep Velocity thread. */
/* clang-format on */

/**
 * @brief Idle hook for FreeRTOS to put device in low power mode
 */
void vApplicationIdleHook(void) { nrf_pwr_mgmt_run(); }

static void advertising_start(void* p_erase_bonds);

/**
 * @brief Configures the accelerometer on.
 * Accel reads at 200HZ, 2G sensitivity, 12-bit resolution.
 */
static void accel_on(void) {
    ret_code_t err_code;
    LIS2DH12_DATA_CFG(m_lis2dh12, LIS2DH12_ODR_200HZ, false, true, true, true, LIS2DH12_SCALE_2G, true);
    LIS2DH12_FIFO_CFG(m_lis2dh12, true, LIS2DH12_STREAM, false, ACCEl_BUFFER_SIZE);
    LIS2DH12_INT1_PIN_CFG(m_lis2dh12, false, false, false, false, true, false, true, false);
    LIS2DH12_FILTER_CFG(m_lis2dh12, LIS2DH12_FILTER_MODE_NORMAL, LIS2DH12_FILTER_FREQ_1, true, false, false, false);
    err_code = lis2dh12_cfg_commit(&m_lis2dh12);

    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Configures the accelerometer off.
 */
static void accel_off(void) {
    ret_code_t err_code;
    LIS2DH12_DATA_CFG(m_lis2dh12, LIS2DH12_ODR_POWERDOWN, false, true, true, true, LIS2DH12_SCALE_2G, true);
    err_code = lis2dh12_cfg_commit(&m_lis2dh12);

    APP_ERROR_CHECK(err_code);
}

/**@brief Use current data in the accel buffer to update the device velocity.
 */
static void update_velocity(lis2dh12_data_t* p_accel_buffer) {
    static uint8_t m_samples_to_read[3];
    static float velocity_xyz_mmps[3];
    float accel_magnitude_mmpss[3];
    float velocity_magnitude_mmps;

    for (uint8_t i = 0; i < ACCEl_BUFFER_SIZE; i++) {
        accel_magnitude_mmpss[0] = MG_TO_MMPSS(p_accel_buffer[i].x >> 4);
        accel_magnitude_mmpss[1] = MG_TO_MMPSS(p_accel_buffer[i].y >> 4);
        accel_magnitude_mmpss[2] = MG_TO_MMPSS(p_accel_buffer[i].z >> 4);
        for (uint8_t i = 0; i < sizeof(accel_magnitude_mmpss) / sizeof(float); i++) {
            if (abs(accel_magnitude_mmpss[i]) > ACCEL_VALUE_MINIMUM_MMPSS) {
                m_samples_to_read[i] = ACCEL_SAMPLES_TO_READ;
            }
            if (m_samples_to_read[i]) {
                velocity_xyz_mmps[i] += accel_magnitude_mmpss[i] * ACCEL_PERIOD;
            } else {
                velocity_xyz_mmps[i] = 0;
            }
            m_samples_to_read[i] = MAX(m_samples_to_read[i] - 1, 0);
        }
        velocity_magnitude_mmps =
            sqrt(velocity_xyz_mmps[0] * velocity_xyz_mmps[0] + velocity_xyz_mmps[1] * velocity_xyz_mmps[1] +
                 velocity_xyz_mmps[2] * velocity_xyz_mmps[2]);
        xQueueSend(m_velocity_queue, (void*)&velocity_magnitude_mmps, 0);
        NRF_LOG_INFO("Velocity: " NRF_LOG_FLOAT_MARKER, NRF_LOG_FLOAT(velocity_magnitude_mmps));
    }
}

/**
 * @brief Accel thread to read and process accel data.
 *
 * @param[in]   arg   Unused
 */
static void accel_thread(void* arg) {
    UNUSED_PARAMETER(arg);
    ret_code_t err_code;
    lis2dh12_data_t accel_buffer[ACCEl_BUFFER_SIZE] = {0};

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        err_code = lis2dh12_data_read(&m_lis2dh12, NULL, accel_buffer, ACCEl_BUFFER_SIZE);
        APP_ERROR_CHECK(err_code);
        update_velocity(accel_buffer);
    }
}

/**
 * @brief Rep velocity thread to count rep velocity from global velocity
 * measurements.
 *
 * @param[in]   arg   Unused
 */
static void rep_velocity_thread(void* arg) {
    UNUSED_PARAMETER(arg);
    ret_code_t err_code;
    device_state_t m_device_state = REST;
    uint32_t sample_count = 0;
    float rep_velocity_sum_mmps = 0;
    float velocity_mmps;

    for (;;) {
        xQueueReceive(m_velocity_queue, &velocity_mmps, portMAX_DELAY);
        switch (m_device_state) {
        case REST:
            if (velocity_mmps >= REP_VELOCITY_VALUE_MINIMUM_MMPS) {
                rep_velocity_sum_mmps = 0;
                sample_count = 0;
                m_device_state = BEGIN_MOVING;
            }
            break;

        case BEGIN_MOVING:
            rep_velocity_sum_mmps += velocity_mmps;
            sample_count++;
            if (velocity_mmps < REP_VELOCITY_VALUE_MINIMUM_MMPS) {
                m_device_state = REST;
            } else if (sample_count == REP_VELOCITY_SAMPLES_TILL_MOVING) {
                m_device_state = MOVING;
            }
            break;

        case MOVING:
            if (velocity_mmps >= REP_VELOCITY_VALUE_MINIMUM_MMPS) {
                rep_velocity_sum_mmps += velocity_mmps;
                sample_count++;
            } else if (velocity_mmps == 0) {
                m_rep_velocity_mmps.data.velocity = rep_velocity_sum_mmps / sample_count;
                m_rep_velocity_mmps.data.timestamp += 1;
                m_device_state = REST;
            }
            break;

        default:
            break;
        }
        NRF_LOG_INFO("Rep state %d, Timestamp: %d, Rep Velocity: " NRF_LOG_FLOAT_MARKER, m_device_state,
                     m_rep_velocity_mmps.data.timestamp, NRF_LOG_FLOAT(m_rep_velocity_mmps.data.velocity));
    }
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product.
 * You need to analyze how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t* p_file_name) {
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const* p_evt) {
    bool delete_bonds = false;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_disconnect_on_sec_failure(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id) {
    case PM_EVT_PEERS_DELETE_SUCCEEDED:
        advertising_start(&delete_bonds);
        break;

    default:
        break;
    }
}

/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC,
 * convert the value into percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const* p_event) {
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE) {
        nrf_saadc_value_t adc_result;
        uint16_t batt_lvl_in_milli_volts;
        uint8_t percentage_batt_lvl;
        uint32_t err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result);
        percentage_batt_lvl = MILLIVOLTS_TO_PERCENT(batt_lvl_in_milli_volts);

        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl, BLE_CONN_HANDLE_ALL);
        if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_INVALID_STATE) && (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_BUSY) && (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)) {
            APP_ERROR_HANDLER(err_code);
        }
        NRF_LOG_INFO("Millivolts: %d | Battery percent: %d", batt_lvl_in_milli_volts, percentage_batt_lvl);
    }
}

/**@brief Function for configuring ADC to do battery level conversion.
 */
static void adc_configure(void) {
    ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN3);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement
 * timer expires. This function will start the ADC.
 *
 * @param[in] p_context   Pointer used for passing some arbitrary information
 * (context) from the app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void* p_context) {
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code;
    err_code = nrf_drv_saadc_sample();
    APP_ERROR_CHECK(err_code);
}

static void workout_data_notification_timeout_handler(void* p_context) {
    UNUSED_PARAMETER(p_context);
    ret_code_t err_code;
    err_code = ble_workout_data_value_update(&m_workout_data, m_rep_velocity_mmps);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application
 * timers.
 */
static void timers_init(void) {
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    m_ble_workout_data_notif_timer = xTimerCreate("WKT", WORKOUT_DATA_NOTIFICATION_INTERVAL, pdTRUE, NULL,
                                                  workout_data_notification_timeout_handler);
    m_battery_timer =
        xTimerCreate("BATT", BATTERY_LEVEL_MEAS_INTERVAL, pdTRUE, NULL, battery_level_meas_timeout_handler);
}

/**@brief Initialzes FreeRTOS queues
 */
static void queues_init(void) { m_velocity_queue = xQueueCreate(VELOCITY_QUEUE_LENGTH, sizeof(float)); }

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile)
 * parameters of the device including the device name, appearance, and the
 * preferred connection parameters.
 */
static void gap_params_init(void) {
    ret_code_t err_code;
    ble_gap_conn_params_t gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t*)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_COMPUTER);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the GATT module. */
static void gatt_init(void) {
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may
 * need to inform the application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went
 * wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error) { APP_ERROR_HANDLER(nrf_error); }

static void on_workout_data_evt(ble_workout_data_t* p_workout_data, ble_workout_data_evt_t* p_evt) {
    ret_code_t err_code;

    switch (p_evt->evt_type) {
    case BLE_WORKOUT_DATA_EVT_NOTIFICATION_ENABLED:
        NRF_LOG_INFO("WORKOUT EVENT: NOTIF ENABLED");
        if (pdPASS != xTimerStart(m_ble_workout_data_notif_timer, OSTIMER_WAIT_FOR_QUEUE)) {
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }

        err_code = bsp_indication_set(BSP_INDICATE_IDLE) || bsp_indication_set(BSP_INDICATE_USER_STATE_1) ||
                   bsp_event_to_button_action_assign(BUTTON_SLEEPWAKE, BSP_BUTTON_ACTION_LONG_PUSH, BSP_EVENT_DEFAULT);
        APP_ERROR_CHECK(err_code);
        accel_on();
        memset(&m_rep_velocity_mmps, 0, sizeof(workout_data_t));
        break;

    case BLE_WORKOUT_DATA_EVT_NOTIFICATION_DISABLED:
        NRF_LOG_INFO("WORKOUT EVENT: NOTIF DISABLED");
        if (pdPASS != xTimerStop(m_ble_workout_data_notif_timer, OSTIMER_WAIT_FOR_QUEUE)) {
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }

        err_code = bsp_indication_set(BSP_INDICATE_IDLE) || bsp_indication_set(BSP_INDICATE_CONNECTED) ||
                   bsp_event_to_button_action_assign(BUTTON_SLEEPWAKE, BSP_BUTTON_ACTION_LONG_PUSH, BSP_EVENT_SLEEP);
        APP_ERROR_CHECK(err_code);
        accel_off();
        break;

    case BLE_WORKOUT_DATA_EVT_CONNECTED:
        NRF_LOG_INFO("WORKOUT EVENT: CONNECTED");
        break;

    case BLE_WORKOUT_DATA_EVT_DISCONNECTED:
        NRF_LOG_INFO("WORKOUT EVENT: DISCONNECTED");
        if (pdPASS != xTimerStop(m_ble_workout_data_notif_timer, OSTIMER_WAIT_FOR_QUEUE)) {
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
        accel_off();
        break;

    default:
        // No implementation needed.
        break;
    }
}

void on_bas_evt(ble_bas_t* p_bas, ble_bas_evt_t* p_evt) {
    switch (p_evt->evt_type) {
    case BLE_BAS_EVT_NOTIFICATION_ENABLED:
        NRF_LOG_INFO("BAS EVENT: NOTIF ENABLED");
        if (pdPASS != xTimerStart(m_battery_timer, OSTIMER_WAIT_FOR_QUEUE)) {
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
        break;

    case BLE_BAS_EVT_NOTIFICATION_DISABLED:
        NRF_LOG_INFO("BAS EVENT: NOTIF DISABLED");
        if (pdPASS != xTimerStop(m_battery_timer, OSTIMER_WAIT_FOR_QUEUE)) {
            APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
        }
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for initializing services that will be used by the
 * application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void) {
    ret_code_t err_code;
    ble_bas_init_t bas_init = {0};
    ble_dis_init_t dis_init = {0};
    nrf_ble_qwr_init_t qwr_init = {0};
    ble_workout_data_init_t workout_data_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec = SEC_OPEN;
    bas_init.bl_cccd_wr_sec = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler = on_bas_evt;
    bas_init.support_notification = true;
    bas_init.p_report_ref = NULL;
    bas_init.initial_batt_level = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char*)MANUFACTURER_NAME);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // Initialize workout data Service init structure to zero.
    memset(&workout_data_init, 0, sizeof(workout_data_init));

    workout_data_init.evt_handler = on_workout_data_evt;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&workout_data_init.custom_value_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&workout_data_init.custom_value_char_attr_md.write_perm);

    err_code = ble_workout_data_init(&m_workout_data, &workout_data_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection
 * Parameters Module which are passed to the application.
 * @note All this function does is to disconnect. This could have been
 * done by simply setting the disconnect_on_fail config parameter, but instead
 * we use the event handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t* p_evt) {
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went
 * wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) { APP_ERROR_HANDLER(nrf_error); }

/**@brief Function for initializing the Connection Parameters module. */
static void conn_params_init(void) {
    ret_code_t err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail = false;
    cp_init.evt_handler = on_conn_params_evt;
    cp_init.error_handler = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for disabling advertising and indcating sleep
 */
static void sleep_mode_enter(void) {
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    sd_ble_gap_adv_stop(m_advertising.adv_handle);
    err_code = bsp_event_to_button_action_assign(BUTTON_SLEEPWAKE, BSP_BUTTON_ACTION_RELEASE, BSP_EVENT_WAKEUP);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for enabling and indicating advertising
 */
static void sleep_mode_exit(void) {
    ret_code_t err_code;

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(BUTTON_SLEEPWAKE, BSP_BUTTON_ACTION_RELEASE, BSP_EVENT_SLEEP);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed
 * to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
    uint32_t err_code;

    switch (ble_adv_evt) {
    case BLE_ADV_EVT_FAST:
        NRF_LOG_INFO("Fast advertising.");
        err_code = bsp_indication_set(BSP_INDICATE_IDLE) || bsp_indication_set(BSP_INDICATE_ADVERTISING);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_ADV_EVT_IDLE:
        sleep_mode_enter();
        break;

    default:
        break;
    }
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const* p_ble_evt, void* p_context) {
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
        NRF_LOG_INFO("Connected");
        err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        APP_ERROR_CHECK(err_code);
        m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
        err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected");
        m_conn_handle = BLE_CONN_HANDLE_INVALID;
        break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
        NRF_LOG_DEBUG("PHY update request.");
        ble_gap_phys_t const phys = {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    } break;

    case BLE_GATTC_EVT_TIMEOUT:
        // Disconnect on GATT Client timeout event.
        NRF_LOG_DEBUG("GATT Client Timeout.");
        err_code =
            sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GATTS_EVT_TIMEOUT:
        // Disconnect on GATT Server timeout event.
        NRF_LOG_DEBUG("GATT Server Timeout.");
        err_code =
            sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        break;

    default:
        // No implementation needed.
        break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
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

/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event) {
    ret_code_t err_code;

    switch (event) {
    case BSP_EVENT_SLEEP:
        NRF_LOG_INFO("BSP SLEEP");
        sleep_mode_enter();
        break;

    case BSP_EVENT_WAKEUP:
        NRF_LOG_INFO("BSP WAKEUP");
        sleep_mode_exit();
        break;

    case BSP_EVENT_DISCONNECT:
        NRF_LOG_INFO("BSP DISCONNECT");
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_ERROR_INVALID_STATE) {
            APP_ERROR_CHECK(err_code);
        }
        break;

    case BSP_EVENT_RESET:
        NRF_LOG_INFO("BSP RESET");
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET);
        break;

    default:
        break;
    }
}

/**@brief Function for the Peer Manager initialization. */
static void peer_manager_init(void) {
    ble_gap_sec_params_t sec_param;
    ret_code_t err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond = SEC_PARAM_BOND;
    sec_param.mitm = SEC_PARAM_MITM;
    sec_param.lesc = SEC_PARAM_LESC;
    sec_param.keypress = SEC_PARAM_KEYPRESS;
    sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob = SEC_PARAM_OOB;
    sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc = true;
    sec_param.kdist_own.id = true;
    sec_param.kdist_peer.enc = true;
    sec_param.kdist_peer.id = true;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Clear bond information from persistent storage. */
static void delete_bonds(void) {
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality. */
static void advertising_init(void) {
    ret_code_t err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type = BLE_ADV_DATA_MATCH_FULL_NAME;
    init.advdata.include_appearance = true;
    init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids = m_adv_uuids;

    init.srdata.uuids_complete.uuid_cnt = sizeof(m_sr_uuids) / sizeof(m_sr_uuids[0]);
    init.srdata.uuids_complete.p_uuids = m_sr_uuids;

    init.config.ble_adv_fast_enabled = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was
 * pressed to wake the application up.
 */
static void buttons_leds_init(bool* p_erase_bonds) {
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_event_to_button_action_assign(BUTTON_RESET, BSP_BUTTON_ACTION_PUSH, BSP_EVENT_RESET);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

/**@brief Function for starting advertising. */
static void advertising_start(void* p_erase_bonds) {
    bool erase_bonds = *(bool*)p_erase_bonds;

    if (erase_bonds) {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    } else {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}

#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are
 * deferred. Thread flushes all log entries and suspends. It is resumed by idle
 * task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information
 * (context) from the osThreadCreate() call to the thread.
 */
static void logger_thread(void* arg) {
    UNUSED_PARAMETER(arg);

    while (1) {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif // NRF_LOG_ENABLED

#if NRF_LOG_ENABLED && NRF_LOG_DEFERRED
void log_pending_hook(void) {
    BaseType_t YieldRequired = pdFAIL;
    if (__get_IPSR() != 0) {
        YieldRequired = xTaskResumeFromISR(m_logger_thread);
        portYIELD_FROM_ISR(YieldRequired);
    } else {
        UNUSED_RETURN_VALUE(vTaskResume(m_logger_thread));
    }
}
#endif

/**@brief Function for initializing the clock.
 */
static void clock_init(void) {
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Initialize LIS2DH12 instance
 */
static void accel_init(void) {
    ret_code_t err_code;

    nrf_drv_twi_config_t const config = {.scl = LIS2DH12_SCL,
                                         .sda = LIS2DH12_SDA,
                                         .frequency = NRF_DRV_TWI_FREQ_100K,
                                         .interrupt_priority = APP_IRQ_PRIORITY_LOWEST,
                                         .clear_bus_init = true};

    err_code = nrf_twi_mngr_init(&m_nrf_twi_mngr, &config);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_twi_sensor_init(&m_nrf_twi_sensor);
    APP_ERROR_CHECK(err_code);
    err_code = lis2dh12_init(&m_lis2dh12);
    APP_ERROR_CHECK(err_code);
    accel_off();
}

static void accel_interrupt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(m_accel_thread, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static void charge_inerrupt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
    NRF_LOG_INFO("Charging status changed!");
}

static void gpiote_init(void) {
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_config_t accel_interrupt_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    err_code = nrf_drv_gpiote_in_init(LIS2DH12_INT1, &accel_interrupt_config, accel_interrupt_handler);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_in_event_enable(LIS2DH12_INT1, true);

    nrf_drv_gpiote_in_config_t battery_interrupt_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    err_code = nrf_drv_gpiote_in_init(BATTERY_CHARGE_INDICATION, &battery_interrupt_config, charge_inerrupt_handler);
    nrf_drv_gpiote_in_event_enable(BATTERY_CHARGE_INDICATION, true);

    nrf_drv_gpiote_out_config_t battery_charge_indicator_config =
        GPIOTE_CONFIG_OUT_TASK_TOGGLE(!nrf_gpio_pin_read(BATTERY_CHARGE_INDICATION));
    err_code = nrf_drv_gpiote_out_init(LED_GREEN, &battery_charge_indicator_config);
    nrf_drv_gpiote_out_task_enable(LED_GREEN);
}

static void ppi_init(void) {
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel, nrf_drv_gpiote_in_event_addr_get(BATTERY_CHARGE_INDICATION),
                                          nrf_drv_gpiote_out_task_addr_get(LED_GREEN));
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for application main entry.
 */
int main(void) {
    bool erase_bonds;

    // Initialize modules.
    log_init();
    clock_init();

    // Do not start any interrupt that uses system functions before system
    // initialisation. The best solution is to start the OS before any other
    // initalisation.

    if (
#if NRF_LOG_ENABLED
        // Start execution.
        pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread) ||
#endif
        pdPASS != xTaskCreate(accel_thread, "accel", 256, NULL, 1, &m_accel_thread) ||
        pdPASS != xTaskCreate(rep_velocity_thread, "rep_velocity", 256, NULL, 2, &m_rep_velocity_thread)) {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Configure and initialize the BLE stack.
    ble_stack_init();

    // Initialize modules.
    adc_configure();
    gpiote_init();
    ppi_init();
    timers_init();
    queues_init();
    buttons_leds_init(&erase_bonds);
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    conn_params_init();
    peer_manager_init();
    accel_init();

    // Create a FreeRTOS task for the BLE stack.
    // The task will run advertising_start() before entering its loop.
    nrf_sdh_freertos_init(advertising_start, &erase_bonds);

    NRF_LOG_INFO("RepXcel Started.");
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;) {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}
