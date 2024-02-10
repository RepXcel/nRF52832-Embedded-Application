#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_WORKOUT_DATA_DEF(_name)                                                                 \
static ble_workout_data_t _name;                                                                    \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HRS_BLE_OBSERVER_PRIO,                                                     \
                     ble_workout_data_on_ble_evt, &_name)                                                                            

#define CUSTOM_SERVICE_UUID_BASE         {0xBC, 0x8A, 0xBF, 0x45, 0xCA, 0x05, 0x50, 0xBA, \
                                          0x40, 0x42, 0xB0, 0x00, 0xC9, 0xAD, 0x64, 0xF3}

#define CUSTOM_SERVICE_UUID               0x1400
#define CUSTOM_VALUE_CHAR_UUID            0x1401

typedef struct ble_workout_data_s ble_workout_data_t;

typedef enum
{
    BLE_WORKOUT_DATA_EVT_NOTIFICATION_ENABLED,                             /**< Custom value notification enabled event. */
    BLE_WORKOUT_DATA_EVT_NOTIFICATION_DISABLED,                            /**< Custom value notification disabled event. */
    BLE_WORKOUT_DATA_EVT_DISCONNECTED,
    BLE_WORKOUT_DATA_EVT_CONNECTED
} ble_workout_data_evt_type_t;

typedef struct
{
    ble_workout_data_evt_type_t evt_type;                                  /**< Type of event. */
} ble_workout_data_evt_t;

typedef void (*ble_workout_data_evt_handler_t) (ble_workout_data_t * p_workout_data, ble_workout_data_evt_t * p_evt);

struct ble_workout_data_s {
    ble_workout_data_evt_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Custom Service. */
    uint16_t                        service_handle;                 /**< Handle of Custom Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t        custom_value_handles;           /**< Handles related to the Custom Value characteristic. */
    uint16_t                        conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint8_t                         uuid_type; 
};

typedef struct {
    ble_workout_data_evt_handler_t         evt_handler;             /**< Event handler to be called for handling events in the Custom Service. */
    float                         initial_custom_value;             /**< Initial custom value */
    ble_srv_cccd_security_mode_t  custom_value_char_attr_md;        /**< Initial security level for Custom characteristics attribute */
} ble_workout_data_init_t;

typedef union{
    float   data;
    uint8_t byte_array[sizeof(float)];
} workout_data_t;

ret_code_t ble_workout_data_init(ble_workout_data_t * p_workout_data, const ble_workout_data_init_t * p_workout_data_init);

void ble_workout_data_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context );

ret_code_t ble_workout_data_custom_value_update(ble_workout_data_t * p_workout_data, workout_data_t custom_value);