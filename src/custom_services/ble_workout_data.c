#include "sdk_common.h"
#include "ble_srv_common.h"
#include "ble_workout_data.h"
#include <string.h>
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_log.h"

static void reverse_endianess(workout_data_t * p_data){
    uint32_t num_bytes = sizeof(workout_data_t);
    for(uint8_t i = 0; i < num_bytes/2; i++){
        uint8_t temp = p_data->byte_array[i];
        p_data->byte_array[i] = p_data->byte_array[num_bytes-1-i];
        p_data->byte_array[num_bytes-1-i] = temp;
    }
}

static void on_connect(ble_workout_data_t * p_workout_data, ble_evt_t const * p_ble_evt)
{
    p_workout_data->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    if(p_workout_data->evt_handler != NULL){
        ble_workout_data_evt_t evt  = { BLE_WORKOUT_DATA_EVT_CONNECTED };
        p_workout_data->evt_handler(p_workout_data, &evt);
    }
}

static void on_disconnect(ble_workout_data_t * p_workout_data, ble_evt_t const * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_workout_data->conn_handle = BLE_CONN_HANDLE_INVALID;

    if(p_workout_data->evt_handler != NULL){
        ble_workout_data_evt_t evt  = { BLE_WORKOUT_DATA_EVT_DISCONNECTED };
        p_workout_data->evt_handler(p_workout_data, &evt);
    }
}

static void on_write(ble_workout_data_t * p_workout_data, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
    
    // Check if the handle passed with the event matches the Custom Value Characteristic handle.
    if (p_evt_write->handle == p_workout_data->custom_value_handles.value_handle)
    {
        
    }

    if ((p_evt_write->handle == p_workout_data->custom_value_handles.cccd_handle) && (p_evt_write->len == 2))
    {
        // CCCD written, call application event handler
        if (p_workout_data->evt_handler != NULL)
        {
            ble_workout_data_evt_t evt = {0};
            evt.evt_type = ble_srv_is_notification_enabled(p_evt_write->data) ? BLE_WORKOUT_DATA_EVT_NOTIFICATION_ENABLED : BLE_WORKOUT_DATA_EVT_NOTIFICATION_DISABLED;
            // Call the application event handler.
            p_workout_data->evt_handler(p_workout_data, &evt);
        }

    }
}

static ret_code_t custom_value_char_add(ble_workout_data_t * p_workout_data, const ble_workout_data_init_t * p_workout_data_init)
{
    ret_code_t err_code;
    ble_gatts_char_md_t char_md         = {0};
    ble_gatts_attr_md_t cccd_md         = {0};
    ble_gatts_attr_t attr_char_value    = {0};
    ble_uuid_t ble_uuid                 = {0};
    ble_gatts_attr_md_t attr_md         = {0};

    //  Read  operation on Cccd should be possible without authentication.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc       = BLE_GATTS_VLOC_STACK;

    char_md.char_props.read   = true;
    char_md.char_props.write  = true;
    char_md.char_props.notify = true; 
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md; 
    char_md.p_sccd_md         = NULL;

    attr_md.read_perm  = p_workout_data_init->custom_value_char_attr_md.read_perm;
    attr_md.write_perm = p_workout_data_init->custom_value_char_attr_md.write_perm;
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = false;
    attr_md.wr_auth    = false;
    attr_md.vlen       = false;

    ble_uuid.type = p_workout_data->uuid_type;
    ble_uuid.uuid = CUSTOM_VALUE_CHAR_UUID;

    attr_char_value.p_uuid    = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len  = sizeof(workout_data_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len   = sizeof(workout_data_t);

    err_code = sd_ble_gatts_characteristic_add(p_workout_data->service_handle, &char_md,
                                               &attr_char_value,
                                               &p_workout_data->custom_value_handles);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

ret_code_t ble_workout_data_init(ble_workout_data_t * p_workout_data, const ble_workout_data_init_t * p_workout_data_init)
{
    if (p_workout_data == NULL || p_workout_data_init == NULL)
    {
        return NRF_ERROR_NULL;
    }

    ret_code_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure
    p_workout_data->evt_handler               = p_workout_data_init->evt_handler;
    p_workout_data->conn_handle               = BLE_CONN_HANDLE_INVALID;

    // Add Custom Service UUID
    ble_uuid128_t base_uuid = {CUSTOM_SERVICE_UUID_BASE};
    err_code =  sd_ble_uuid_vs_add(&base_uuid, &p_workout_data->uuid_type);
    VERIFY_SUCCESS(err_code);
    
    ble_uuid.type = p_workout_data->uuid_type;
    ble_uuid.uuid = CUSTOM_SERVICE_UUID;

    // Add the Custom Service
    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_workout_data->service_handle);
    VERIFY_SUCCESS(err_code);

    err_code = custom_value_char_add(p_workout_data, p_workout_data_init);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

ret_code_t ble_workout_data_custom_value_update(ble_workout_data_t * p_workout_data, workout_data_t custom_value)
{
    if (p_workout_data == NULL)
    {
        return NRF_ERROR_NULL;
    }

    ret_code_t err_code = NRF_SUCCESS;
    ble_gatts_value_t gatts_value = {0};
    reverse_endianess(&custom_value);
    gatts_value.len     = sizeof(workout_data_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = custom_value.byte_array;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_workout_data->conn_handle,
                                      p_workout_data->custom_value_handles.value_handle,
                                      &gatts_value);
    VERIFY_SUCCESS(err_code);

    // Send value if connected and notifying.
    if (p_workout_data->conn_handle != BLE_CONN_HANDLE_INVALID) 
    {
        ble_gatts_hvx_params_t hvx_params = {0};

        hvx_params.handle = p_workout_data->custom_value_handles.value_handle;
        hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
        hvx_params.offset = gatts_value.offset;
        hvx_params.p_len  = &gatts_value.len;
        hvx_params.p_data = gatts_value.p_value;

        err_code = sd_ble_gatts_hvx(p_workout_data->conn_handle, &hvx_params);
    }

    return err_code;
}

void ble_workout_data_on_ble_evt( ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_workout_data_t * p_workout_data = (ble_workout_data_t *) p_context;
    
    if (p_workout_data == NULL || p_ble_evt == NULL)
    {
        return;
    }
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_workout_data, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_workout_data, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_workout_data, p_ble_evt);
            break;

        default:
            
            break;
    }
}