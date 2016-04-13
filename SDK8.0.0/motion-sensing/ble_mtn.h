/** @file
 *
 * @brief simple motion sensor service module.
 *
 * @details This module implements the simple motion service with the accelerometer and gyro information characteristic.
 *          During initialization it adds the motion service, accelerometer and gyro characteristic
 *          to the BLE stack database. 
 *
 *          The module will support notification of the accelerometer and gyro characteristic
 *          through the ble_mtn_acc_status_update() and ble_mtn_gyro_status_update() function.
 *
 */

#ifndef BLE_mtn_H__
#define BLE_mtn_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

/**@brief motion service event type. */
typedef enum
{
    BLE_MTN_EVT_NOTIFICATION_ENABLED,                             /**< motion information notification enabled event. */
    BLE_MTN_EVT_NOTIFICATION_DISABLED                             /**< motion information notification disabled event. */
} ble_mtn_evt_type_t;

/**@brief motion service event. */
typedef struct
{
    ble_mtn_evt_type_t evt_type;                                  /**< Type of event. */
} ble_mtn_evt_t;

// Forward declaration of the ble_mtn_t type. 
typedef struct ble_mtn_s ble_mtn_t;

/**@brief motion service event handler type. */
typedef void (*ble_mtn_evt_handler_t) (ble_mtn_t * p_mtn, ble_mtn_evt_t * p_evt);

/**@brief motion service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_mtn_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the motionr service. */
    ble_srv_cccd_security_mode_t  acc_status_char_attr_md;     /**< Initial security level for accelerometer information characteristics attribute */
    ble_srv_cccd_security_mode_t  gyro_status_char_attr_md;     /**< Initial security level for gyro information characteristics attribute */
} ble_mtn_init_t;

/**@brief motion service structure. This contains various status information for the service. */
typedef struct ble_mtn_s
{
    ble_mtn_evt_handler_t         evt_handler;                    /**< Event handler to be called for handling events in the motion service. */
    uint16_t                      service_handle;                 /**< Handle of motion service (as provided by the BLE stack). */
    ble_gatts_char_handles_t      acc_status_handles;          /**< Handles related to the accelerometer information characteristic. */
    ble_gatts_char_handles_t      gyro_status_handles;          /**< Handles related to the gyro information characteristic. */
    uint16_t                      conn_handle;                    /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
} ble_mtn_t;

/**@brief Function for initializing the motion service.
 *
 * @param[out]  p_mtn       motion service structure. This structure will have to be supplied by
 *                          the application. It will be initialized by this function, and will later
 *                          be used to identify this particular service instance.
 * @param[in]   p_mtn_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_mtn_init(ble_mtn_t * p_mtn, const ble_mtn_init_t * p_mtn_init);

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the motion service.
 *
 * @param[in]   p_mtn      motion service structure.
 * @param[in]   p_ble_evt  Event received from the BLE stack.
 */
void ble_mtn_on_ble_evt(ble_mtn_t * p_mtn, ble_evt_t * p_ble_evt);

/**@brief Function for updating the accelerometer information.
 *
 * @details The application calls this function after having performed a acceleration measurement. If
 *          notification has been enabled, the accelerometer information characteristic is sent to the client.
 *
 * @param[in]   p_mtn          motion serivce structure.
 * @param[in]   acc_status		new acceleration inforamtion,
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_mtn_acc_status_update(ble_mtn_t * p_mtn, int16_t* acc_status, uint8_t acc_range);

/**@brief Function for updating the accelerometer information.
 *
 * @details The application calls this function after having performed a angular velocity measurement. If
 *          notification has been enabled, the gyro information characteristic is sent to the client.
 *
 * @param[in]   p_mtn          motion serivce structure.
 * @param[in]   gyro_status 	new angular velocity measurement value 
 *
 * @return      NRF_SUCCESS on success, otherwise an error code.
 */
uint32_t ble_mtn_gyro_status_update(ble_mtn_t * p_mtn, int16_t* gyro_status, uint8_t gyro_range);

#endif // BLE_mtn_H__

/** @} */
