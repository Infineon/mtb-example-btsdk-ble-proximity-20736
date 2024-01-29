/*
 * Copyright 2021-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
*
* LE Proximity profile, service, application
*
* These are updates to the ROM code for LE Proximity device.
*
* Refer to Bluetooth SIG Proximity Profile 1.0 specifications for details.
* A proximity client application is needed to test full functionality of
* this application.
*
* Features demonstrated
*  - Proximity implementation
*  - Implement link loss service, immediate alert service,
*    TX Power service, and battery service
*
* To demonstrate the app, work through the following steps.
* 1. Plug two WICED eval boards into your computer.
* 2. Build and download this application onto the first WICED board and then download
*    proximity client app to the second WICED board.
* 3. Proximity device starts advertisements after the download.
  4. Push and hold the application button on the client board for 6 seconds to
     start the connection process.
* 5. After connection is established quickly push and release the application
*    button on the proximity client board to send Alert notification to the proximity device.
* 6. Push and release the application button on the proximity client board to stop Alert.
* 7. On the same board push and hold button for 6 seconds, to disconnect and repeat connection.
*
*/

#include "spar_utils.h"
#include "bleprofile.h"
#include "bleapp.h"
#include "bleprox.h"
#include "gpiodriver.h"
#include "string.h"
#include "stdio.h"
#include "platform.h"
#include "sparcommon.h"



//////////////////////////////////////////////////////////////////////////////
//                      global variables
//////////////////////////////////////////////////////////////////////////////

PLACE_IN_DROM const UINT8 proximity_db_data[]=
{
    // GATT service
    PRIMARY_SERVICE_UUID16 (0x0001, UUID_SERVICE_GATT),

    CHARACTERISTIC_UUID16  (0x0002, 0x0003, UUID_CHARACTERISTIC_SERVICE_CHANGED, LEGATTDB_CHAR_PROP_INDICATE, LEGATTDB_PERM_NONE, 4),
        0x00, 0x00, 0x00, 0x00,

    // GAP service
    PRIMARY_SERVICE_UUID16 (0x0014, UUID_SERVICE_GAP),

    CHARACTERISTIC_UUID16 (0x0015, 0x0016, UUID_CHARACTERISTIC_DEVICE_NAME, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 15),
        'L','E',' ','P','r','o','x',' ','k','e','y',' ','f','o','b',

    CHARACTERISTIC_UUID16 (0x0017, 0x0018, UUID_CHARACTERISTIC_APPEARANCE, LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE, 2),
        0x00,0x00,

    // Link Loss service
    PRIMARY_SERVICE_UUID16 (0x0028, UUID_SERVICE_LINK_LOSS),

    CHARACTERISTIC_UUID16_WRITABLE (0x0029, 0x002a, UUID_CHARACTERISTIC_ALERT_LEVEL,
                                     LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_WRITE,
                                     LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_REQ,  1),
        0x01,

    // Immediate alert service
    PRIMARY_SERVICE_UUID16 (0x002B, UUID_SERVICE_IMMEDIATE_ALERT),

    CHARACTERISTIC_UUID16_WRITABLE (0x002c, 0x002d, UUID_CHARACTERISTIC_ALERT_LEVEL,
                                     LEGATTDB_CHAR_PROP_WRITE_NO_RESPONSE, LEGATTDB_PERM_WRITE_CMD,  1),
        0x00,

    // Tx Power service
    PRIMARY_SERVICE_UUID16 (0x002e, UUID_SERVICE_TX_POWER),

    CHARACTERISTIC_UUID16 (0x002f, 0x0030, UUID_CHARACTERISTIC_TX_POWER_LEVEL,
                           LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE,  1),
        0x04,                       // this should be matched to ADV data

    // Battery service
    PRIMARY_SERVICE_UUID16 (0x0031, UUID_SERVICE_BATTERY),

    CHARACTERISTIC_UUID16 (0x0032, 0x0033, UUID_CHARACTERISTIC_BATTERY_LEVEL,
                           LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_READABLE,  1),
        0x64,

    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0034, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                     LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,

    CHARACTERISTIC_UUID16 (0x041, 0x0042, UUID_CHARACTERISTIC_BATTERY_POWER_STATE,
                           LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_READABLE, 1),
        BLEBAT_POWERSTATE_PRESENT_PRESENT|
        BLEBAT_POWERSTATE_DISCHARGING_NOTSUPPORTED|
        BLEBAT_POWERSTATE_CHARGING_NOTSUPPORTED|
        BLEBAT_POWERSTATE_LEVEL_GOODLEVEL,

    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0043, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                     LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,

    CHARACTERISTIC_UUID16 (0x0044, 0x0045, UUID_CHARACTERISTIC_SERVICE_REQUIRED,
                           LEGATTDB_CHAR_PROP_READ | LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_READABLE, 1),
        BLEBAT_SERVICEREQUIRED_NOSERVICEREQUIRED,

    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x0046, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                     LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD | LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,

    CHARACTERISTIC_UUID16 (0x0047, 0x0048, UUID_CHARACTERISTIC_REMOVABLE,
                           LEGATTDB_CHAR_PROP_READ, LEGATTDB_PERM_READABLE,  1),
        BLEBAT_REMOVABLE_UNKNOWN,

    CHARACTERISTIC_UUID16 (0x004a, 0x004b, UUID_CHARACTERISTIC_BATTERY_LEVEL_STATE,
                           LEGATTDB_CHAR_PROP_BROADCAST | LEGATTDB_CHAR_PROP_NOTIFY, LEGATTDB_PERM_NONE, 5),
        0x64,           // Level
        BLEBAT_POWERSTATE_PRESENT_PRESENT|
        BLEBAT_POWERSTATE_DISCHARGING_NOTSUPPORTED|
        BLEBAT_POWERSTATE_CHARGING_NOTSUPPORTED|
        BLEBAT_POWERSTATE_LEVEL_GOODLEVEL,
        0x00,           // Namespace
        0x00, 0x00,     // Description

    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x004C, UUID_DESCRIPTOR_CLIENT_CHARACTERISTIC_CONFIGURATION,
                                     LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,

    CHAR_DESCRIPTOR_UUID16_WRITABLE (0x004D, UUID_DESCRIPTOR_SERVER_CHARACTERISTIC_CONFIGURATION,
                                     LEGATTDB_PERM_READABLE | LEGATTDB_PERM_WRITE_CMD |LEGATTDB_PERM_WRITE_REQ, 2),
        0x00,0x00,
};

///////////////////////////////////////////////////////////////////////////////////////////////////
// Function definitions
///////////////////////////////////////////////////////////////////////////////////////////////////
// Following structure defines UART configuration
const BLE_PROFILE_PUART_CFG bleprox_puart_cfg =
{
    /*.baudrate   =*/ 115200,
    /*.txpin      =*/ PUARTENABLE | GPIO_PIN_UART_TX,
    /*.rxpin      =*/ PUARTENABLE | GPIO_PIN_UART_RX,
};

// Following structure defines GPIO configuration used by the application
const BLE_PROFILE_GPIO_CFG bleprox_gpio_cfg =
{
    /*.gpio_pin =*/
    {
    GPIO_PIN_WP,      // This need to be used to enable/disable NVRAM write protect
    GPIO_PIN_BUTTON,  // Button GPIO is configured to trigger either direction of interrupt
    GPIO_PIN_LED,     // LED GPIO, optional to provide visual effects
    GPIO_PIN_BATTERY, // Battery monitoring GPIO. When it is lower than particular level, it will give notification to the application
    GPIO_PIN_BUZZER,  // Buzzer GPIO, optional to provide audio effects
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 // other GPIOs are not used
    },
    /*.gpio_flag =*/
    {
    GPIO_SETTINGS_WP,
    GPIO_SETTINGS_BUTTON,
    GPIO_SETTINGS_LED,
    GPIO_SETTINGS_BATTERY,
    GPIO_SETTINGS_BUZZER,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    }
};



APPLICATION_INIT()
{
    bleapp_set_cfg((UINT8 *)proximity_db_data, sizeof(proximity_db_data), (void *)&bleprox_cfg,
       (void *)&bleprox_puart_cfg, (void *)&bleprox_gpio_cfg, bleprox_Create);

    ble_trace0("proximity_create\n");

    // BLE_APP_DISABLE_TRACING();     ////// Uncomment to disable all tracing
    BLE_APP_ENABLE_TRACING_ON_PUART();
}
