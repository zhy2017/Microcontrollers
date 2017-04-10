#ifndef BLE_LBS_H__
#define BLE_LBS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define VOLMEASUUID_BASE        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define VOLMEASS_UUID_SERVICE     	0x1526
#define VOLMEASS_UUID_VOL_CHAR    	0x1527
#define KEY_UUID_SERVICE 						0x1528
#define KEY_FLAG                  	0x1529

#define CHARG_UUID_SERVICE          0x1536
#define CHARG_FLAG                  0x1537															
															
// Forward declaration of the ble_lbs_t type. 
typedef struct ble_volmeass_s ble_volmeass_t;
typedef struct ble_key_s ble_key_t;
typedef struct ble_charg_s ble_charg_t;

typedef void (*ble_lbs_led_write_handler_t) (ble_volmeass_t * p_lbs, uint8_t new_state);
															
typedef struct
{
    ble_lbs_led_write_handler_t led_write_handler; /**< Event handler to be called when the LED Characteristic is written. */
} ble_volmeass_init_t;

typedef struct
{
    ble_lbs_led_write_handler_t led_write_handler; /**< Event handler to be called when the LED Characteristic is written. */
} ble_key_init_t;

typedef struct
{
    ble_lbs_led_write_handler_t led_write_handler; /**< Event handler to be called when the LED Characteristic is written. */
} ble_charg_init_t;

/**@brief mpu6050 Service structure. This structure contains various status information for the service. */
struct ble_volmeass_s
{
    uint16_t                    service_handle;      /**< Handle of Mpu6050 Service (as provided by the BLE stack). */
	  //ble_gatts_char_handles_t    led_char_handles;    /**< Handles related to the LED Characteristic. */
    ble_gatts_char_handles_t    mpu_char_handles;    /**< Handles related to the mpu Characteristic. */
	  uint8_t                     uuid_type;           /**< UUID type for the LED Button Service. */
    uint16_t                    conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
	  //ble_lbs_led_write_handler_t led_write_handler;   /**< Event handler to be called when the LED Characteristic is written. */
};	

struct ble_key_s
{
	uint16_t       service_handle;
	ble_gatts_char_handles_t  key_char_handles;
	uint8_t        uuid_type;
	uint16_t       conn_handle; 
};

struct ble_charg_s
{
	uint16_t       service_handle;
	ble_gatts_char_handles_t  charg_char_handles;
	uint8_t        uuid_type;
	uint16_t       conn_handle; 
};

uint32_t ble_charg_init(ble_charg_t * p_volmeass, const ble_charg_init_t * p_volmeass_init);
uint32_t ble_key_init(ble_key_t * p_volmeass, const ble_key_init_t * p_volmeass_init);
uint32_t ble_volmeass_init(ble_volmeass_t * p_mpus, const ble_volmeass_init_t * p_mpus_init);
uint16_t ble_send_charg_data(ble_charg_t * p_volmeass, uint8_t *pCharg);
uint16_t ble_send_key_data(ble_key_t * p_volmeass, uint8_t *pKey);
uint16_t ble_send_vol_data(ble_volmeass_t * p_mpus, int16_t *pAVal);
void ble_charg_on_ble_evt(ble_charg_t * p_lbs, ble_evt_t * p_ble_evt);
void ble_key_on_ble_evt(ble_key_t * p_lbs, ble_evt_t * p_ble_evt);
void ble_volmeass_on_ble_evt(ble_volmeass_t * p_lbs, ble_evt_t * p_ble_evt);
#endif
