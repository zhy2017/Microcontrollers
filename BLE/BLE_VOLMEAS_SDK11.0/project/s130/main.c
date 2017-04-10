/* Copyright (c) 2015 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

/** 
 * @brief Blinky Sample Application main file.
 *
 * This file contains the source code for a sample server application using the LED Button service.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "bsp.h"
#include "ble_gap.h"
#include "ble_volmeas.h"
#include "ble_dfu.h"
#include "dfu_app_handler.h"
#include "pstorage.h"
#include "app_trace.h"

#include "nrf_delay.h"
#include "twi_master.h"
#include "app_uart.h"

#include "nrf_drv_adc.h"
#include "app_util_platform.h"

#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 1                           /**< UART RX buffer size. */

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

//#define ADVERTISING_LED_PIN             BSP_LED_0_MASK                              /**< Is on when device is advertising. */
//#define CONNECTED_LED_PIN               BSP_LED_1_MASK                              /**< Is on when device has connected. */

//#define LEDBUTTON_LED_PIN               BSP_LED_2_MASK                              /**< LED to be toggled with the help of the LED Button Service. */
//#define LEDBUTTON_BUTTON_PIN            BSP_BUTTON_0                                /**< Button that will trigger the notification event with the LED Button Service */

#define DEVICE_NAME                     "King-SW"                                  /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms; this value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      BLE_GAP_ADV_TIMEOUT_GENERAL_UNLIMITED       /**< The advertising time-out (in units of seconds). When set to 0, we will never time out. */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            6                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */
#define VOLMEAS_READ_INTERVAL           APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< ADC采样间隔 1000ms读取一次*/


#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(100, UNIT_1_25_MS)            /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(200, UNIT_1_25_MS)            /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   70                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(30000, UNIT_10_MS)             /**< Connection supervisory time-out (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20000, APP_TIMER_PRESCALER) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (15 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define APP_GPIOTE_MAX_USERS            1                                           /**< Maximum number of users of the GPIOTE handler. */
#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_BOND                   1                                          /**< Perform bonding. */
#define SEC_PARAM_MITM                   0                                          /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                   0                                          /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS               0                                          /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES        BLE_GAP_IO_CAPS_NONE                       /**< No I/O capabilities. */
#define SEC_PARAM_OOB                    0                                          /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE           7                                          /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE           16                                         /**< Maximum encryption key size. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */

static dm_application_instance_t         m_app_handle;                              /**< Application identifier allocated by device manager */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_volmeass_t                   m_volmeass;                                      /**< MPU6050 Service instance. */
static ble_key_t                        m_key;
static ble_charg_t                      m_charg;
static int count = 0;
//#define LED                             24
#define BUTTON_PIN                      	5
#define CHARG_PIN                         3
//#define LED_PIN                         21
//#define LED_TEST                        22

#define ADC_BUFFER_SIZE 1                                /**< Size of buffer for ADC samples.  */
static nrf_adc_value_t       adc_buffer[ADC_BUFFER_SIZE]; /**< ADC buffer. */
static uint8_t               key_buffer;
static uint8_t               charg_buffer;
static int keyFlag = 0;
static ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */
APP_TIMER_DEF(m_volmeas_timer_id);      /**< adc采样定时器 */


void adc_sample(void);


/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
//static void leds_init(void)
//{
//    LEDS_CONFIGURE(ADVERTISING_LED_PIN | CONNECTED_LED_PIN | LEDBUTTON_LED_PIN);
//    LEDS_OFF(ADVERTISING_LED_PIN | CONNECTED_LED_PIN | LEDBUTTON_LED_PIN);
//}

static void volmeas_timeout_handler(void * p_context)
{
	  UNUSED_PARAMETER(p_context);
	//  printf("adc sampling ... \r\n");
		count++;
		if(count >= 6)
    {
			//adc_sample();
			count = 0;
			if(nrf_gpio_pin_read(CHARG_PIN) == 1)
			{
			    charg_buffer = 0x00;
					ble_send_charg_data(&m_charg, &charg_buffer);
					adc_sample();
			}
			else
			{
			    charg_buffer = 0x01;
					ble_send_charg_data(&m_charg, &charg_buffer);
			}
		}
}
/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    uint32_t err_code;
	
	  // 初始化定时器模块
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, false);
	  // 创建一个周期执行的定时器，用于读取MPU6050数据
    err_code = app_timer_create(&m_volmeas_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                volmeas_timeout_handler);
	  APP_ERROR_CHECK(err_code);	
}
static void application_timers_start(void)
{
    uint32_t err_code;

    // Start application timers.
    err_code = app_timer_start(m_volmeas_timer_id, VOLMEAS_READ_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
}

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

static void advertising_stop(void)
{
    uint32_t err_code;

    err_code = sd_ble_gap_adv_stop();
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;

    ble_uuid_t adv_uuids[] = {{VOLMEASS_UUID_SERVICE, m_volmeass.uuid_type}};

    // Build and set advertising data
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance = true;
    advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;


    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;

    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}

static void reset_prepare(void)
{
    uint32_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // Disconnect from peer.
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
        err_code = bsp_indication_set(BSP_INDICATE_IDLE);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        // If not connected, the device will be advertising. Hence stop the advertising.
        advertising_stop();
    }

    err_code = ble_conn_params_stop();
    APP_ERROR_CHECK(err_code);

    nrf_delay_ms(500);
}

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_volmeass_init_t init;
		ble_key_init_t key_init;
		ble_charg_init_t charg_init;
		
    err_code = ble_volmeass_init(&m_volmeass, &init);
    APP_ERROR_CHECK(err_code);
	
		err_code = ble_key_init(&m_key, &key_init);
		APP_ERROR_CHECK(err_code);
	
		err_code = ble_charg_init(&m_charg, &charg_init);
		APP_ERROR_CHECK(err_code);
	
		ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
    /** @snippet [DFU BLE Service initialization] */
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module that
 *          are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply
 *       setting the disconnect_on_fail config parameter, but instead we use the event
 *       handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;

    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));

    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);
 //   LEDS_ON(ADVERTISING_LED_PIN);
}


/**@brief Function for handling the Application's BLE stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
    //        LEDS_ON(CONNECTED_LED_PIN);
//            LEDS_OFF(ADVERTISING_LED_PIN);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
//            LEDS_OFF(CONNECTED_LED_PIN);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            err_code = app_button_disable();
            APP_ERROR_CHECK(err_code);

            advertising_start();
            break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle,
                                                   BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP,
                                                   NULL,
                                                   NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack
 *          event has been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_volmeass_on_ble_evt(&m_volmeass, p_ble_evt);
		ble_key_on_ble_evt(&m_key, p_ble_evt);
		ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
		ble_charg_on_ble_evt(&m_charg, p_ble_evt);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
		 ble_enable_params.gatts_enable_params.service_changed = 1;
	
    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    ble_gap_addr_t addr;

    err_code = sd_ble_gap_address_get(&addr);
    APP_ERROR_CHECK(err_code);
    err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &addr);
    APP_ERROR_CHECK(err_code);

    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

#ifdef BLE_DFU_APP_SUPPORT
    if (p_event->event_id == DM_EVT_LINK_SECURED)
    {
        app_context_load(p_handle);
    }
#endif // BLE_DFU_APP_SUPPORT

    return NRF_SUCCESS;
}

static void device_manager_init(bool erase_bonds)
{
    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;

    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));

    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.lesc         = SEC_PARAM_LESC;
    register_param.sec_param.keypress     = SEC_PARAM_KEYPRESS;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);
}
/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press/release).
 */
//static void button_event_handler(uint8_t pin_no, uint8_t button_action)
//{

//    switch (pin_no)
//    {
//        case LEDBUTTON_BUTTON_PIN:
//   
//            break;

//        default:
//            APP_ERROR_HANDLER(pin_no);
//            break;
//    }
//}

void TIMER2_IRQHandler()
{
	if(NRF_TIMER2->EVENTS_COMPARE[0] == 1)
	{
		NRF_TIMER2->EVENTS_COMPARE[0] = 0;
		
		if(nrf_gpio_pin_read(BUTTON_PIN) == 0)
		{
			keyFlag++;
		}
		else
		{
				if((keyFlag > 0) && (keyFlag  <= 50))
				{
					keyFlag = 0;

					key_buffer = 0x01;
					ble_send_key_data(&m_key, &key_buffer);
					NRF_TIMER2->TASKS_STOP = 1;
				}
				else if((keyFlag > 0) && (keyFlag >= 60))
				{
					keyFlag = 0;

					key_buffer = 0x11;
					ble_send_key_data(&m_key, &key_buffer);
					NRF_TIMER2->TASKS_STOP = 1;
				}
		}
			
	}
}


static void key_check_timer()
{
	NRF_TIMER2->PRESCALER = 4;
	NRF_TIMER2->MODE = 0;
	NRF_TIMER2->BITMODE = 3;
	NRF_TIMER2->CC[0] = 20000;
	NRF_TIMER2->INTENSET = 1 << 16;
	
	NRF_TIMER2->SHORTS = 1;
	
	NVIC_SetPriority(TIMER2_IRQn, 3);
	NVIC_ClearPendingIRQ(TIMER2_IRQn);
	NVIC_EnableIRQ(TIMER2_IRQn);
}

void GPIOTE_IRQHandler(void)
{
    if ( NRF_GPIOTE->EVENTS_IN[0] == 1 )
	  {
		    NRF_GPIOTE->EVENTS_IN[0] = 0; 
			
				NRF_TIMER2->TASKS_CLEAR = 1;
				NRF_TIMER2->TASKS_START = 1;
			
//				charg_buffer = 0x01;
//				ble_send_charg_data(&m_charg, &charg_buffer);
    }
		
    if ( NRF_GPIOTE->EVENTS_IN[1] == 1 )
	  {
		    NRF_GPIOTE->EVENTS_IN[1] = 0; 

				if(nrf_gpio_pin_read(CHARG_PIN) == 0)
				{
					charg_buffer = 0x01;
					ble_send_charg_data(&m_charg, &charg_buffer);
				}
				else
				{
					adc_sample();
					nrf_delay_ms(10);
					if(adc_buffer[0] > 389)
					{
						charg_buffer = 0x03;
						ble_send_charg_data(&m_charg, &charg_buffer);
					}
					else
					{
						charg_buffer = 0x02;
						ble_send_charg_data(&m_charg, &charg_buffer);
					}
					
				}
    } 
		
}

static void key_check_init()
{
	nrf_gpio_pin_pull_t config = NRF_GPIO_PIN_NOPULL;
  nrf_gpio_cfg_input(BUTTON_PIN, config);
	
	nrf_gpio_pin_pull_t charg_config = NRF_GPIO_PIN_PULLUP;
  nrf_gpio_cfg_input(CHARG_PIN, charg_config);

	NRF_GPIOTE->CONFIG[1] = 1 << 0
                     |(CHARG_PIN << 8)
                     |(3 << 16);
	
	NRF_GPIOTE->CONFIG[0] = 1 << 0
                     |(BUTTON_PIN << 8)
                     |(3 << 16);
	
	
	NRF_GPIOTE->INTENSET  = 0x03;
	
	NVIC_SetPriority(GPIOTE_IRQn, 3);
  NVIC_ClearPendingIRQ(GPIOTE_IRQn);
  NVIC_EnableIRQ(GPIOTE_IRQn);
}

/*@brief Function for initializing the button handler module.
 */
//static void buttons_init(void)
//{
//    uint32_t err_code;

//    //The array must be static because a pointer to it will be saved in the button handler module.
//    static app_button_cfg_t buttons[] =
//    {
//        {LEDBUTTON_BUTTON_PIN, false, BUTTON_PULL, button_event_handler}
//    };

//    err_code = app_button_init(buttons, sizeof(buttons) / sizeof(buttons[0]),
//                               BUTTON_DETECTION_DELAY);
//    APP_ERROR_CHECK(err_code);
//}


/**@brief Function for the Power Manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();

    APP_ERROR_CHECK(err_code);
}

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}
static void adc_event_handler(nrf_drv_adc_evt_t const * p_event)
{

	
    if (p_event->type == NRF_DRV_ADC_EVT_DONE)
    {	
      //  printf("V0%d", adc_buffer[0]);//输出ADC的采样值
			//  printf("\r\n");
			
			  ble_send_vol_data(&m_volmeass, &adc_buffer[0]);
//				ble_send_charg_data(&m_charg, &charg_buffer);
				nrf_delay_ms(10);
				adc_buffer[0] = 0;
    }
}
/**********************************************************************************************
 * 描  述 : ADC配置。输入通道：AIN5，分辨率：10位，参考电压：内部1.2V，分压系数：1/3
 * 参   数: 无
 * 返回值 : 无
 ***********************************************************************************************/ 
static void adc_config(void)
{
    ret_code_t ret_code;
	
    //Initialize ADC
    nrf_drv_adc_config_t config = NRF_DRV_ADC_DEFAULT_CONFIG;
    ret_code = nrf_drv_adc_init(&config, adc_event_handler);
    APP_ERROR_CHECK(ret_code);
	
	  //Configure and enable ADC channel 3
    static nrf_drv_adc_channel_t m_channel_0_config = NRF_DRV_ADC_DEFAULT_CHANNEL(NRF_ADC_CONFIG_INPUT_3); 
		m_channel_0_config.config.config.input = NRF_ADC_CONFIG_SCALING_INPUT_TWO_THIRDS;
    nrf_drv_adc_channel_enable(&m_channel_0_config);
	
}
void adc_sample(void)
{
    ret_code_t ret_code;

    ret_code = nrf_drv_adc_buffer_convert(adc_buffer, ADC_BUFFER_SIZE);       // Allocate buffer for ADC
    APP_ERROR_CHECK(ret_code);
	  nrf_drv_adc_sample();// Trigger ADC conversion			
}
/**@brief Function for application main entry.
 */
int main(void)
{
	//  uint32_t err_code;
		bool erase_bonds;
	
//	  const app_uart_comm_params_t comm_params =
//    {
//          RX_PIN_NUMBER,
//          TX_PIN_NUMBER,
//          RTS_PIN_NUMBER,
//          CTS_PIN_NUMBER,
//          APP_UART_FLOW_CONTROL_DISABLED,
//          false,
//          UART_BAUDRATE_BAUDRATE_Baud115200
//    };

//    APP_UART_FIFO_INIT(&comm_params,
//                         UART_RX_BUF_SIZE,
//                         UART_TX_BUF_SIZE,
//                         uart_error_handle,
//                         APP_IRQ_PRIORITY_LOW,
//                         err_code);

//    APP_ERROR_CHECK(err_code);
	

	  //Initialize.
    timers_init();
		key_check_timer();
		key_check_init();
		
    ble_stack_init();
		device_manager_init(erase_bonds);
    gap_params_init();
    services_init();
    advertising_init();
		
    conn_params_init();

    adc_config();
		application_timers_start();//启动定时器
    advertising_start();

    // Enter main loop.
    for (;;)
    {
        power_manage();
    }
}


/**
 * @}
 */
