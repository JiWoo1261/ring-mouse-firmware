
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "feature_config.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "ble_advertising.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "peer_manager_handler.h"

#include "nrf_drv_clock.h"

#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"

#include "nrf_drv_wdt.h"

#include "nrf_delay.h"
#include "mpu9255_register_map.h"
#include "Drv_mpu.h"
#include "math.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_power.h"
#include "common.h"
#include "app_config2.h"
#include "sig_processing.h"
#include "uicr.h"

/* add buttonless dfu header */
#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"
#include "ble_dfu.h"
#include "nrf_bootloader_info.h"


#ifdef NDEBUG
#define C_FIRMWARE_VERSION              "V2.219c3"
#else
#define	C_FIRMWARE_VERSION		"V2.219.Debug"
#endif

#define DEVICE_NAME			"RingMouse [V2.219c3]"			/**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME		"Futuristec"                            /**< Manufacturer. Will be passed to Device Information Service. */

#define C_PUSH_PIN_NUM                  8
#define	C_POWER_OFF_PIN_NUM             24
#define	C_CHARGE_COMPLETE_PIN_NUM	15
#define IMU_INT_PIN                     18

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define SWITCH_TIME			50                                      /* milliseconds */
#ifdef POLL_125HZ
#define	MOUSE_MOVE_TIME			8
#else
#define	MOUSE_MOVE_TIME			15 //20
#endif

/* Only used in Mouse_movement_handler *
 * 3 minutes -> 20ms uint ticks */
#define C_NO_MOUSE_ACTION_TIMEOUT       ((3)*(60)*(1000)/(MOUSE_MOVE_TIME))
#define CURSOR_MOVEMENT_TIMEOUT         ((200)/(MOUSE_MOVE_TIME))

#define LED_TOGGLE_INTERVAL             100
#define SWITCH_INTERVAL                 SWITCH_TIME
#define MOUSE_MOVE_INTERVAL             MOUSE_MOVE_TIME
#define CURSOR_MODE_TIMEOUT             3000

#define MIN_BATTERY_LEVEL               81                                          /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Increment between each simulated battery level measurement. */

#define PNP_ID_VENDOR_ID_SOURCE         0x02                                        /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                0x1915                                      /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID               0xEEEE                                      /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION          0x0001                                      /**< Product Version. */

/*lint -emacro(524, MIN_CONN_INTERVAL) // Loss of precision */

/* Original Value regarding connection parameters*/
//#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum connection interval (7.5 ms). */
//#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Maximum connection interval (15 ms). */
//#define SLAVE_LATENCY                   20                                          /**< Slave latency. */
//#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(3000, UNIT_10_MS)             /**< Connection supervisory timeout (3000 ms). */
//#define APP_ADV_FAST_INTERVAL           0x0028                                      /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
//#define APP_ADV_SLOW_INTERVAL           0x00A0                                      /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 100 ms.). */
//#define APP_ADV_FAST_DURATION           300                                         /**< The advertising duration of fast advertising in units of 10 milliseconds. */
//#define APP_ADV_SLOW_DURATION           6000                                        /**< The advertising duration of slow advertising in units of 10 milliseconds. */

/* Modified Value to improve connection speed @ V2.219 (2025.12.07)  */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Maximum connection interval (15 ms). */
#define SLAVE_LATENCY                   0                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(2000, UNIT_10_MS)             /**< Connection supervisory timeout (3000 ms). */

#define APP_ADV_FAST_INTERVAL           0x0020                                      /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL           0x00A0                                      /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 100 ms.). */

#define APP_ADV_FAST_DURATION           3000                                         /**< The advertising duration of fast advertising in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION           9000                                        /**< The advertising duration of slow advertising in units of 10 milliseconds. */
/**********************************************************************/

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAM_UPDATE_COUNT     3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define SWIFT_PAIR_SUPPORTED            1                                           /**< Swift Pair feature is supported. */
#if SWIFT_PAIR_SUPPORTED == 1
#define MICROSOFT_VENDOR_ID             0x0006                                      /**< Microsoft Vendor ID.*/
#define MICROSOFT_BEACON_ID             0x03                                        /**< Microsoft Beacon ID, used to indicate that Swift Pair feature is supported. */
#define MICROSOFT_BEACON_SUB_SCENARIO   0x00                                        /**< Microsoft Beacon Sub Scenario, used to indicate how the peripheral will pair using Swift Pair feature. */
#define RESERVED_RSSI_BYTE              0x80                                        /**< Reserved RSSI byte, used to maintain forwards and backwards compatibility. */
#endif

#define MOVEMENT_SPEED                  5                                           /**< Number of pixels by which the cursor is moved each time a button is pushed. */
#define INPUT_REPORT_COUNT              3                                           /**< Number of input reports in this application. */
#define INPUT_REP_BUTTONS_LEN           3                                           /**< Length of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_LEN          3                                           /**< Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1                                           /**< Length of Mouse Input Report containing media player data. */
#define INPUT_REP_BUTTONS_INDEX         0                                           /**< Index of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_INDEX        1                                           /**< Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MPLAYER_INDEX         2                                           /**< Index of Mouse Input Report containing media player data. */
#define INPUT_REP_REF_BUTTONS_ID        1                                           /**< Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_MOVEMENT_ID       2                                           /**< Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MPLAYER_ID        3                                           /**< Id of reference to Mouse Input Report containing media player data. */

#define BASE_USB_HID_SPEC_VERSION       0x0101                                      /**< Version number of base USB HID Specification implemented by this application. */

#define SCHED_MAX_EVENT_DATA_SIZE       APP_TIMER_SCHED_EVENT_DATA_SIZE             /**< Maximum size of scheduler events. */

#ifdef SVCALL_AS_NORMAL_FUNCTION
//#define SCHED_QUEUE_SIZE                20                                          /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#define SCHED_QUEUE_SIZE                40            //Ken                              /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
//#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */
#define SCHED_QUEUE_SIZE                30            //Ken                              /**< Maximum number of events in the scheduler queue. */
#endif

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
 
#define SCROLL_THRESHOLD                15
#define SCROLL_SCALE_CONSTANT           23

/* Scaling constant */
#define NORMAL_SCALE_X                  ((RAD_TO_DEG)*(33.51))
#define NORMAL_SCALE_Y                  ((RAD_TO_DEG)*(23.04))
//#define NORMAL_SCALE_X                  ((RAD_TO_DEG)*(60))
//#define NORMAL_SCALE_Y                  ((RAD_TO_DEG)*(40))
/* End of scaling */

#define TILT_ANGLE                      ((HORIZANTAL_TILT+VERTICAL_TILT)/(2.0))

/* 앞으로 가기/뒤로가기 기능을 위한 변수들 */
#define Criteria_Distance   10     //스크롤과 앞/뒤 동작을 판단하기 위한 최소 이동거리 
#define Criteria_ScrollCheck_Positive  80  // 스크롤과 앞으로가기/뒤로가기를 구분하기 위한 + 기울기 값 (accum_y * 100 / accum_x 값)
#define Criteria_ScrollCheck_Negative  -80  // 스크롤과 앞으로가기/뒤로가기를 구분하기 위한 - 기울기 값 (accum_y * 100 / accum_x 값)
#define NumberofAbandonInputs   5  //초기 입력 중 진동으로 인하여 무시 할 입력 갯수.
static int32_t accum_dx = 0;          // 자이로의 deltax를 누적한 값. 기울기 값 및 이동거리 계산에 사용.
static int32_t accum_dy = 0;          // 자이로의 deltay를 누적한 값. 기울기 값 및 이동거리 계산에 사용.
static int32_t IMU_input_counts = 0;      // 스크롤 버튼 터치 후 초기 5개? 값을 버릴 때 사용하기 위한 변수. 버튼 터치 시 진동에 의한 입력을 무시 하기 위한 동작임. 
static bool bRatio_calculate_once = true;  // ratio 계산으로 조건 만족하면 한번만 하면 됨. 한번만 계산할 때 사용하기 위한 변수.
static bool bScrollActivated = false;      //스크롤 버튼 터치 후 일정거리 이상 움직이면 ratio 에 따라 스크롤일지 앞/뒤일지 한번만 결정 함. 이 때 사용하기 위한 변수
static bool bScrollAction = false;     //위 ratio 계산결과 스크롤일경우, 스크롤 동작을 수행하기 위한 변수.
static bool bBackforwardAction = false;  //위 ratio 계산결과 앞으로가기/뒤로가기일경우, 앞으로가기/뒤로가기 동작을 수행하기 위한 변수.
static int32_t ratio;
// 스크롤 동작일때는 스크롤버튼을 터치하고 있는동안 계속 스크롤 동작을 수행 함. 그러나 앞으로 가기/뒤로가기 동작일경우, 
// 앞으로가기 또는 뒤로가기를 한번만 수행하고 나면 스크롤 버튼을 놓았다고 다시 터치할 때 까지는 deactivate 되어야 함.

enum Skilled_mode_t {
    S_MODE_NONE = 0,
    S_MODE_1,
    S_MODE_2
};

APP_TIMER_DEF(m_LED_timer_id);                                                      /**< LED Timer. */
APP_TIMER_DEF(m_switch_timer_id);                                                   /**< switch time.r */
APP_TIMER_DEF(m_mouse_move_id);
APP_TIMER_DEF(m_cursor_mode_timer);

BLE_BAS_DEF(m_bas);                                                                 /**< Battery service instance. */
BLE_HIDS_DEF(m_hids,                                                                /**< HID service instance. */
             NRF_SDH_BLE_TOTAL_LINK_COUNT,
             INPUT_REP_BUTTONS_LEN,
             INPUT_REP_MOVEMENT_LEN,
             INPUT_REP_MEDIA_PLAYER_LEN);
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint32_t base_ticks = 0;
static bool b_mouse_movement_flag = false;
static bool b_mouse_movement_flag_2 = false;
static bool b_scroll_flag = false;
static bool b_hold_cursor_flag = false;
volatile bool b_left_handed_flag = false;
static volatile int skilled_mode = S_MODE_NONE;
static int mouse_movement_skip300ms_counter = 30;

static bool fine_scale_flag = false;
static int small_value_counter = 0;

static uint64_t t2_timestamps[4] = {0};
static bool b_move_pushed = false;
static bool b_sdh_disable_requested = false;

/* cursor movement history */
#define SIZE_BUFFER   8   //was 8
int c_index = 0;             /* cursor position index */
int dx[SIZE_BUFFER] = {0};
int dy[SIZE_BUFFER] = {0};

/* back forward, scroll */
int back_forward_state = 0;  /* 0: None, 1: Back, 2: Forward, 3: Scroll */
int skip_scroll_after_backforward_count = 0;
int scroll_count = 0;

#ifdef D_USE_LINACC
/* lin acc test */
static float prev_laccx = 0, prev_laccz = 0;
static float prev_velx = 0, prev_velz = 0;
static float prev_dispx = 0, prev_dispz = 0;
#endif

static uint8_t ub_cursor_mode_changed_count = 5;

static uint16_t				uw_count = 0;
static uint32_t				uw_poweroff_count = 0;
#ifdef D_TIMEOUT_POWER_OFF
static bool				b_timeout_poweroff_flag = false;
#endif // D_TIMEOUT_POWER_OFF
static uint32_t				ul_no_mouse_action_count = 0;
static bool				m_in_boot_mode = false;                                    /**< Current protocol mode. */
static uint16_t				m_conn_handle  = BLE_CONN_HANDLE_INVALID;                  /**< Handle of the current connection. */
static pm_peer_id_t			m_peer_id;                                                 /**< Device reference handle to the current bonded central. */
static sensorsim_cfg_t                  m_battery_sim_cfg;                                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t                m_battery_sim_state;                                       /**< Battery Level sensor simulator state. */
static ble_uuid_t			m_adv_uuids[] =                                            /**< Universally unique service identifiers. */
{
    {BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}
};

#if SWIFT_PAIR_SUPPORTED == 1
static uint8_t m_sp_payload[] =                                                     /**< Payload of advertising data structure for Microsoft Swift Pair feature. */
{
    MICROSOFT_BEACON_ID,
    MICROSOFT_BEACON_SUB_SCENARIO,
    RESERVED_RSSI_BYTE
};
static ble_advdata_manuf_data_t m_sp_manuf_advdata =                                /**< Advertising data structure for Microsoft Swift Pair feature. */
{
    .company_identifier = MICROSOFT_VENDOR_ID,
    .data               =
    {
        .size   = sizeof(m_sp_payload),
        .p_data = &m_sp_payload[0]
    }
};
static ble_advdata_t m_sp_advdata;
#endif

static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt);
static void mouse_movement_send(int16_t x_delta, int16_t y_delta);
static void mouse_button_send(uint8_t button, uint8_t scroll, uint8_t sideways);
static void Mouse_movement_handler(void *p_context);
void save_bias(int gbias0, int gbias2);

magn_values_t magn_values;

volatile uint8_t m_connected = 0;

typedef enum
{
	POWER_OFF_CHECK_PUSH_BUTTON,
	POWER_OFF_CHECK_RELEASE_BUTTON,
	POWER_OFF_PROCESSING,
	NO_OF_POWER_OFF_STATE
}TPOWEROFFSTATE_ID;
TPOWEROFFSTATE_ID	ub_PowerOffState;

#ifdef D_USE_ADC
#define	NUM_OF_ADC_CH		2
#define	SAMPLES_IN_BUFFER	5
#define	TOTAL_SAMPLES		(NUM_OF_ADC_CH*SAMPLES_IN_BUFFER)

#define	C_ADC_VBUS_ARRARY_NUM	0	// AIN4, VBUS
#define	C_ADC_VBAT_ARRARY_NUM	1	// AIN6, VBAT

#endif // D_USE_ADC

#ifdef D_BATTERY_LEVEL_CONTROL
#define C_DISCHARGE_VOLT    	3.0		// 3.0V
#define	C_LOW_BATTERY_VOLT	3.35            // 3.35V
#define	C_VERY_LOW_BATTERY_VOLT	2.8		// 2.8V
#define C_FULL_CHARGE_VOLT  	4.1		// 4.1V
#define	C_CONNECTED_VBUS_VOLT	4.3		// 4.3V
#define	C_LOW_BATTERY_LEVEL	5		// 5%
#define	C_VERYLOW_BATTERY_LEVEL	1		// 1%
static int8_t  battery_level = 100;
static bool	b_battery_low = false;
static bool	b_battery_very_low = false;
static float adc_voltage[NUM_OF_ADC_CH] = {0,0};
#endif 	// D_BATTERY_LEVEL_CONTROL

#ifdef D_USE_WDT
static	nrf_drv_wdt_channel_id m_channel_id;

/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
    NRF_LOG_INFO("wdt_event_handler");

    bsp_board_leds_off();

    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}
#endif // D_USE_WDT

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (b_sdh_disable_requested) {
        return;
    }

    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

static void advertising_config_get(ble_advertising_t      * const p_advertising,
                                   ble_adv_modes_config_t * p_config)
{
    *p_config = m_advertising.adv_modes_config;
    p_config->ble_adv_fast_enabled = true;
}

static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&m_advertising, &config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}



/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
#ifdef D_BATTERY_LEVEL_CONTROL
#else // D_BATTERY_LEVEL_CONTROL
    uint8_t  battery_level;
	battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);
#endif // D_BATTERY_LEVEL_CONTROL

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_FORBIDDEN) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

#ifdef D_USE_ADC
static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(1);
static nrf_saadc_value_t     m_buffer_pool[2][TOTAL_SAMPLES];
static nrf_ppi_channel_t     m_ppi_channel;
static uint32_t              m_adc_evt_counter;

void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    NRF_LOG_INFO("timer_handler");
}

void saadc_sampling_event_init(void)
{
//	NRF_LOG_INFO("saadc_sampling_event_init");
    ret_code_t err_code;

    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);

    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, 400);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
    nrf_drv_timer_enable(&m_timer);

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_enable(void)
{
//	NRF_LOG_INFO("saadc_sampling_event_enable");

    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
	static	uint16_t	uw_sum_adc[NUM_OF_ADC_CH] = {0,0};
	static	uint16_t	uw_avg_adc[NUM_OF_ADC_CH] = {0,0};
	static	uint8_t		prev_battery_level = 100;
//	NRF_LOG_INFO("saadc_callback");
	if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
	{
		ret_code_t err_code;

		err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, TOTAL_SAMPLES);
		APP_ERROR_CHECK(err_code);

		int i;
//        NRF_LOG_INFO("%d ADC event number: %d", base_ticks, (int)m_adc_evt_counter);

		for (i = 0; i < NUM_OF_ADC_CH; i++)
		{
			uw_sum_adc[i] = 0;
		}

		for (i = 0; i < TOTAL_SAMPLES; i++)
		{
			if(p_event->data.done.p_buffer[i] < 0)
			{
				p_event->data.done.p_buffer[i] = 0;
			}

			uw_sum_adc[i % NUM_OF_ADC_CH] += p_event->data.done.p_buffer[i];
#ifdef D_ADC_DEBUG
		    NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
#endif // D_ADC_DEBUG
		}

		for (i = 0; i < NUM_OF_ADC_CH; i++)
		{
		    uw_avg_adc[i] = uw_sum_adc[i] / SAMPLES_IN_BUFFER;
		    adc_voltage[i] = (float)(uw_avg_adc[i])/1023*6;
#ifdef D_ADC_DEBUG
			NRF_LOG_INFO("Avg ADC[%d] = %d", i, uw_avg_adc[i]);
			NRF_LOG_INFO("Real Volt = " NRF_LOG_FLOAT_MARKER "V",NRF_LOG_FLOAT(adc_voltage[i]));
#endif // D_ADC_DEBUG
		}
#ifdef D_BATTERY_LEVEL_CONTROL
		//battery_level = adc_voltage[1]/4.2*100;		// 0: ADC5, 1: ADC6, 2: ADC7
#ifdef D_NEW_BATTERY_LEVEL_CALC
		if(adc_voltage[C_ADC_VBAT_ARRARY_NUM] >= C_LOW_BATTERY_VOLT)
		{
			battery_level = (100-C_LOW_BATTERY_LEVEL)/(C_FULL_CHARGE_VOLT-C_LOW_BATTERY_VOLT)*adc_voltage[C_ADC_VBAT_ARRARY_NUM]
							+ (100-(100-C_LOW_BATTERY_LEVEL)/(C_FULL_CHARGE_VOLT-C_LOW_BATTERY_VOLT)*C_FULL_CHARGE_VOLT);
		}
		else
		{
			battery_level = (C_LOW_BATTERY_LEVEL-C_VERYLOW_BATTERY_LEVEL)/(C_LOW_BATTERY_VOLT-C_VERY_LOW_BATTERY_VOLT)*adc_voltage[C_ADC_VBAT_ARRARY_NUM]
							+ (C_VERYLOW_BATTERY_LEVEL-(C_LOW_BATTERY_LEVEL-C_VERYLOW_BATTERY_LEVEL)/(C_LOW_BATTERY_VOLT-C_VERY_LOW_BATTERY_VOLT)*C_VERY_LOW_BATTERY_VOLT);
		}

		if(battery_level < 1)
		{
			battery_level = 1;
		}
#else //
		if(adc_voltage[C_ADC_VBAT_ARRARY_NUM] > C_DISCHARGE_VOLT)
		{
			battery_level = (adc_voltage[C_ADC_VBAT_ARRARY_NUM]-C_DISCHARGE_VOLT)/(C_FULL_CHARGE_VOLT - C_DISCHARGE_VOLT)*100;		// 0: ADC5, 1: ADC6, 2: ADC7
		}
		else
		{
			battery_level = 0;
		}
#endif //
		
		if(battery_level > 100)
		{
			battery_level = 100;
		}

		// to prevent the increase in battery levels, when the USB cable is not connected
		if(adc_voltage[C_ADC_VBUS_ARRARY_NUM] < C_CONNECTED_VBUS_VOLT)	// if the USB cable is not connected
		{
			if(battery_level > prev_battery_level)
			{
				battery_level = prev_battery_level;
			}
		}

		prev_battery_level = battery_level;
#ifdef D_BATTERY_LEVEL_DEBUG
		NRF_LOG_INFO("%d, Real Volt = " NRF_LOG_FLOAT_MARKER "V, battery level = %d",base_ticks, NRF_LOG_FLOAT(adc_voltage[C_ADC_VBAT_ARRARY_NUM]), battery_level);
//		NRF_LOG_INFO("%d battery_level = %d", base_ticks, battery_level);
#endif // D_BATTERY_LEVEL_DEBUG

		battery_level_update();

//		adc_voltage = 3.0; // for battery low test
		if(adc_voltage[C_ADC_VBAT_ARRARY_NUM] > (float)C_LOW_BATTERY_VOLT)
		{
			b_battery_low = false;
			b_battery_very_low = false;
		}
		else if(adc_voltage[C_ADC_VBAT_ARRARY_NUM] > (float)C_VERY_LOW_BATTERY_VOLT)
		{
			b_battery_low = true;
			b_battery_very_low = false;
		}
		else
		{
			b_battery_low = true;
			b_battery_very_low = true;
		}
#endif // D_BATTERY_LEVEL_CONTROL
		m_adc_evt_counter++;
	}
}

void saadc_init(void)
{
    ret_code_t err_code;

    nrf_saadc_channel_config_t channel_config = NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN4);

    err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

	err_code = nrf_drv_saadc_channel_init(4, &channel_config);
	APP_ERROR_CHECK(err_code);

	channel_config.pin_p = NRF_SAADC_INPUT_AIN6;
	err_code = nrf_drv_saadc_channel_init(6, &channel_config);
	APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], TOTAL_SAMPLES);
    APP_ERROR_CHECK(err_code);

//    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
//    APP_ERROR_CHECK(err_code);

}
#endif // D_USE_ADC

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for setting filtered whitelist.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void whitelist_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
                   peer_id_count,
                   BLE_GAP_WHITELIST_ADDR_MAX_COUNT);

    err_code = pm_whitelist_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for setting filtered device identities.
 *
 * @param[in] skip  Filter passed to @ref pm_peer_id_list.
 */
static void identities_set(pm_peer_id_list_skip_t skip)
{
    pm_peer_id_t peer_ids[BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT];
    uint32_t     peer_id_count = BLE_GAP_DEVICE_IDENTITIES_MAX_COUNT;

    ret_code_t err_code = pm_peer_id_list(peer_ids, &peer_id_count, PM_PEER_ID_INVALID, skip);
    APP_ERROR_CHECK(err_code);

    err_code = pm_device_identities_list_set(peer_ids, peer_id_count);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(bool erase_bonds)
{
   uint32_t err_code;

//   err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
//   APP_ERROR_CHECK(err_code);

//	NRF_LOG_INFO("advertising start %d", erase_bonds);
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
         whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);
//        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, 4);
//        APP_ERROR_CHECK(err_code);

        ret_code_t ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(ret);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
	pm_handler_on_pm_evt(p_evt);
	pm_handler_disconnect_on_sec_failure(p_evt);
	pm_handler_flash_clean(p_evt);

	switch (p_evt->evt_id)
	{
		case PM_EVT_BONDED_PEER_CONNECTED:	// added
			NRF_LOG_INFO("%d PM_EVT_BONDED_PEER_CONNECTED in pm_evt_handler", base_ticks);
			break;

		case PM_EVT_CONN_CONFIG_REQ:	// added
			NRF_LOG_INFO("%d PM_EVT_CONN_CONFIG_REQ in pm_evt_handler", base_ticks);
			break;

		case PM_EVT_CONN_SEC_START:	// added
			NRF_LOG_INFO("%d PM_EVT_CONN_SEC_START in pm_evt_handler", base_ticks);
			break;

		case PM_EVT_CONN_SEC_SUCCEEDED:
			NRF_LOG_INFO("%d PM_EVT_CONN_SEC_SUCCEEDED in pm_evt_handler", base_ticks);
			m_peer_id = p_evt->peer_id;
                        m_connected = 1;
			break;

		case PM_EVT_CONN_SEC_FAILED:	// added
			NRF_LOG_INFO("%d PM_EVT_CONN_SEC_FAILED in pm_evt_handler", base_ticks);
			break;

		case PM_EVT_CONN_SEC_CONFIG_REQ:	// added
			NRF_LOG_INFO("%d PM_EVT_CONN_SEC_CONFIG_REQ in pm_evt_handler", base_ticks);
			pm_conn_sec_config_t config = {.allow_repairing = true};
			pm_conn_sec_config_reply(p_evt->conn_handle, &config);
			break;

		case PM_EVT_CONN_SEC_PARAMS_REQ:	// added
			NRF_LOG_INFO("%d PM_EVT_CONN_SEC_PARAMS_REQ in pm_evt_handler", base_ticks);
			break;

		case PM_EVT_PEERS_DELETE_SUCCEEDED:
			NRF_LOG_INFO("%d PM_EVT_PEERS_DELETE_SUCCEEDED in pm_evt_handler", base_ticks);
			advertising_start(false);
			break;

		case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
			if (p_evt->params.peer_data_update_succeeded.flash_changed
				&& (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
			{
				NRF_LOG_INFO("%d New Bond, add the peer to the whitelist if possible", base_ticks);
				// Note: You should check on what kind of white list policy your application should use.

				whitelist_set(PM_PEER_ID_LIST_SKIP_NO_ID_ADDR);
			}
			break;

		case PM_EVT_LOCAL_DB_CACHE_APPLIED:	// added
			NRF_LOG_INFO("%d PM_EVT_LOCAL_DB_CACHE_APPLIED in pm_evt_handler", base_ticks);
			break;

		default:
			NRF_LOG_INFO("%d pm_evt_handler, default %d", base_ticks, p_evt->evt_id);
			break;
	}
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void ble_advertising_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


static void LED_timer_handler(void *p_context)
{
    static uint8_t ub_connected_count = 0;
    static uint8_t ub_disconnected_count = 0;
    
    UNUSED_PARAMETER(p_context);

    base_ticks++;

    if(ub_PowerOffState == POWER_OFF_CHECK_RELEASE_BUTTON)
    {
        nrf_gpio_pin_toggle(BSP_LED_0);    // LED Blinking
    }
    else if(ub_PowerOffState == POWER_OFF_CHECK_PUSH_BUTTON)
    {
        if((adc_voltage[C_ADC_VBUS_ARRARY_NUM] >= C_CONNECTED_VBUS_VOLT)	// if the USB cable is connected
                && (nrf_gpio_pin_read(C_CHARGE_COMPLETE_PIN_NUM) == 0)			// if not full-charged
                 && (adc_voltage[C_ADC_VBAT_ARRARY_NUM] < C_FULL_CHARGE_VOLT))	// if VBAT < Full_Charge_Voltage(4.1V)
        {
            nrf_gpio_pin_write(BSP_LED_0, 1);  // LED OFF
        }
        else
        {
            if(m_connected == 1)
            {
#ifdef D_BATTERY_LEVEL_CONTROL
                if(b_battery_low == true)
                {
                    if(ub_connected_count == 0)
                    {
                        nrf_gpio_pin_write(BSP_LED_0, 1);  // LED OFF
                    }
                    else if(ub_connected_count == 23)	// 2.3 sec
                    {
                        nrf_gpio_pin_write(BSP_LED_0, 0);  // LED ON
                    }
                    else if(ub_connected_count == 24)	// 2.4 sec
                    {
                        nrf_gpio_pin_write(BSP_LED_0, 1);  // LED OFF
                    }
                    else if(ub_connected_count == 25)	// 2.5 sec
                    {
                        nrf_gpio_pin_write(BSP_LED_0, 0);  // LED ON
                    }
                    else if(ub_connected_count >= 26)	// 2.6 sec
                    {
                        nrf_gpio_pin_write(BSP_LED_0, 1);  // LED OFF
                        ub_connected_count = 0;
                    }
                }
                else
#endif // D_BATTERY_LEVEL_CONTROL
                {
                    if (ub_cursor_mode_changed_count <= 5) {
                        if (ub_cursor_mode_changed_count == 0) {
                            nrf_gpio_pin_write(BSP_LED_0, 0);  // LED ON
                        } else if (ub_cursor_mode_changed_count == 5) {
                            nrf_gpio_pin_write(BSP_LED_0, 1);
                        }
                        ub_cursor_mode_changed_count++;

                    } else {
    #ifdef D_CHECK_ALIVE_LED_ACTION
                        if(ub_connected_count == 0)
                        {
                            nrf_gpio_pin_write(BSP_LED_0, 1);  // LED OFF
                        }
                        else if(ub_connected_count == 39)	// 3.9 sec
                        {
                            nrf_gpio_pin_write(BSP_LED_0, 0);  // LED ON
                        }
                        else if(ub_connected_count >= 40)	// 4.0 sec
                        {
                            nrf_gpio_pin_write(BSP_LED_0, 1);  // LED OFF
                            ub_connected_count = 0;
                        }
    #else// D_CHECK_ALIVE_LED_ACTION
                        nrf_gpio_pin_write(BSP_LED_0, 1);  // LED OFF
    #endif // D_CHECK_ALIVE_LED_ACTION
                    }
                }
                ub_connected_count++;
                ub_disconnected_count = 0;
            }
            else if(m_connected == 0)
            {
                ub_connected_count = 0;
                if(ub_disconnected_count >= 8)                
                {
                    nrf_gpio_pin_toggle(BSP_LED_0);    // LED Blinking
                    ub_disconnected_count = 0;
                }
                ub_disconnected_count++;
            }
        }
    }
}

static void switch_timer_handler(void *p_context)
{
    static uint32_t	uw_battery_very_low_count = 0;
    uint32_t poweroff_ms;

    UNUSED_PARAMETER(p_context);

    switch(ub_PowerOffState)
    {
        case POWER_OFF_CHECK_PUSH_BUTTON:
            if(nrf_gpio_pin_read(C_PUSH_PIN_NUM) == 0)	// if pushed
            {
                uw_count++;
                if(uw_count == 1)
                {
                    NRF_LOG_INFO("%d, Push pressed",base_ticks);
                }
                if(uw_count == (2*1000/SWITCH_TIME))
                {
                    ub_PowerOffState = POWER_OFF_CHECK_RELEASE_BUTTON;
                    NRF_LOG_INFO("%d, power off flag is true", base_ticks);
                }
            }
            else
            {
                uw_count = 0;
                if(b_battery_very_low == true)
                {
                    uw_battery_very_low_count++;
                    if(uw_battery_very_low_count > (2*60*1000/SWITCH_TIME))
                    {
                        ub_PowerOffState = POWER_OFF_CHECK_RELEASE_BUTTON;
                        b_timeout_poweroff_flag = true;
                        NRF_LOG_INFO("%d, Battery very low, Power Off... Bye~~~",base_ticks);
                    }
                }
                else
                {
                    uw_battery_very_low_count= 0;
                }
            }
            break;
        case POWER_OFF_CHECK_RELEASE_BUTTON:
    #ifdef D_TIMEOUT_POWER_OFF
            if((nrf_gpio_pin_read(C_PUSH_PIN_NUM) == 1) || (b_timeout_poweroff_flag == true))	// if released or power-off timeout 
    #else // D_TIMEOUT_POWER_OFF
            if(nrf_gpio_pin_read(C_PUSH_PIN_NUM) == 1)	// if released 
    #endif // D_TIMEOUT_POWER_OFF
            {
                //app_timer_stop(m_LED_timer_id);
                //app_timer_stop(m_switch_timer_id);
                //app_timer_stop(m_mouse_move_id);

                if (nrf_sdh_is_enabled()) {
                    b_sdh_disable_requested = true;
                    //If SD is enabled, stop it.
                    nrf_sdh_disable_request();
                }

                saveCalibration(acc_bias, gyro_bias, mag_bias, mag_scale, b_left_handed_flag);
                nrf_delay_ms(10);

                nrf_gpio_pin_write(C_POWER_OFF_PIN_NUM , 1); 			  // POWER_OFF High
                ub_PowerOffState = POWER_OFF_PROCESSING;
                uw_poweroff_count = 0;
                NRF_LOG_INFO("%d, Push Released or Timeout, POWER_OFF High", base_ticks);
            }
            break;
        case POWER_OFF_PROCESSING:
            uw_poweroff_count++;
            poweroff_ms = uw_poweroff_count * SWITCH_TIME;

            if(poweroff_ms == SWITCH_TIME)
            {
                nrf_gpio_pin_write(BSP_LED_0, 1);				// LED Off
            }
            else if(poweroff_ms == 500)
            {
                nrf_gpio_pin_write(C_POWER_OFF_PIN_NUM , 0);		// POWER_OFF Low
                NRF_LOG_INFO("%d, POWER_OFF Low",base_ticks);
            }
            else if(poweroff_ms == 1000)
            {
                nrf_gpio_pin_write(C_POWER_OFF_PIN_NUM , 1);		// POWER_OFF High
                NRF_LOG_INFO("%d, POWER_OFF High",base_ticks);
            }
            else if(poweroff_ms == 1500)
            {
                nrf_gpio_pin_write(C_POWER_OFF_PIN_NUM , 0);		// POWER_OFF Low
                nrf_gpio_pin_write(BSP_LED_0, 0);				// LED On
                NRF_LOG_INFO("%d, POWER_OFF Low, LED On",base_ticks);
            }
            else if(poweroff_ms == 1700)
            {
                nrf_gpio_pin_write(BSP_LED_0, 1);				// LED Off
                NRF_LOG_INFO("%d, LED Off",base_ticks);
            }
            else if(poweroff_ms == 1900)
            {
                nrf_gpio_pin_write(BSP_LED_0, 0);				// LED On
                NRF_LOG_INFO("%d, LED On 2",base_ticks);
            }
            else if(poweroff_ms == 2100)
            {
                nrf_gpio_pin_write(BSP_LED_0, 1);				// LED Off
                NRF_LOG_INFO("%d, LED Off",base_ticks);
            }
            else if(poweroff_ms == 2300)
            {
                nrf_gpio_pin_write(BSP_LED_0, 0);				// LED On
                NRF_LOG_INFO("%d, LED On 3",base_ticks);
            }
            else if(poweroff_ms == 2500)
            {
                nrf_gpio_pin_write(BSP_LED_0, 1);				// LED Off
                NRF_LOG_INFO("%d, LED Off",base_ticks);
            }
            else if(poweroff_ms == 3700)	// 2nd blinking
            {
                nrf_gpio_pin_write(BSP_LED_0, 0);				// LED On
                NRF_LOG_INFO("%d, LED On 4",base_ticks);
            }
            else if(poweroff_ms == 3900)
            {
                nrf_gpio_pin_write(BSP_LED_0, 1);                           // LED Off
                NRF_LOG_INFO("%d, LED Off",base_ticks);
            }
            else if(poweroff_ms == 4100)
            {
                nrf_gpio_pin_write(BSP_LED_0, 0);				// LED On
                NRF_LOG_INFO("%d, LED On 5",base_ticks);
            }
            else if(poweroff_ms == 4300)
            {
                nrf_gpio_pin_write(BSP_LED_0, 1);				// LED Off
                NRF_LOG_INFO("%d, LED Off",base_ticks);
            }
            else if(poweroff_ms == 4500)
            {
                nrf_gpio_pin_write(BSP_LED_0, 0);				// LED On
                NRF_LOG_INFO("%d, LED On 6",base_ticks);
            }
            else if(poweroff_ms == 4700)
            {
                nrf_gpio_pin_write(BSP_LED_0, 1);				// LED Off
                NRF_LOG_INFO("%d, LED Off",base_ticks);
            }
            else if(poweroff_ms == 5700)	// 3rd blinking
            {
                nrf_gpio_pin_write(BSP_LED_0, 0);				// LED On
                NRF_LOG_INFO("%d, LED On 7",base_ticks);
            }
            else if(poweroff_ms == 6100)
            {
                nrf_gpio_pin_write(BSP_LED_0, 1);				// LED Off
                NRF_LOG_INFO("%d, LED Off",base_ticks);
            }
            else if(poweroff_ms == 6300)
            {
                nrf_gpio_pin_write(BSP_LED_0, 0);				// LED On
                NRF_LOG_INFO("%d, LED On 8",base_ticks);
            }
            else if(poweroff_ms == 6500)
            {
                nrf_gpio_pin_write(BSP_LED_0, 1);				// LED Off
                NRF_LOG_INFO("%d, LED Off",base_ticks);
            }
            else if(poweroff_ms == 6700)
            {
                nrf_gpio_pin_write(BSP_LED_0, 0);				// LED On
                NRF_LOG_INFO("%d, LED On 9",base_ticks);
            }
        default:
            break;
    }
}

#ifdef D_USE_LINACC
void reset_prev_lacc(int i) {
    if (i==0) {
        prev_laccx = 0;
        prev_velx = 0;
        prev_dispx = 0;
    } else {
        prev_laccz = 0;
        prev_velz = 0;
        prev_dispz = 0;
    }
}
#endif


static void cursor_mode_timer_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

    uint32_t mask;
    uint32_t pin_state = nrf_gpio_pin_read(TOUCH_PAD_T2_PIN);

    NRF_LOG_INFO("T1: %d", pin_state);

    if (pin_state == 1)
    {
        if (b_left_handed_flag) {
            b_left_handed_flag = false;
            mask = 1;

        } else {
            b_left_handed_flag = true;
            mask = 2;
        }

        ub_cursor_mode_changed_count = 0;

        NRF_LOG_INFO("left_handed_flag: %d", b_left_handed_flag);

        sd_power_gpregret_clr(1, 0xffffffff);
        sd_power_gpregret_set(1, mask);

        NRF_LOG_INFO("reset");

        NVIC_SystemReset();
    }
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_LED_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                LED_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_mouse_move_id,
                                APP_TIMER_MODE_REPEATED,
                                Mouse_movement_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_switch_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                switch_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_cursor_mode_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                cursor_mode_timer_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_MOUSE);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Queued Write Module.
 */
static void qwr_init(void)
{
    ret_code_t         err_code;
    nrf_ble_qwr_init_t qwr_init_obj = {0};

    qwr_init_obj.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Device Information Service.
 */
static void dis_init(void)
{
    ret_code_t       err_code;
    ble_dis_init_t   dis_init_obj;
    ble_dis_pnp_id_t pnp_id;

    pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
    pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
    pnp_id.product_id       = PNP_ID_PRODUCT_ID;
    pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

    memset(&dis_init_obj, 0, sizeof(dis_init_obj));

    ble_srv_ascii_to_utf8(&dis_init_obj.fw_rev_str, C_FIRMWARE_VERSION);
    ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
    dis_init_obj.p_pnp_id = &pnp_id;

    dis_init_obj.dis_char_rd_sec = SEC_JUST_WORKS;

    err_code = ble_dis_init(&dis_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Battery Service.
 */
static void bas_init(void)
{
    ret_code_t     err_code;
    ble_bas_init_t bas_init_obj;

    memset(&bas_init_obj, 0, sizeof(bas_init_obj));

    bas_init_obj.evt_handler          = NULL;
    bas_init_obj.support_notification = true;
    bas_init_obj.p_report_ref         = NULL;
    bas_init_obj.initial_batt_level   = 100;

    bas_init_obj.bl_rd_sec        = SEC_JUST_WORKS;
    bas_init_obj.bl_cccd_wr_sec   = SEC_JUST_WORKS;
    bas_init_obj.bl_report_rd_sec = SEC_JUST_WORKS;

    err_code = ble_bas_init(&m_bas, &bas_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing HID Service.
 */
static void hids_init(void)
{
    ret_code_t                err_code;
    ble_hids_init_t           hids_init_obj;
    ble_hids_inp_rep_init_t * p_input_report;
    uint8_t                   hid_info_flags;

    static ble_hids_inp_rep_init_t inp_rep_array[INPUT_REPORT_COUNT];
    static uint8_t rep_map_data[] =
    {
        0x05, 0x01, // Usage Page (Generic Desktop)
        0x09, 0x02, // Usage (Mouse)

        0xA1, 0x01, // Collection (Application)

        // Report ID 1: Mouse buttons + scroll/pan
        0x85, 0x01,       // Report Id 1
        0x09, 0x01,       // Usage (Pointer)
        0xA1, 0x00,       // Collection (Physical)
        0x95, 0x05,       // Report Count (5)
        0x75, 0x01,       // Report Size (1)
        0x05, 0x09,       // Usage Page (Buttons)
        0x19, 0x01,       // Usage Minimum (01)
        0x29, 0x05,       // Usage Maximum (05)
        0x15, 0x00,       // Logical Minimum (0)
        0x25, 0x01,       // Logical Maximum (1)
        0x81, 0x02,       // Input (Data, Variable, Absolute)
        0x95, 0x01,       // Report Count (1)
        0x75, 0x03,       // Report Size (3)
        0x81, 0x01,       // Input (Constant) for padding
        0x75, 0x08,       // Report Size (8)
        0x95, 0x01,       // Report Count (1)
        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x38,       // Usage (Wheel)
        0x15, 0x81,       // Logical Minimum (-127)
        0x25, 0x7F,       // Logical Maximum (127)
        0x81, 0x06,       // Input (Data, Variable, Relative)
        0x05, 0x0C,       // Usage Page (Consumer)
        0x0A, 0x38, 0x02, // Usage (AC Pan)
        0x95, 0x01,       // Report Count (1)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0xC0,             // End Collection (Physical)

        // Report ID 2: Mouse motion
        0x85, 0x02,       // Report Id 2
        0x09, 0x01,       // Usage (Pointer)
        0xA1, 0x00,       // Collection (Physical)
        0x75, 0x0C,       // Report Size (12)
        0x95, 0x02,       // Report Count (2)
        0x05, 0x01,       // Usage Page (Generic Desktop)
        0x09, 0x30,       // Usage (X)
        0x09, 0x31,       // Usage (Y)
        0x16, 0x01, 0xF8, // Logical maximum (2047)
        0x26, 0xFF, 0x07, // Logical minimum (-2047)
        0x81, 0x06,       // Input (Data, Variable, Relative)
        0xC0,             // End Collection (Physical)
        0xC0,             // End Collection (Application)

        // Report ID 3: Advanced buttons
        0x05, 0x0C,       // Usage Page (Consumer)
        0x09, 0x01,       // Usage (Consumer Control)
        0xA1, 0x01,       // Collection (Application)
        0x85, 0x03,       // Report Id (3)
        0x15, 0x00,       // Logical minimum (0)
        0x25, 0x01,       // Logical maximum (1)
        0x75, 0x01,       // Report Size (1)
        0x95, 0x01,       // Report Count (1)

        0x09, 0xCD,       // Usage (Play/Pause)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x83, 0x01, // Usage (AL Consumer Control Configuration)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xB5,       // Usage (Scan Next Track)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xB6,       // Usage (Scan Previous Track)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)

        0x09, 0xEA,       // Usage (Volume Down)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x09, 0xE9,       // Usage (Volume Up)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x25, 0x02, // Usage (AC Forward)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0x0A, 0x24, 0x02, // Usage (AC Back)
        0x81, 0x06,       // Input (Data,Value,Relative,Bit Field)
        0xC0              // End Collection
    };

    memset(inp_rep_array, 0, sizeof(inp_rep_array));
    // Initialize HID Service.
    p_input_report                      = &inp_rep_array[INPUT_REP_BUTTONS_INDEX];
    p_input_report->max_len             = INPUT_REP_BUTTONS_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_BUTTONS_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    p_input_report                      = &inp_rep_array[INPUT_REP_MOVEMENT_INDEX];
    p_input_report->max_len             = INPUT_REP_MOVEMENT_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MOVEMENT_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    p_input_report                      = &inp_rep_array[INPUT_REP_MPLAYER_INDEX];
    p_input_report->max_len             = INPUT_REP_MEDIA_PLAYER_LEN;
    p_input_report->rep_ref.report_id   = INPUT_REP_REF_MPLAYER_ID;
    p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

    p_input_report->sec.cccd_wr = SEC_JUST_WORKS;
    p_input_report->sec.wr      = SEC_JUST_WORKS;
    p_input_report->sec.rd      = SEC_JUST_WORKS;

    hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

    memset(&hids_init_obj, 0, sizeof(hids_init_obj));

    hids_init_obj.evt_handler                    = on_hids_evt;
    hids_init_obj.error_handler                  = service_error_handler;
    hids_init_obj.is_kb                          = false;
    hids_init_obj.is_mouse                       = true;
    hids_init_obj.inp_rep_count                  = INPUT_REPORT_COUNT;
    hids_init_obj.p_inp_rep_array                = inp_rep_array;
    hids_init_obj.outp_rep_count                 = 0;
    hids_init_obj.p_outp_rep_array               = NULL;
    hids_init_obj.feature_rep_count              = 0;
    hids_init_obj.p_feature_rep_array            = NULL;
    hids_init_obj.rep_map.data_len               = sizeof(rep_map_data);
    hids_init_obj.rep_map.p_data                 = rep_map_data;
    hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
    hids_init_obj.hid_information.b_country_code = 0;
    hids_init_obj.hid_information.flags          = hid_info_flags;
    hids_init_obj.included_services_count        = 0;
    hids_init_obj.p_included_services_array      = NULL;

    hids_init_obj.rep_map.rd_sec         = SEC_JUST_WORKS;
    hids_init_obj.hid_information.rd_sec = SEC_JUST_WORKS;

    hids_init_obj.boot_mouse_inp_rep_sec.cccd_wr = SEC_JUST_WORKS;
    hids_init_obj.boot_mouse_inp_rep_sec.wr      = SEC_JUST_WORKS;
    hids_init_obj.boot_mouse_inp_rep_sec.rd      = SEC_JUST_WORKS;

    hids_init_obj.protocol_mode_rd_sec = SEC_JUST_WORKS;
    hids_init_obj.protocol_mode_wr_sec = SEC_JUST_WORKS;
    hids_init_obj.ctrl_point_wr_sec    = SEC_JUST_WORKS;

    err_code = ble_hids_init(&m_hids, &hids_init_obj);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t err_code;
    ble_dfu_buttonless_init_t dfus_init = {0};

    qwr_init();
    dis_init();
    bas_init();
    hids_init();

    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the battery sensor simulator.
 */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAM_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void timers_start(void)
{
    ret_code_t err_code;

    err_code = app_timer_start(m_LED_timer_id, APP_TIMER_TICKS(LED_TOGGLE_INTERVAL), NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_mouse_move_id, APP_TIMER_TICKS(MOUSE_MOVE_INTERVAL), NULL);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_switch_timer_id, APP_TIMER_TICKS(SWITCH_INTERVAL), NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    nrf_gpio_pin_write(BSP_LED_0, 1);  // LED OFF

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

#ifdef WOM_ENABLED
    // Prepare wakeup pin
    nrf_gpio_cfg_sense_set(IMU_INT_PIN, NRF_GPIO_PIN_SENSE_LOW);
#endif

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling HID events.
 *
 * @details This function will be called for all HID events which are passed to the application.
 *
 * @param[in]   p_hids  HID service structure.
 * @param[in]   p_evt   Event received from the HID service.
 */
static void on_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
            NRF_LOG_INFO("BLE_HIDS_EVT_BOOT_MODE_ENTERED");
            m_in_boot_mode = true;
            break;

        case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
            NRF_LOG_INFO("BLE_HIDS_EVT_REPORT_MODE_ENTERED");
            m_in_boot_mode = false;
            break;

        case BLE_HIDS_EVT_NOTIF_ENABLED:
            NRF_LOG_INFO("BLE_HIDS_EVT_NOTIF_ENABLED");
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
            NRF_LOG_INFO("%d Directed advertising.", base_ticks);
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("%d Fast advertising.", base_ticks);
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW:
            NRF_LOG_INFO("%d Slow advertising.", base_ticks);
#if SWIFT_PAIR_SUPPORTED == 1
            m_sp_advdata.p_manuf_specific_data = NULL;
            err_code = ble_advertising_advdata_update(&m_advertising, &m_sp_advdata, NULL);
            APP_ERROR_CHECK(err_code);
#endif
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_FAST_WHITELIST:
            NRF_LOG_INFO("%d Fast advertising with whitelist.", base_ticks);
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_SLOW_WHITELIST:
            NRF_LOG_INFO("%d Slow advertising with whitelist.", base_ticks);
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
            APP_ERROR_CHECK(err_code);
            err_code = ble_advertising_restart_without_whitelist(&m_advertising);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            NRF_LOG_INFO("%d BLE_ADV_EVT_IDLE", base_ticks);
            err_code = bsp_indication_set(BSP_INDICATE_IDLE);
            APP_ERROR_CHECK(err_code);

#ifdef NO_SLEEP
            break;
#endif
            /* Maybe timers disturb to enter sleep, so stop it before sleep */
            app_timer_stop(m_LED_timer_id);
            app_timer_stop(m_switch_timer_id);
            app_timer_stop(m_mouse_move_id);
#ifdef WOM_ENABLED
            configure_wom();
#else
            ICM20948_lowPower(false);

            /* MPU09250 enters sleep before System OFF */
            uint32_t err_code = app_mpu_set_sleep_mode(true);
            APP_ERROR_CHECK(err_code);
#endif

#ifdef D_USE_MADGWICK
            /* Power down AK09916 */
            uint32_t powerdownAK09916(void);
            powerdownAK09916();
#endif

#if DRIFT_FIX == 1
            save_bias(gyro_bias[0], gyro_bias[2]);
#endif
            sleep_mode_enter();
            break;

        case BLE_ADV_EVT_WHITELIST_REQUEST:
        {
            ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
            uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
            uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

            err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
                                        whitelist_irks,  &irk_cnt);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
                           addr_cnt,
                           irk_cnt);

            // Set the correct identities list (no excluding peers with no Central Address Resolution).
            identities_set(PM_PEER_ID_LIST_SKIP_NO_IRK);

            // Apply the whitelist.
            err_code = ble_advertising_whitelist_reply(&m_advertising,
                                                       whitelist_addrs,
                                                       addr_cnt,
                                                       whitelist_irks,
                                                       irk_cnt);
            APP_ERROR_CHECK(err_code);
        }
        break;

        case BLE_ADV_EVT_PEER_ADDR_REQUEST:
        {
            pm_peer_data_bonding_t peer_bonding_data;

            // Only Give peer address if we have a handle to the bonded peer.
            if (m_peer_id != PM_PEER_ID_INVALID)
            {

                err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
                if (err_code != NRF_ERROR_NOT_FOUND)
                {
                    APP_ERROR_CHECK(err_code);

                    // Manipulate identities to exclude peers with no Central Address Resolution.
                    identities_set(PM_PEER_ID_LIST_SKIP_ALL);

                    ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
                    err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
        }

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("%d Connected", base_ticks);
//            pm_handler_secure_on_connection(p_ble_evt);
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);

            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("%d Disconnected", base_ticks);
            // LED indication will be changed when advertising starts.

            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            m_connected = 0;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_INFO("%d PHY update request.", base_ticks);
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_INFO("%d GATT Client Timeout.", base_ticks);
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_INFO("%d GATT Server Timeout.", base_ticks);
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

		case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            NRF_LOG_INFO("%d BLE_GAP_EVT_CONN_PARAM_UPDATE", base_ticks);
			break;

		case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_INFO("%d BLE_GAP_EVT_SEC_PARAMS_REQUEST", base_ticks);
            break;

		case BLE_GAP_EVT_AUTH_STATUS:
			NRF_LOG_INFO("%d BLE_GAP_EVT_AUTH_STATUS in pm_evt_handler", base_ticks);
			break;

		case BLE_GAP_EVT_SEC_INFO_REQUEST:
            NRF_LOG_INFO("%d BLE_GAP_EVT_SEC_INFO_REQUEST", base_ticks);
			break;

		case BLE_GAP_EVT_SEC_REQUEST:
            NRF_LOG_INFO("%d BLE_GAP_EVT_SEC_REQUEST", base_ticks);
			break;

        default:
//            NRF_LOG_INFO("BLE_GAP_EVT_Other, %d", p_ble_evt->header.evt_id);
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
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


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ret_code_t             err_code;
    uint8_t                adv_flags;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    adv_flags                            = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = adv_flags;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;
#if SWIFT_PAIR_SUPPORTED == 1
    init.advdata.p_manuf_specific_data = &m_sp_manuf_advdata;
    memcpy(&m_sp_advdata, &init.advdata, sizeof(m_sp_advdata));
#endif

    init.config.ble_adv_whitelist_enabled          = true;
    init.config.ble_adv_directed_high_duty_enabled = true;
    init.config.ble_adv_directed_enabled           = false;
    init.config.ble_adv_directed_interval          = 0;
    init.config.ble_adv_directed_timeout           = 0;
    init.config.ble_adv_fast_enabled               = true;
    init.config.ble_adv_fast_interval              = APP_ADV_FAST_INTERVAL;
    init.config.ble_adv_fast_timeout               = APP_ADV_FAST_DURATION;
    init.config.ble_adv_slow_enabled               = true;
    init.config.ble_adv_slow_interval              = APP_ADV_SLOW_INTERVAL;
    init.config.ble_adv_slow_timeout               = APP_ADV_SLOW_DURATION;

    init.evt_handler   = on_adv_evt;
    init.error_handler = ble_advertising_error_handler;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for sending a Mouse Movement.
 *
 * @param[in]   x_delta   Horizontal movement.
 * @param[in]   y_delta   Vertical movement.
 */
static void mouse_movement_send(int16_t x_delta, int16_t y_delta)
{
    ret_code_t err_code;

    if (m_in_boot_mode)
    {
        x_delta = MIN(x_delta, 0x00ff);
        y_delta = MIN(y_delta, 0x00ff);

        err_code = ble_hids_boot_mouse_inp_rep_send(&m_hids,
                                                    0x00,
                                                    (int8_t)x_delta,
                                                    (int8_t)y_delta,
                                                    0,
                                                    NULL,
                                                    m_conn_handle);
    }
    else
    {
        uint8_t buffer[INPUT_REP_MOVEMENT_LEN];

        APP_ERROR_CHECK_BOOL(INPUT_REP_MOVEMENT_LEN == 3);

        x_delta = MIN(x_delta, 0x0fff);
        y_delta = MIN(y_delta, 0x0fff);

        buffer[0] = x_delta & 0x00ff;
        buffer[1] = ((y_delta & 0x000f) << 4) | ((x_delta & 0x0f00) >> 8);
        buffer[2] = (y_delta & 0x0ff0) >> 4;

        err_code = ble_hids_inp_rep_send(&m_hids,
                                         INPUT_REP_MOVEMENT_INDEX,
                                         INPUT_REP_MOVEMENT_LEN,
                                         buffer,
                                         m_conn_handle);
    }

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


//@brief Function for sending a Mouse Button.
static void mouse_button_send(uint8_t button, uint8_t scroll, uint8_t sideways)
{
    ret_code_t err_code;

//    if (m_in_boot_mode)
    {
	//return;
    }
//    else
    {
        uint8_t buffer[INPUT_REP_BUTTONS_LEN];

	APP_ERROR_CHECK_BOOL(INPUT_REP_BUTTONS_LEN == 3);

	buffer[0] = button;		// Left button (bit 0) pressed
	buffer[1] = scroll;		// Scroll value (-127, 128)
	buffer[2] = sideways;           // Sideways scroll value (-127, 128)

	err_code = ble_hids_inp_rep_send(&m_hids,
					INPUT_REP_BUTTONS_INDEX,
					INPUT_REP_BUTTONS_LEN,
					buffer,
                                        m_conn_handle);
    }

    if ((err_code != NRF_SUCCESS) &&
	(err_code != NRF_ERROR_INVALID_STATE) &&
	(err_code != NRF_ERROR_RESOURCES) &&
	(err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING))
    {
        APP_ERROR_HANDLER(err_code);
    }
}


void move_cursor_back(void)
{
    int deltax=0, deltay=0, remain = 2; // remain = 2 @ 2025.12.05
    int pos = c_index;

    while (remain-- > 0) {
        pos--;
        if (pos < 0)
            pos = SIZE_BUFFER - 1;
        deltax -= dx[pos];
        deltay -= dy[pos];
    }

    if (deltax != 0 || deltay != 0) {
        mouse_movement_send(deltax, deltay);
    }
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;
   
    switch (event)
    {
        case BSP_EVENT_DISCONNECT:
            NRF_LOG_INFO("[%d] BSP_EVENT_DISCONNECT",base_ticks);

            m_advertising.adv_modes_config.ble_adv_on_disconnect_disabled = true;
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            NRF_LOG_INFO("[%d] BSP_EVENT_WHITELIST_OFF: m_conn_handle:%d", base_ticks, m_conn_handle);
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        case BSP_EVENT_KEY_0:
            if (m_conn_handle != BLE_CONN_HANDLE_INVALID && m_connected == 1)
            {
                // NRF_LOG_INFO("[%d] sleep_mode_enter",base_ticks);
                // sleep_mode_enter();
            }
            break;

        case BSP_EVENT_KEY_4:	// Right button

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID && m_connected == 1)
            {
                NRF_LOG_INFO("rb push");

                ul_no_mouse_action_count = 0;

                if (nrf_gpio_pin_read(CLICK_L_PIN) == 0)	// if pushed
                {
                    NRF_LOG_INFO("T2: L is firstly pushed, break");
                    break;
                }

                if (b_move_pushed)
                {
                    app_timer_start(m_cursor_mode_timer, APP_TIMER_TICKS(CURSOR_MODE_TIMEOUT), NULL);

                    NRF_LOG_INFO("cursor_mode_timer start");
                }
                else
                {
                    mouse_button_send(0x02, 0x00, 0x00); /* right push */
                    b_mouse_movement_flag = true;
                    mouse_movement_skip300ms_counter = 0;
                }

                if (skilled_mode) {
#ifdef SKILLED_MODE_CLICK_MIN_DELAY
                    mouse_movement_skip300ms_counter = 13;
#else
                    mouse_movement_skip300ms_counter = 0;
#endif
                }
            } 
            break;

        case BSP_EVENT_KEY_4_RELEASE :

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID && m_connected == 1)
            {
                NRF_LOG_INFO("rb release");

                mouse_button_send(0x00, 0x00, 0x00);  /* right release */
                fine_scale_flag = false;
                b_mouse_movement_flag = false;

                app_timer_stop(m_cursor_mode_timer);
            }
            break;

        case BSP_EVENT_KEY_3:	// left button

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID && m_connected == 1)
            {
                NRF_LOG_INFO("LB push");

                if (((app_timer_t *)m_cursor_mode_timer)->end_val != APP_TIMER_IDLE_VAL)
                {
                    //app_timer_stop(m_cursor_mode_timer);

                    //NRF_LOG_INFO("cursor_mode_timer stop");
                    break;
                }

                if (b_scroll_flag) {
                    break;
                }

                /* Normal push */
                ul_no_mouse_action_count = 0;

                mouse_button_send(0x01, 0x00, 0x00);  /* left push */
                b_mouse_movement_flag = true;

                mouse_movement_skip300ms_counter = 6;

                if (skilled_mode) {
#ifdef SKILLED_MODE_CLICK_MIN_DELAY
                    mouse_movement_skip300ms_counter = 13;
#else
                    mouse_movement_skip300ms_counter = 0;
#endif
                }
            }
            break;

        case BSP_EVENT_KEY_3_RELEASE:

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID && m_connected == 1)
            {
                mouse_button_send(0x00, 0x00, 0x00);  /* left release */

                NRF_LOG_INFO("Lb release");

                fine_scale_flag = false;
                b_mouse_movement_flag = false;
#ifdef D_USE_LINACC
                reset_prev_lacc(0);
                reset_prev_lacc(1);
#endif

                if (skilled_mode) {
#ifdef SKILLED_MODE_CLICK_MIN_DELAY
                    mouse_movement_skip300ms_counter = 20;
#else
                    mouse_movement_skip300ms_counter = 0;
#endif
                }
            }
            break;

        case BSP_EVENT_KEY_2:	// move button

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID && m_connected == 1)
            {
                bool switch_mode = true;
                b_move_pushed = true;

                for (int i=3; i>0; i--) {
                    t2_timestamps[i] = t2_timestamps[i-1];
                }
                t2_timestamps[0] = get_now();

                for (int i=0; i<3; i++) {
                    if  ((t2_timestamps[i] - t2_timestamps[i+1]) > APP_TIMER_TICKS(300)) {
                        switch_mode = false;
                        break;
                    } 
                }

                if (switch_mode) {
                    if (skilled_mode == S_MODE_NONE) {
                        skilled_mode = S_MODE_1;
                    } else if (skilled_mode == S_MODE_1) {
                        skilled_mode = S_MODE_2;
                        b_mouse_movement_flag_2 = true;
                    } else {
                        skilled_mode = S_MODE_NONE;
                        b_mouse_movement_flag_2 = false;
                    }
                    NRF_LOG_INFO("skilled:%d", skilled_mode);
                    ub_cursor_mode_changed_count = 0;
                }

                ul_no_mouse_action_count = 0;
                b_mouse_movement_flag = true;
                if (skilled_mode == S_MODE_1) {
                    b_mouse_movement_flag_2 = true;
                } else if (skilled_mode == S_MODE_2) {
                    b_hold_cursor_flag = true;
                }
                mouse_movement_skip300ms_counter = 18;

                NRF_LOG_INFO("mb push");
            }
            break;
            
        case BSP_EVENT_KEY_2_RELEASE:

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID && m_connected == 1)
            {
                fine_scale_flag = false;
                b_mouse_movement_flag = false;

                if (skilled_mode == S_MODE_NONE)
                    move_cursor_back();

                b_move_pushed = false;

                if (skilled_mode == S_MODE_2) {
                    b_hold_cursor_flag = false;
                }
#ifdef D_USE_LINACC
                reset_prev_lacc(0);
                reset_prev_lacc(1);
#endif
                NRF_LOG_INFO("mb release");
            }
            break;

       case BSP_EVENT_KEY_1:	// scroll button

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID && m_connected == 1)
            {
                NRF_LOG_INFO("sb push");
                ul_no_mouse_action_count = 0;

                b_scroll_flag = true;
                back_forward_state = 0;
                skip_scroll_after_backforward_count = 0;
                scroll_count = 0;

                //below 7 variable are for scroll, backforward function
                IMU_input_counts = 0;
                accum_dx = 0;
                accum_dy = 0;
                bRatio_calculate_once = true;                
                bScrollActivated = false;
                bScrollAction = false;
                bBackforwardAction = false;
                ratio = 0;                

                if (skilled_mode) {
                    mouse_movement_skip300ms_counter = 0;
                }
            } 
            break;

        case BSP_EVENT_KEY_1_RELEASE :

            if (m_conn_handle != BLE_CONN_HANDLE_INVALID && m_connected == 1)
            {
                NRF_LOG_INFO("sb release");
                app_timer_stop(m_cursor_mode_timer);

                b_scroll_flag = false;
            }
            break;

        default:
            break;
    }

}


#if DRIFT_FIX == 1
int16_t gy[3] = {0};  /* Gyroscope value without adjusting offset */

#define N       10
static int input_array[2][N] = {0};

static int average(int i)
{
    int sum = 0;
    
    for (int k=0; k<N; k++)
    {
        sum += input_array[i][k];
    }
    return sum / N;
}


static int variance(int avg, int i)
{
    int64_t square_sum = 0;
    int *ar = input_array[i];

    for (int k=0; k<N; k++)
    {
        square_sum += (uint64_t)((float)(ar[k] - avg) * (float)(ar[k] - avg));
    }
    return square_sum / N;
}


static void drift_fix_1(int16_t gy0, int16_t gy2)
{
    static int num_input = 0;
    static int avg0, avg2, var0, var2;

    if (num_input < N) {
        input_array[0][num_input] = gy0;
        input_array[1][num_input] = gy2;
    } else if (num_input == N) {

        avg0 = average(0);
        avg2 = average(1);
        var0 = variance(avg0, 0);
        var2 = variance(avg2, 1);

 /*       if (0 <= var0 && var0 < 6400) {
            if (-8192 < avg0 && avg0 < 8192) {
                gyro_bias[0] = avg0;
            }
        }
        if (0 <= var2 && var2 < 6400) {
            if (-8192 < avg2 && avg2 < 8192) {
                gyro_bias[2] = avg2;
            }
        }     */
        if (0 <= var0 && var0 < 3000) {
            if (-4000 < avg0 && avg0 < 4000) {
                gyro_bias[0] = avg0;
            }
        }
        if (0 <= var2 && var2 < 3000) {
            if (-4000 < avg2 && avg2 < 4000) {
                gyro_bias[2] = avg2;
            }
        } 
    }

    if (num_input < 200) {  /* 200 * 15ms = 3,000ms */
        num_input++;
    } else {
        num_input = 0;
    }

#ifdef DEBUG
    int num_input_log;
    if (num_input == 0) num_input_log = 101;
    else num_input_log = num_input;

    //NRF_LOG_INFO(", %6d, avg0: %6d, var0: %6d, offset: %6d, corr: %6d, %6d",
    //        gy[0], avg0, var0, (int)gyro_bias[0], (int)gyro[0], num_input_log);
#endif
}


#define RETENTION_ADDR      ((volatile uint32_t *)0x2000C000)
#define MEMORY_SAVED_WORD    0xFEEDBEEFUL

void save_bias(int gbias0, int gbias2)
{
    NRF_LOG_INFO("save gyro_bias: %d, %d, %x", gbias0, gbias2, MEMORY_SAVED_WORD);

    sd_power_ram_power_set(6, (POWER_RAM_POWER_S0RETENTION_On << POWER_RAM_POWER_S0RETENTION_Pos));
            
    *(RETENTION_ADDR+0) = (uint32_t)gbias0;
    *(RETENTION_ADDR+1) = (uint32_t)gbias2;
    *(RETENTION_ADDR+2) = MEMORY_SAVED_WORD;
}


bool load_bias(float *gbias0, float *gbias2)
{
    bool ret;
    uint32_t saved = (uint32_t)(*(RETENTION_ADDR+2));

    if (saved == MEMORY_SAVED_WORD)
    {
        *gbias0 = (float)(int)(*(RETENTION_ADDR+0));
        *gbias2 = (float)(int)(*(RETENTION_ADDR+1));

        ret = true;
        NRF_LOG_INFO("load gyro_bias: %d, %d, %x", (int)*gbias0, (int)*gbias2, saved);
    }
    else
    {
        ret = false;
        NRF_LOG_INFO("invalid gyro_bias: %x", saved);
    }
    return ret;
}
#endif


bool back_or_forward_action(int32_t accum_dx)
{
  bool sent = false;
  NRF_LOG_INFO("back_or_forward_action accum_dx : %d", accum_dx);

  if (accum_dx > 20)
  {
            mouse_button_send(0x10, 0, 0);
            mouse_button_send(0, 0, 0);
            back_forward_state = 2;
            sent = true;
            NRF_LOG_INFO("forward");
  }

  else if(accum_dx < -20)
  {
            mouse_button_send(0x08, 0, 0);
            mouse_button_send(0, 0, 0);
            back_forward_state = 1;
            sent = true;
            NRF_LOG_INFO("back");
  }

  return sent;
}

bool scroll_action(float d_as)
{
    static int accumulated_scroll = 0;
    int data;
    int8_t unit;
    bool sent = false;

    data = d_as * SCROLL_SCALE_CONSTANT;

    if (data == 1 || data == -1) {
        accumulated_scroll += data;
        //NRF_LOG_INFO("%5d %5d", accumulated_scroll, data);
    } else { 
        accumulated_scroll += (int)(data*0.5f);
        //NRF_LOG_INFO("%5d %5d %5d", accumulated_scroll, data, (int)(data*0.5));
    }

    if (accumulated_scroll <= -SCROLL_THRESHOLD || SCROLL_THRESHOLD <= accumulated_scroll) {

        unit = accumulated_scroll > 0 ? -1 : 1;

        if (b_left_handed_flag) {
            unit *= -1;
        }

        mouse_button_send(0x00, unit, 0x00);
        sent = true;
        //NRF_LOG_INFO("sending1... %d", unit);

        accumulated_scroll = 0;
    }
    return sent;
}


static void Mouse_movement_handler(void *p_context)
{
    UNUSED_PARAMETER(p_context);

    static double filteredDeltaAngle2, filteredDeltaAngle0;
    static float filteredDeltaAngleScroll = 0;

    double factor_x, factor_y;   
    int abs_x, abs_y;

    double radian_2, radian_0, tan_2, tan_0;
    double deltaAngle2, deltaAngle1, deltaAngle0;
    int16_t raw_acc_gyro_data[7];

    float w_rx, w_rz, w_mx, w_mz;

    static uint64_t old_time = 0;
    static bool first_run = 0;

#ifdef D_USE_LINACC
    float tmp_laccx, tmp_laccz, velx, velz;
#endif
    int16_t deltax = 0;
    int16_t deltay = 0;

#ifdef D_USE_WDT
     nrf_drv_wdt_channel_feed(m_channel_id);
#endif

    uint64_t now = get_now();

    if (first_run) {
        deltaT = (double)(now - old_time)/16384;
    } else {
        deltaT = MOUSE_MOVE_INTERVAL*0.001;
        first_run = 1;
    }
    old_time = now;

    if(m_connected == 1 && m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        if (mouse_movement_skip300ms_counter < 20) {
            mouse_movement_skip300ms_counter++;
            return;
        }

        // update accel gyro 
        read_accel_gyro(raw_acc_gyro_data);

#ifdef D_USE_COMPFLT
        accel[0] = (float)raw_acc_gyro_data[0] - acc_bias[0];
        accel[1] = (float)raw_acc_gyro_data[1] - acc_bias[1];
        accel[2] = (float)raw_acc_gyro_data[2] - acc_bias[2];

        gyro[0] = (float)raw_acc_gyro_data[3] - gyro_bias[0];
        gyro[1] = (float)raw_acc_gyro_data[4] - gyro_bias[1];
        gyro[2] = (float)raw_acc_gyro_data[5] - gyro_bias[2];

#if DRIFT_FIX == 1
        gy[0] = (float)raw_acc_gyro_data[3];
        gy[2] = (float)raw_acc_gyro_data[5];

        drift_fix_1(gy[0], gy[2]);
#endif

        w_mz = gyro[2];
        w_mx = gyro[0];

        w_rx = (double)w_mz*sin(DEG_TO_RAD*TILT_ANGLE) + (double)w_mx*cos(DEG_TO_RAD*TILT_ANGLE);
        w_rz = (double)w_mz*cos(DEG_TO_RAD*TILT_ANGLE) - (double)w_mx*sin(DEG_TO_RAD*TILT_ANGLE);

        //compFilter(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], deltaT, angles, angleAccels);
        compFilter(accel[0], accel[1], accel[2], w_rx, gyro[1], w_rz, deltaT, angles, angleAccels);

        deltaAngle2 = angles[2] - prevAngle2;
        deltaAngle1 = angles[1] - prevAngle1;
        deltaAngle0 = angles[0] - prevAngle0;

        prevAngle2 = angles[2];
        prevAngle1 = angles[1];
        prevAngle0 = angles[0];

        deltaAngle2 = 0.9*deltaAngle2 + 0.1*deltaAngle1;

        filteredDeltaAngle2 = 0.7 * filteredDeltaAngle2 + 0.3 * deltaAngle2;
        filteredDeltaAngle0 = 0.7 * filteredDeltaAngle0 + 0.3 * deltaAngle0;
        filteredDeltaAngleScroll = filteredDeltaAngle0;

        if (skilled_mode == S_MODE_1 && !b_mouse_movement_flag_2) {
            if (filteredDeltaAngle0 < -2.5) {
                b_mouse_movement_flag_2 = true;
                filteredDeltaAngle2 = 0;
                filteredDeltaAngle0 = 0;
                mouse_movement_skip300ms_counter = 0;
                return;
            }
        }

        if (fabsf((float)filteredDeltaAngle2) > 0.15 || fabsf((float)filteredDeltaAngle0) > 0.15)
            ul_no_mouse_action_count = 0;

        if (b_mouse_movement_flag ||  b_mouse_movement_flag_2) 
        {
            radian_2 = filteredDeltaAngle2 * DEG_TO_RAD;
            radian_0 = filteredDeltaAngle0 * DEG_TO_RAD;
            tan_2 = tan(radian_2);
            tan_0 = tan(radian_0);

            deltax = tan_2 * NORMAL_SCALE_X;
            deltay = tan_0 * NORMAL_SCALE_Y;
/*
            fine_scale_flag = false; // NO FINE SCALING !!!!!!!!!!!!!!

            if (fine_scale_flag) {
                if (-12 > deltax || deltax > 12 || -10 > deltay || deltay > 10) {
                    fine_scale_flag = false;

                     //NRF_LOG_INFO("fine scale -> false");
                }
            } else {
                if (-10 <= deltax && deltax <=10 && -6 <= deltay && deltay <=6) {
                    if (++small_value_counter >= 40) {  // 40 x 15ms = about 0.6 second 
                        fine_scale_flag = true;
                        small_value_counter = 0;

                        //NRF_LOG_INFO("fine scale -> true");
                    }
                } else {
                    small_value_counter = 0;
                }
            }

            // Fine Scale factor
            if (fine_scale_flag) {
                 // scale factor 
                 // if abs(x) is 0 ~ 1, then factor_x is 1 *
                 // if abs(x) is 2, then factor_x is 0.5 *
                 // if abs(x) is 3 ~, then factor_x is 0.9 *
                 // if abs(y) is 0 ~ 1, then factor_y is 1 *
                 // if abs(y) is 2, then factor_y is 0.5 *
                 // if abs(y) is 3 ~, then factor_y is 0.9 *
                 
                abs_x = abs(deltax);
                abs_y = abs(deltay);

                if (abs_x <= 1) {
                    factor_x = 1;
                } else if (abs_x <= 2) {
                    factor_x = 0.5;
                } else {
                    factor_x = 0.9;
                }

                if (abs_y <=1) {
                    factor_y = 1;
                } else if (abs_y <= 2) {
                    factor_y = 0.5;
                } else {
                    factor_y = 0.9;
                }
                factor_x = 1; // NO FINE SCALING !!!!!!!!!!!!!!
                factor_y = 1; // NO FINE SCALING !!!!!!!!!!!!!!
                deltax = (double)deltax * factor_x, deltay = (double)deltay * factor_y;
                
            }
            */
        }
#endif /* D_USE_COMPFLT */

#ifdef D_USE_MADGWICK
        app_mpu_read_magnetometer(&magn_values);

        accel[0] = (float)raw_acc_gyro_data[0] * acc_resolution;
        accel[1] = (float)raw_acc_gyro_data[1] * acc_resolution;
        accel[2] = (float)raw_acc_gyro_data[2] * acc_resolution;

        gyro[0] = (float)raw_acc_gyro_data[3] * gyro_resolution;
        gyro[1] = (float)raw_acc_gyro_data[4] * gyro_resolution;
        gyro[2] = (float)raw_acc_gyro_data[5] * gyro_resolution;

        //bias_to_current_bits = mag_resolution / get_mag_resolution(MAG_OUTPUT_BITS::M16BITS);
        magn[0] = (float)(magn_values.x * mag_resolution * mag_bias_factory[0] - mag_bias[0]) * mag_scale[0];
        magn[1] = (float)(magn_values.y * mag_resolution * mag_bias_factory[1] - mag_bias[1]) * mag_scale[1];
        magn[2] = (float)(magn_values.z * mag_resolution * mag_bias_factory[2] - mag_bias[2]) * mag_scale[2];

        an = -accel[0];
        ae = +accel[1];
        ad = +accel[2];
        gn = +gyro[0] * 0.0174532925;
        ge = -gyro[1] * 0.0174532925;
        gd = -gyro[2] * 0.0174532925;
        mn = +magn[1];
        me = -magn[0];
        md = +magn[2];

        madgwick(an, ae, ad, gn, ge, gd, mn, me, md, q);
        a31 = 2.0f * (q[0] * q[1] + q[2] * q[3]);
        a32 = 2.0f * (q[1] * q[3] - q[0] * q[2]);
        a33 = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

        //lin_acc[0] = a[0] + a31;
        //lin_acc[1] = a[1] + a32;
        lin_accel[0] = accel[0] + a32;
        lin_accel[1] = accel[1] - a31;
        lin_accel[2] = accel[2] - a33;
#endif // D_USE_MADGWICK

#ifdef D_USE_LINACC
        if (fabs(lin_accel[0]) < 0.03f) {
            tmp_laccx = 0;
        } else {
            tmp_laccx = lin_accel[0];
        }

        if (fabs(lin_accel[2]) < 0.03f) {
            tmp_laccz = 0;
        } else {
            tmp_laccz = lin_accel[2];
        }

        deltaT = 0.015f;
#endif // D_USE_LINACC

        if (b_mouse_movement_flag || b_scroll_flag || b_mouse_movement_flag_2) 
        {
#ifdef D_USE_LINACC
            velx = prev_velx + 0.5f*(tmp_laccx + prev_laccx)*deltaT;
            velz = prev_velz + 0.5f*(tmp_laccz + prev_laccz)*deltaT;

            prev_laccx = tmp_laccx;
            prev_laccz = tmp_laccz;

            prev_velx = velx;
            prev_velz = velz;

            deltax += -velx*500;
            deltay += velz*500;
            //deltax = -velx*5000;
            //deltay = velz*5000;

            int la_e7 = (int)(tmp_laccz*10000000);
            int v_e7 = (int)(velz*10000000);

            NRF_LOG_INFO(", %5d, %5d", la_e7, v_e7);

#endif  // D_USE_LINACC

            if (b_left_handed_flag) {
                deltay *= -1;
            }

            if (b_scroll_flag && !b_move_pushed) {
              if (IMU_input_counts++ > NumberofAbandonInputs)
              {
                IMU_input_counts = 100;  //fix this varible in order not to overflow

                radian_2 = filteredDeltaAngle2 * DEG_TO_RAD;
                radian_0 = filteredDeltaAngle0 * DEG_TO_RAD;
                tan_2 = tan(radian_2);
                tan_0 = tan(radian_0);

                deltax = tan_2 * NORMAL_SCALE_X;
                deltay = tan_0 * NORMAL_SCALE_Y;

                if(!bScrollActivated){    //링마우스 움직임이 최소 이동거리를 만족하는지 체크
                  accum_dx += deltax;
                  accum_dy += deltay;
                  double distance = sqrt((double)accum_dx * accum_dx + (double)accum_dy * accum_dy);

                  NRF_LOG_INFO("deltax : %d   accum_dx : %d     deltay: %d   accum_dy : %d      distance : %d ",deltax, accum_dx, deltay, accum_dy, distance);
                  if (distance > Criteria_Distance){
                    bScrollActivated = true;
                  }
                }


                if (bScrollActivated)  // 링마우스 움직임이 최소 이동거리를 만족했다면...
                {
                  if(bRatio_calculate_once){
                    if(accum_dx == 0)
                    {
                      bScrollAction = true;
                      bBackforwardAction = false;
                      bRatio_calculate_once = false;
                      ratio = 999999999;
                    }
                    else
                    {
                      ratio = (accum_dy * 100) / accum_dx; // (deltay*100)/deltax

                      if (ratio > Criteria_ScrollCheck_Positive || ratio < Criteria_ScrollCheck_Negative)
                      {
                        bScrollAction = true;
                        bBackforwardAction = false;
                        bRatio_calculate_once = false;
                      }
                      else
                      {
                        bScrollAction = false;
                        bBackforwardAction = true;
                        bRatio_calculate_once = false;
                        accum_dx = 0 ;  // 앞으로 가기, 뒤로가기를 판단하기 위해서 accum_dx 다시 사용하기 위해서 초기화
                      }
                    }
                  }
                    
                  if(bScrollAction){  // 판단결과 스크롤 동작이라면...
                   if (skip_scroll_after_backforward_count-- <= 0 ) 
                   {
                      NRF_LOG_INFO("[Ken] Scroll Action     skip count : %d    ratio : %d", skip_scroll_after_backforward_count, ratio);                    
                      scroll_action(filteredDeltaAngleScroll);                          
                    }
                  }
                  else if(bBackforwardAction)    // 판단결과 앞으로가기/뒤로가기라면...
                  {
                    NRF_LOG_INFO("[Ken] Back_Forward   RATIO : %d", ratio);

                    radian_2 = filteredDeltaAngle2 * DEG_TO_RAD;
                    tan_2 = tan(radian_2);
                    deltax = tan_2 * NORMAL_SCALE_X;
                    accum_dx += deltax;
                
                    if (back_or_forward_action(accum_dx)) 
                    {
                          skip_scroll_after_backforward_count = 40;
                          bBackforwardAction = false;    // 앞으로가기 / 뒤로가기 한번 수행하고 나면, 더 이상 동작하지 않도록 함.
                    }
                  }
                }
              }
            }

            else {
                dx[c_index] = deltax;
                dy[c_index] = deltay;
                c_index++;
                if (c_index >= SIZE_BUFFER) {
                    c_index = 0;
                }

                if ((deltax != 0 || deltay != 0) && !b_hold_cursor_flag) {
                    mouse_movement_send(deltax, deltay);
                    ul_no_mouse_action_count = 0;
                } else {
                    ul_no_mouse_action_count++;
                }
            }

        } else {  // if(b_mouse_movement_flag)
            ul_no_mouse_action_count++;
        }   // if(b_mouse_movement_flag) 

        if (skilled_mode == S_MODE_1 && ul_no_mouse_action_count == CURSOR_MOVEMENT_TIMEOUT)
        {
            b_mouse_movement_flag_2 = false;
        }

#ifdef NO_SLEEP
        ul_no_mouse_action_count = 0; /* no sleep */
#endif

        if(ul_no_mouse_action_count == C_NO_MOUSE_ACTION_TIMEOUT)
        {
            uint32_t mask = 0;
            
            /* Maybe timers disturb to enter sleep, so stop it before sleep */
            app_timer_stop(m_LED_timer_id);
            app_timer_stop(m_switch_timer_id);
            app_timer_stop(m_mouse_move_id);

#ifdef WOM_ENABLED
            NRF_LOG_INFO("configure_wom");
            configure_wom();
#else
            ICM20948_lowPower(false);

            uint32_t err_code = app_mpu_set_sleep_mode(true);
            APP_ERROR_CHECK(err_code);
#endif

#ifdef D_USE_MADGWICK
            uint32_t powerdownAK09916(void);
            powerdownAK09916();
#endif

            if (skilled_mode == S_MODE_1) {
                mask = 3;
            } else if (skilled_mode == S_MODE_2) {
                mask = 4;
            }

            if (mask == 3 || mask == 4) {
                sd_power_gpregret_clr(1, 0xffffffff);
                sd_power_gpregret_set(1, mask);
            }

#if DRIFT_FIX == 1
            save_bias(gyro_bias[0], gyro_bias[2]);
#endif

            NRF_LOG_INFO("%d, Timeout Sleep... Bye~~~",base_ticks);
            sleep_mode_enter();
        }
    }
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event = 0;

    //err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    err_code = bsp_init(BSP_INIT_BUTTONS, bsp_event_handler);

    APP_ERROR_CHECK(err_code);

    //err_code = bsp_btn_ble_init(NULL, &startup_event);
    //APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    app_sched_execute();
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for application main entry.
 */
int main(void)
{
    bool verbose = false;
    bool erase_bonds;
    uint32_t err_code;
    uint32_t tmp;
    
// Initialize.
#ifdef D_USE_EXTERNAL_XTAL
    nrf_drv_clock_init();
    nrf_drv_clock_hfclk_request(NULL);                                  // for HF 32MHz external X-tal
    while(!nrf_drv_clock_hfclk_is_running())                            // Just waiting
    nrf_drv_clock_lfclk_request(NULL);                                  // for LF 32.768kHz external X-tal
    while(!nrf_drv_clock_lfclk_is_running());                           // Just waiting
#endif // D_USE_EXTERNAL_XTAL

    nrf_gpio_cfg_output(BSP_LED_0);					// Set DK LED to Output
    nrf_gpio_pin_write(BSP_LED_0, 0);					// LED On
    nrf_gpio_cfg_output(C_POWER_OFF_PIN_NUM);                           // Set POWER_OFF to Output
    nrf_gpio_pin_write(C_POWER_OFF_PIN_NUM , 0);                        // POWER_OFF Low
    nrf_gpio_cfg_input(C_PUSH_PIN_NUM, NRF_GPIO_PIN_PULLUP);		// Set push button to Input
    nrf_gpio_cfg_input(C_CHARGE_COMPLETE_PIN_NUM, NRF_GPIO_PIN_NOPULL);	// Set Charge_Complete to Input, No pull up
    ub_PowerOffState = POWER_OFF_CHECK_PUSH_BUTTON;			// initialize power off state

    nrf_gpio_cfg_input(IMU_INT_PIN, NRF_GPIO_PIN_NOPULL);

    log_init();

    err_code = ble_dfu_buttonless_async_svci_init();
    //APP_ERROR_CHECK(err_code);

    tmp = nrf_power_resetreas_get();
    //NRF_LOG_INFO("reset reason: %08x", tmp);
    nrf_power_resetreas_clear(tmp);

    if ((tmp & POWER_RESETREAS_SREQ_Msk) || (tmp == 0)) {
        verbose = true;
    }
    setupIMU(verbose);

#if DRIFT_FIX == 1
    {
        float offset0, offset2;
        if (load_bias(&offset0, &offset2))
        {
            gyro_bias[0] = offset0;
            gyro_bias[2] = offset2;
        }
    }
#endif

    tmp = nrf_power_gpregret2_get();
    nrf_power_gpregret2_set(0);
    if (tmp == 1 || tmp == 2) {
        if (tmp == 1) {
            b_left_handed_flag = false;
        } else if (tmp == 2) {
            b_left_handed_flag = true;
        }
        saveCalibration(acc_bias, gyro_bias, mag_bias, mag_scale, b_left_handed_flag);
    } else if (tmp == 3 || tmp == 4) {
        switch (tmp)
        {
        case 3:
          skilled_mode = S_MODE_1;
          break;
        case 4:
          skilled_mode = S_MODE_2;
          b_mouse_movement_flag_2 = true;
          break;
        }
    }

    timers_init();                            /* Timer Init for mouse movement, LED blinking, and switch time */
    buttons_leds_init(&erase_bonds);          /* Button init, switch and touch setting */

    if (nrf_gpio_pin_read(CLICK_L_PIN) == 0) {
        erase_bonds = true;

        for (int i=0; i<5; i++) {
            nrf_delay_ms(300);
            nrf_gpio_pin_toggle(BSP_LED_0);
        }
    }

    power_management_init();                  /* power management init for sleep mode */
#ifdef D_DCDC_ENABLE
    NRF_POWER->DCDCEN = 1;
#endif // D_DCDC_ENABLE
    ble_stack_init();                         /* init setting for BLE function */
    scheduler_init();
    gap_params_init();
    gatt_init();
    advertising_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
    peer_manager_init();
#ifdef D_USE_ADC
    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();
#endif // D_USE_ADC

    // Start execution.
    NRF_LOG_INFO(C_FIRMWARE_VERSION);

#ifdef D_USE_WDT
    //Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();	// This function must be located just before timers_start()
#endif // D_USE_WDT

    timers_start();                                     /* LED Output */
    advertising_start(erase_bonds);                     /* Start BLE Advertising */

    for (;;)
    {
        idle_state_handle();                            /* BLE scheduler execution function, BLE works only when this function is executed */
    }
}
