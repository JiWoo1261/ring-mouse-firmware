

#define	D_USE_EXTERNAL_XTAL
#define	D_USE_ADC
//#define	D_ADC_DEBUG
//#define	D_BATTERY_LEVEL_DEBUG
#define	D_USE_WDT
#define	D_CHECK_ALIVE_LED_ACTION	// Blinks LED every 2 seconds to check if mouse is alive. temporary action durinig development
#define	D_TIMEOUT_POWER_OFF		// Powers off when no mouse movement or very low timeout
#define	D_BATTERY_LEVEL_CONTROL		// Display battery level on bluetooth device of windows
#define	D_NEW_BATTERY_LEVEL_CALC
#define	D_DCDC_ENABLE
//#define D_USE_MADGWICK
//#define D_USE_LINACC
#define D_USE_COMPFLT
#define WOM_ENABLED
//#define POLL_125HZ    //use this to change Mouse Polling rate 66Hz --> 125Hz
//#define NO_SLEEP
#define DRIFT_FIX  1
//#define SKILLED_MODE_CLICK_MIN_DELAY

/* Select Full Scale of Accelerometer and Gyroscope */
#define	D_ACCEL_FULL_SCALE	(AFS_2G)               // Accelerometer Configuration, ACCEL_FS_SEL[1:0]
#define	D_GYRO_FULL_SCALE	(GFS_250DPS)           // Gyroscope Configuration, GYRO_FS_SEL[1:0]

