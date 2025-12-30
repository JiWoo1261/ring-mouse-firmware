
#include "feature_config.h"
//#include "mpu9255_register_map.h"
#include "ICM_20948_REGISTERS.h"
#include "Drv_mpu.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "nrf_peripherals.h"
#include "nrf_error.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "common.h"
#include "uicr.h"

#define TWI_INSTANCE_ID                 0
#define TWI_WRITE_LENGTH                20
#define TWI_READ_LENGTH                 20
#define TWI_SLAVE_ADDRESS               MPU_ADDRESS

#define MPU_ADDRESS                     0x68 
#define MPU_AK89XX_MAGN_ADDRESS         0x0C
#define ICM_20948_WHOAMI                0xEA

#ifdef POLL_125HZ
#define SMPLRT_DIV                      8
#else
#define SMPLRT_DIV                      15
#endif

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

static float getAccelResolution(accel_range range);
static float getGyroResolution(gyro_range range);

extern magn_values_t magn_values;

extern float mag_bias[3];
extern float mag_scale[3];
extern float mag_bias_factory[3];
extern float mag_resolution;
extern float acc_resolution;
extern float gyro_resolution;

const double ACC_RES = 2.0/32768.0;
const double GYRO_RES = 250.0/32768.0;

extern bool b_left_handed_flag;


/**
 * @brief Write data to simulated EEPROM.
 *
 * Function uses the TWI interface to write data into EEPROM memory.
 *
 * @param     addr  Start address to write.
 * @param[in] pdata Pointer to data to send.
 * @param     size  Byte count of data to send.
 * @attention       Maximum number of bytes that may be written is @ref EEPROM_SIM_SEQ_WRITE_MAX.
 *                  In sequential write, all data must be in the same page
 *                  (higher address bits do not change).
 *
 * @return NRF_SUCCESS or reason of error.
 *
 * @attention If you wish to communicate with real EEPROM memory chip, check its readiness
 * after writing the data.
 */
static ret_code_t TWI_write(uint16_t addr, uint8_t const * pdata, size_t size)
{
    ret_code_t ret;
    /* Memory device supports only a limited number of bytes written in sequence */
    do
    {
        uint8_t buffer[TWI_WRITE_LENGTH] = {0,}; /* Addr + data */

        memcpy(buffer, &addr, 1);
        memcpy(buffer + 1, pdata, size);
        ret = nrf_drv_twi_tx(&m_twi, MPU_ADDRESS, buffer, size+1 , false);
    }while (0);
    return ret;
}


static ret_code_t TWIM_write(uint16_t addr, uint8_t const * pdata, size_t size)
{
    ret_code_t ret;
    /* Memory device supports only a limited number of bytes written in sequence */
    do
    {
        uint8_t buffer[TWI_WRITE_LENGTH] = {0,}; /* Addr + data */

        memcpy(buffer, &addr, 1);
        memcpy(buffer + 1, pdata, size);
        ret = nrf_drv_twi_tx(&m_twi, MPU_AK89XX_MAGN_ADDRESS, buffer, size+1 , false);
    }while (0);
    return ret;
}

/**
 * @brief Read data from simulated EEPROM.
 *
 * Function uses the TWI interface to read data from EEPROM memory.
 *
 * @param     addr  Start address to read.
 * @param[in] pdata Pointer to the buffer to fill with data.
 * @param     size  Byte count of data to read.
 *
 * @return NRF_SUCCESS or reason of error.
 */
static ret_code_t TWI_read(uint16_t addr, uint8_t * pdata, size_t size)
{
    ret_code_t ret;
    do
    {
       uint16_t addr16 = addr;
       ret = nrf_drv_twi_tx(&m_twi, TWI_SLAVE_ADDRESS, (uint8_t *)&addr16, 1, true);
       if (NRF_SUCCESS != ret)
       {
           break;
       }
       ret = nrf_drv_twi_rx(&m_twi, TWI_SLAVE_ADDRESS, pdata, size);
    }while (0);
    return ret;
}

static ret_code_t TWIM_read(uint16_t addr, uint8_t * pdata, size_t size)
{
    ret_code_t ret;
    do
    {
       uint16_t addr16 = addr;
       ret = nrf_drv_twi_tx(&m_twi, MPU_AK89XX_MAGN_ADDRESS, (uint8_t *)&addr16, 1, true);
       if (NRF_SUCCESS != ret)
       {
           break;
       }
       ret = nrf_drv_twi_rx(&m_twi, MPU_AK89XX_MAGN_ADDRESS, pdata, size);
    }while (0);
    return ret;
}

/**
 * @brief Initialize the master TWI.
 *
 * Function used to initialize the master TWI interface that would communicate with MPU-9250.
 *
 * @return NRF_SUCCESS or the reason of failure.
 */
uint32_t twi_master_init(void)
{
    ret_code_t ret;
    const nrf_drv_twi_config_t config =
    {
       .scl			= 14,
       .sda			= 26,
       .frequency               = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority	= APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init		= false
    };

    ret = nrf_drv_twi_init(&m_twi, &config, NULL, NULL);

    if (NRF_SUCCESS == ret)
    {
        nrf_drv_twi_enable(&m_twi);
    }

    return ret;
}


uint32_t app_mpu_config(app_mpu_config_t * config)
{
    uint8_t *data;
    data = (uint8_t*)config;
    //return nrf_drv_mpu_write_registers(MPU_REG_SMPLRT_DIV, data, 4);

    return TWI_write(MPU_REG_SMPLRT_DIV, data, 4);
}

uint32_t app_mpu_int_cfg_pin(app_mpu_int_pin_cfg_t *cfg)
{
    uint8_t *data;
    data = (uint8_t*)cfg;
    //return nrf_drv_mpu_write_single_register(MPU_REG_INT_PIN_CFG, *data);

    return TWI_write(MPU_REG_INT_PIN_CFG, data, 1);
    
}

uint32_t app_mpu_int_enable(app_mpu_int_enable_t *cfg)
{
    uint8_t *data;
    data = (uint8_t*)cfg;
    //return nrf_drv_mpu_write_single_register(MPU_REG_INT_ENABLE, *data);

    return TWI_write(MPU_REG_INT_ENABLE, data, 1);
}


uint32_t initICM20948(void)
{
    uint32_t err_code;
    uint8_t regTemp = 0, regTemp2[2] = {0};
    int tries = 0;

    regTemp = 0x00;
    err_code = TWI_write(REG_BANK_SEL, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0;
    err_code = TWI_read(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;
    regTemp &= ~(1<<5);
    err_code = TWI_write(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x80;
    err_code = TWI_write(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;
    nrf_delay_ms(10);

    while(tries < 10) {

        regTemp = 0;
        err_code = TWI_read(AGB0_REG_WHO_AM_I, &regTemp, 1);
        if(err_code != NRF_SUCCESS) return err_code;

        NRF_LOG_INFO("Who am I : %x", regTemp);

        if (regTemp == ICM_20948_WHOAMI) {
            break;
        } else {
            regTemp = 0x80;
            err_code = TWI_write(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
            if(err_code != NRF_SUCCESS) return err_code;
            nrf_delay_ms(300);
        }
        tries++;
    }
    if (tries == 10) {
        NRF_LOG_INFO("ICM-20948 is not found");
        return NRF_ERROR_NOT_FOUND;
    }

    regTemp = 0;
    err_code = TWI_read(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;
    regTemp &= ~((1<<6)|(1<<5));
    err_code = TWI_write(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = (0<<6)|(1<<5)|(1<<4);
    err_code = TWI_write(AGB0_REG_LP_CONFIG, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x20;
    err_code = TWI_write(REG_BANK_SEL, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x01;
    err_code = TWI_write(AGB2_REG_ODR_ALIGN_EN, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = SMPLRT_DIV;
    err_code = TWI_write(AGB2_REG_GYRO_SMPLRT_DIV, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp2[0] = SMPLRT_DIV >> 8;
    regTemp2[1] = SMPLRT_DIV & 0xFF;

    err_code = TWI_write(AGB2_REG_ACCEL_SMPLRT_DIV_1, &regTemp2[0], 1);
    if(err_code != NRF_SUCCESS) return err_code;

    err_code = TWI_write(AGB2_REG_ACCEL_SMPLRT_DIV_2, &regTemp2[1], 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = (7<<3)|1;
    err_code = TWI_write(AGB2_REG_ACCEL_CONFIG, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = (6<<3)|1;
    err_code = TWI_write(AGB2_REG_GYRO_CONFIG_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x00;
    err_code = TWI_write(REG_BANK_SEL, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0;
    err_code = TWI_read(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;
    regTemp |= (1<<5);
    err_code = TWI_write(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}

uint32_t ICM20948_lowPower(bool on)
{
    uint32_t err_code;
    uint8_t regTemp = 0;

    regTemp = 0;
    err_code = TWI_read(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;
    if (on) {
      regTemp |= (1<<5);
    } else {
      regTemp &= ~(1<<5);
    }
    err_code = TWI_write(AGB0_REG_PWR_MGMT_1, &regTemp, 1);

    return err_code;
}

uint32_t configure_wom(void)
{
    uint32_t err_code;
    uint8_t regTemp = 0, regTemp2[2] = {0};

    regTemp = 0x00;
    err_code = TWI_write(REG_BANK_SEL, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x80;
    err_code = TWI_write(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;
    nrf_delay_ms(200);

    regTemp = 0;
    err_code = TWI_read(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;
    regTemp &= ~((1<<6)|(1<<5));
    err_code = TWI_write(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x07;
    err_code = TWI_write(AGB0_REG_PWR_MGMT_2, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = (0<<6)|(1<<5)|(1<<4);
    err_code = TWI_write(AGB0_REG_LP_CONFIG, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x20;
    err_code = TWI_write(REG_BANK_SEL, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x01;
    err_code = TWI_write(AGB2_REG_ODR_ALIGN_EN, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp2[0] = 1124 >> 8;   /* MSB */
    regTemp2[1] = 1124 & 0xFF; /* LSB */
    err_code = TWI_write(AGB2_REG_ACCEL_SMPLRT_DIV_1, &regTemp2[0], 1);
    if(err_code != NRF_SUCCESS) return err_code;

    err_code = TWI_write(AGB2_REG_ACCEL_SMPLRT_DIV_2, &regTemp2[1], 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = (7<<3)|1;
    err_code = TWI_write(AGB2_REG_ACCEL_CONFIG, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x00;
    err_code = TWI_write(REG_BANK_SEL, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = (0<<7)|(0<<6)|(1<<5)|(1<<4); 
    err_code = TWI_write(AGB0_REG_INT_PIN_CONFIG, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x20;
    err_code = TWI_write(REG_BANK_SEL, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x03;
    err_code = TWI_write(AGB2_REG_ACCEL_INTEL_CTRL, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 64;
    err_code = TWI_write(AGB2_REG_ACCEL_WOM_THR, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0x00;
    err_code = TWI_write(REG_BANK_SEL, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = (1<<3);
    err_code = TWI_write(AGB0_REG_INT_ENABLE, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    err_code = TWI_read(AGB0_REG_INT_STATUS, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    regTemp = 0;
    err_code = TWI_read(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;
    regTemp |= (1<<5);
    err_code = TWI_write(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
    if(err_code != NRF_SUCCESS) return err_code;

    return NRF_SUCCESS;
}


uint32_t app_mpu_read_temp(temp_value_t * temperature)
{
    uint32_t err_code;
    uint8_t raw_values[2];
    err_code = TWI_read(MPU_REG_TEMP_OUT_H, raw_values, 2);
    if(err_code != NRF_SUCCESS) return err_code;

    *temperature = (temp_value_t)(raw_values[0] << 8) + raw_values[1];

    return NRF_SUCCESS;
}


uint32_t app_mpu_read_int_source(uint8_t * int_source)
{
    //return nrf_drv_mpu_read_registers(MPU_REG_INT_STATUS, int_source, 1);

    return TWI_read(MPU_REG_INT_STATUS, int_source, 1);
}

uint32_t app_mpu_magnetometer_init(app_mpu_magn_config_t * p_magnetometer_conf)
{	
    uint32_t err_code;
    
    // Read out MPU configuration register
    app_mpu_int_pin_cfg_t bypass_config;
    err_code = TWI_read(MPU_REG_INT_PIN_CFG, (uint8_t *)&bypass_config, 1);
	
    // Set I2C bypass enable bit to be able to communicate with magnetometer via I2C
    bypass_config.i2c_bypass_en = 1;
    // Write config value back to MPU config register
    err_code = app_mpu_int_cfg_pin(&bypass_config);
    if (err_code != NRF_SUCCESS) return err_code;
	
    // Write magnetometer config data	
    uint8_t *data;
    data = (uint8_t*)p_magnetometer_conf;	
    return TWIM_write(MPU_AK89XX_REG_CNTL, data, 1);
}

uint32_t app_mpu_read_magnetometer(magn_values_t * p_magnetometer_values)
{
    uint8_t status_1_reg;
    uint8_t raw_data[7];
    uint32_t err_code;

    err_code = TWIM_read(MPU_AK89XX_REG_ST1, &status_1_reg, 1);
    // Read the six raw data and ST2 registers sequentially into data array
    err_code = TWIM_read(MPU_AK89XX_REG_HXL, raw_data, 7);
    if(err_code != NRF_SUCCESS) return err_code;

    //uint8_t c = raw_data[6];    // End data read by reading ST2 register
    //if (!(c & 0x08)) {          // Check if magnetic sensor overflow set, if not then report data
        p_magnetometer_values->x = ((int16_t)raw_data[1] << 8) | raw_data[0];
        p_magnetometer_values->y = ((int16_t)raw_data[3] << 8) | raw_data[2];
        p_magnetometer_values->z = ((int16_t)raw_data[5] << 8) | raw_data[4];
    //}
        
    /* Quote from datasheet: MPU_AK89XX_REG_ST2 register has a role as data reading end register, 
    also. When any of measurement data register is read in continuous measurement mode or 
    external trigger measurement mode, it means data reading start and taken as data reading 
    until ST2 register is read. Therefore, when any of measurement data is read, be sure to 
    read ST2 register at the end. */

    return NRF_SUCCESS;
}


uint32_t app_mpu_set_sleep_mode(bool sleep_flag)
{
	uint32_t err_code;
	uint8_t regTemp = 0;

	err_code = TWI_read(AGB0_REG_PWR_MGMT_1, &regTemp, 1);
	if(err_code != NRF_SUCCESS) return err_code;
//	NRF_LOG_INFO("app_mpu_set_sleep_mode B: 0x%x, %d", regTemp, regTemp);

	if(sleep_flag == true)
	{
		regTemp |= (0x01 << 6);
	}
	else
	{
		regTemp &= ~(0x01 << 6);
	}
//	NRF_LOG_INFO("app_mpu_set_sleep_mode A: 0x%x, %d", regTemp, regTemp);
	err_code = TWI_write(AGB0_REG_PWR_MGMT_1, &regTemp, 1);

	return err_code;
}

uint32_t app_mpu_on_off_sensor(bool acc_disable_flag, bool gyro_disable_flag)
{
	uint32_t err_code;
	uint8_t regTemp = 0;

	err_code = TWI_read(MPU_REG_PWR_MGMT_2, &regTemp, 1);
	if(err_code != NRF_SUCCESS) return err_code;

//	NRF_LOG_INFO("MPU_REG_PWR_MGMT_2 B: 0x%x, %d", regTemp, regTemp);

	if(acc_disable_flag == true)
	{
	    regTemp |= (0x07 << 3);
	}
	else
	{
	    regTemp &= ~(0x07 << 3);
	}

	if(gyro_disable_flag == true)
	{
	    regTemp |= (0x07);
	}
	else
	{
	    regTemp &= ~(0x07);
	}
//	NRF_LOG_INFO("MPU_REG_PWR_MGMT_2 A: 0x%x, %d", regTemp, regTemp);
	err_code = TWI_write(MPU_REG_PWR_MGMT_2, &regTemp, 1);
	return err_code;
}

static float getAccelResolution(accel_range range)
{
	float fRes;
	switch(range)
	{
		case AFS_2G: 
			fRes =  2.0f/32768.0f;
			break;   
		case AFS_4G: 
			fRes =  4.0f/32768.0f;
			break;      
		case AFS_8G:
			fRes =  8.0f/32768.0f;
			break; 
		case AFS_16G:   
			fRes =  16.0f/32768.0f;
			break; 
		default:
			break;
	}
	return fRes;
}

static float getGyroResolution(gyro_range range)
{
	float fRes;
	switch(range)
	{
		case GFS_250DPS: 
			fRes =  250.0/32768.0;
			break;   
		case GFS_500DPS: 
			fRes =  500.0/32768.0;
			break;      
		case GFS_1000DPS:
			fRes =  1000.0/32768.0;
			break; 
		case GFS_2000DPS:   
			fRes =  2000.0/32768.0;
			break; 
		default:
			break;
	}
	return fRes;
}


/*
MIT License

Copyright (c) 2018 Hideaki Tai

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.*/

/*
calibrateAccelGyro()
set_acc_gyro_to_calibration()
collect_acc_gyro_data_to()
write_accel_offset()

*/

// Defines
#define read_bytes(reg_addr, nbytes, pdata)       \
    err_code = TWI_read(reg_addr, pdata, nbytes); \
    if(err_code != NRF_SUCCESS) return err_code

#define write_byte(reg_addr, val)                 \
    reg_data = val;                               \
    err_code = TWI_write(reg_addr, &reg_data, 1); \
    if (err_code != NRF_SUCCESS) return err_code

// Calibration Parameters
float acc_bias[3] = {0.f, 0.f, 0.f};   // acc calibration value in ACCEL_FS_SEL: 2g
float gyro_bias[3] = {0.f, 0.f, 0.f};  // gyro calibration value in GYRO_FS_SEL: 250dps
//extern float mag_bias_factory[3];   // {0., 0., 0.};
//extern float mag_bias[3];           // {0., 0., 0.}; mag calibration value in MAG_OUTPUT_BITS: 16BITS
//extern float mag_scale[3];          // {1., 1., 1.};
static const uint16_t CALIB_GYRO_SENSITIVITY = 131;     // LSB/degrees/sec
static const uint16_t CALIB_ACCEL_SENSITIVITY = 16384;  // LSB/g

static uint32_t set_acc_gyro_to_calibration(void);
static uint32_t collect_acc_gyro_data_to(float* a_bias, float* g_bias);
static uint32_t write_accel_offset(void);
static uint32_t write_gyro_offset(void);


static void setAccelGyroBias(void)
{
    write_accel_offset();
    write_gyro_offset();
    nrf_delay_ms(100);
}
/**@brief Averaging initial values and saving
*/
static void calibrateAccelGyro(void) 
{
    set_acc_gyro_to_calibration();
    collect_acc_gyro_data_to(acc_bias, gyro_bias);
    write_accel_offset();
    write_gyro_offset();
    nrf_delay_ms(100);
    initMPU9250();
    nrf_delay_ms(1000);
}

static uint32_t set_acc_gyro_to_calibration(void)
{
    uint32_t err_code;
    uint8_t reg_data = 0;

    // reset device
    // Write a one to bit 7 reset bit; toggle reset device
    write_byte(MPU_REG_PWR_MGMT_1, 0x80);
    nrf_delay_ms(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
    write_byte(MPU_REG_PWR_MGMT_1, 0x01);
    write_byte(MPU_REG_PWR_MGMT_2, 0x00);
    nrf_delay_ms(200);

    // Configure device for bias calculation
    write_byte(MPU_REG_INT_ENABLE, 0x00);   // Disable all interrupts
    write_byte(MPU_REG_FIFO_EN, 0x00);      // Disable FIFO
    write_byte(MPU_REG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
    write_byte(MPU_REG_I2C_MST_CTRL, 0x00); // Disable I2C master
    write_byte(MPU_REG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    write_byte(MPU_REG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
    nrf_delay_ms(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    write_byte(MPU_REG_CONFIG, 0x01);       // Set low-pass filter to 188 Hz
    write_byte(MPU_REG_SMPLRT_DIV, 0x00);   // Set sample rate to 1 kHz
    write_byte(MPU_REG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    write_byte(MPU_REG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
    write_byte(MPU_REG_ACCEL_CONFIG_2, 0x00); // Set low-pass filter ot 218.1 Hz, 1kHz sample rate

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    write_byte(MPU_REG_USER_CTRL, 0x40);    // Enable FIFO
    write_byte(MPU_REG_FIFO_EN, 0x78);      // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9250)
    nrf_delay_ms(40);                       // accumulate 40 samples in 40 milliseconds = 480 bytes

    return NRF_SUCCESS;
}

 static uint32_t collect_acc_gyro_data_to(float* a_bias, float* g_bias) 
 {
    uint32_t err_code;
    uint8_t reg_data = 0;

    // At the end of sample accumulation, turn off FIFO sensor read
    uint8_t data[12];                                    // data array to hold accelerometer and gyro x, y, z, data
    write_byte(MPU_REG_FIFO_EN, 0x00);             // Disable gyro and accelerometer sensors for FIFO
    read_bytes(MPU_REG_FIFO_COUNTH, 2, &data[0]);  // read FIFO sample count
    uint16_t fifo_count = ((uint16_t)data[0] << 8) | data[1];
    uint16_t packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging

    for (uint16_t ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        read_bytes(MPU_REG_FIFO_R_W, 12, &data[0]);              // read data for averaging
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        a_bias[0] += (float)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        a_bias[1] += (float)accel_temp[1];
        a_bias[2] += (float)accel_temp[2];
        g_bias[0] += (float)gyro_temp[0];
        g_bias[1] += (float)gyro_temp[1];
        g_bias[2] += (float)gyro_temp[2];
    }
    a_bias[0] /= (float)packet_count;  // Normalize sums to get average count biases
    a_bias[1] /= (float)packet_count;
    a_bias[2] /= (float)packet_count;
    g_bias[0] /= (float)packet_count;
    g_bias[1] /= (float)packet_count;
    g_bias[2] /= (float)packet_count;

    if (a_bias[2] > 0L) {
        a_bias[2] -= (float)CALIB_ACCEL_SENSITIVITY;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else {
        a_bias[2] += (float)CALIB_ACCEL_SENSITIVITY;
    }

    return NRF_SUCCESS;
}

static uint32_t write_accel_offset() {
    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    uint32_t err_code;
    uint8_t reg_data = 0;

    uint8_t read_data[2] = {0};
    int16_t acc_bias_reg[3] = {0, 0, 0};      // A place to hold the factory accelerometer trim biases
    read_bytes(MPU_REG_XA_OFFSET_H, 2, &read_data[0]);  // Read factory accelerometer trim values
    acc_bias_reg[0] = ((int16_t)read_data[0] << 8) | read_data[1];
    read_bytes(MPU_REG_YA_OFFSET_H, 2, &read_data[0]);
    acc_bias_reg[1] = ((int16_t)read_data[0] << 8) | read_data[1];
    read_bytes(MPU_REG_ZA_OFFSET_H, 2, &read_data[0]);
    acc_bias_reg[2] = ((int16_t)read_data[0] << 8) | read_data[1];

    int16_t mask_bit[3] = {1, 1, 1};  // Define array to hold mask bit for each accelerometer bias axis
    for (int i = 0; i < 3; i++) {
        if (acc_bias_reg[i] % 2) {
            mask_bit[i] = 0;
        }
        acc_bias_reg[i] -= (int16_t)acc_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
        if (mask_bit[i]) {
            acc_bias_reg[i] = acc_bias_reg[i] & ~mask_bit[i];  // Preserve temperature compensation bit
        } else {
            acc_bias_reg[i] = acc_bias_reg[i] | 0x0001;  // Preserve temperature compensation bit
        }
    }

    uint8_t write_data[6] = {0};
    write_data[0] = (acc_bias_reg[0] >> 8) & 0xFF;
    write_data[1] = (acc_bias_reg[0]) & 0xFF;
    write_data[2] = (acc_bias_reg[1] >> 8) & 0xFF;
    write_data[3] = (acc_bias_reg[1]) & 0xFF;
    write_data[4] = (acc_bias_reg[2] >> 8) & 0xFF;
    write_data[5] = (acc_bias_reg[2]) & 0xFF;

    // Push accelerometer biases to hardware registers
    write_byte(MPU_REG_XA_OFFSET_H, write_data[0]);
    write_byte(MPU_REG_XA_OFFSET_L, write_data[1]);
    write_byte(MPU_REG_YA_OFFSET_H, write_data[2]);
    write_byte(MPU_REG_YA_OFFSET_L, write_data[3]);
    write_byte(MPU_REG_ZA_OFFSET_H, write_data[4]);
    write_byte(MPU_REG_ZA_OFFSET_L, write_data[5]);

    return NRF_SUCCESS;
}

static uint32_t write_gyro_offset() {
    uint32_t err_code;
    uint8_t reg_data = 0;

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    uint8_t gyro_offset_data[6] = {0};
    gyro_offset_data[0] = (-(int16_t)gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    gyro_offset_data[1] = (-(int16_t)gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
    gyro_offset_data[2] = (-(int16_t)gyro_bias[1] / 4 >> 8) & 0xFF;
    gyro_offset_data[3] = (-(int16_t)gyro_bias[1] / 4) & 0xFF;
    gyro_offset_data[4] = (-(int16_t)gyro_bias[2] / 4 >> 8) & 0xFF;
    gyro_offset_data[5] = (-(int16_t)gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    write_byte(MPU_REG_XG_OFFSET_H, gyro_offset_data[0]);
    write_byte(MPU_REG_XG_OFFSET_L, gyro_offset_data[1]);
    write_byte(MPU_REG_YG_OFFSET_H, gyro_offset_data[2]);
    write_byte(MPU_REG_YG_OFFSET_L, gyro_offset_data[3]);
    write_byte(MPU_REG_ZG_OFFSET_H, gyro_offset_data[4]);
    write_byte(MPU_REG_ZG_OFFSET_L, gyro_offset_data[5]);

    return NRF_SUCCESS;
}

uint32_t read_accel_gyro(int16_t* destination) {
    uint32_t err_code;
    uint8_t raw_data[14];   // Read Accel, gyro, temp

    read_bytes(AGB0_REG_ACCEL_XOUT_H, 14, raw_data);                     // Read the 14 raw data registers into data array
    destination[0] = (int16_t)((raw_data[0] << 8) | raw_data[1]);  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    destination[2] = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    destination[3] = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    destination[4] = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    destination[5] = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    destination[6] = (int16_t)((raw_data[12] << 8) | raw_data[13]);

    return NRF_SUCCESS;
}


// Defines
#define read_bytes_m(reg_addr, nbytes, pdata)      \
    err_code = TWIM_read(reg_addr, pdata, nbytes); \
    if(err_code != NRF_SUCCESS) return err_code;

#define write_byte_m(reg_addr, val)                \
    reg_data = val;                               \
    err_code = TWIM_write(reg_addr, &reg_data, 1); \
    if (err_code != NRF_SUCCESS) return err_code;

static uint32_t initAK09916(void) 
{
    uint32_t err_code;
    uint8_t reg_data = 0;

    mag_resolution = 10. * 4912. / 32760.0;

    // First extract the factory calibration for each magnetometer axis
    uint8_t raw_data[3];                            // x/y/z gyro calibration data stored here
    write_byte_m(MPU_AK89XX_REG_CNTL, 0x00);  // Power down magnetometer
    nrf_delay_ms(10);
    write_byte_m(MPU_AK89XX_REG_CNTL, 0x0F);  // Enter Fuse ROM access mode
    nrf_delay_ms(10);
    read_bytes_m(MPU_AK89XX_REG_ASAX, 3, &raw_data[0]);      // Read the x-, y-, and z-axis calibration values
    mag_bias_factory[0] = (float)(raw_data[0] - 128) / 256. + 1.;  // Return x-axis sensitivity adjustment values, etc.
    mag_bias_factory[1] = (float)(raw_data[1] - 128) / 256. + 1.;
    mag_bias_factory[2] = (float)(raw_data[2] - 128) / 256. + 1.;
    write_byte_m(MPU_AK89XX_REG_CNTL, 0x00);  // Power down magnetometer
    nrf_delay_ms(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition MAG_MODE (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
    write_byte_m(MPU_AK89XX_REG_CNTL, (uint8_t)OUTPUT_RESOLUTION_16bit << 4 | MAG_MODE);  // Set magnetometer data resolution and sample ODR
    nrf_delay_ms(10);   
}

uint32_t powerdownAK09916(void) 
{
    uint32_t err_code;
    uint8_t reg_data = 0;

    write_byte_m(M_REG_CNTL2, 0x00);  // Power down magnetometer
    nrf_delay_ms(10);  
}

static void calibrateMag(void) 
{
    int32_t bias[3] = {0, 0, 0}, scale[3] = {0, 0, 0};
    int16_t mag_max[3] = {-32767, -32767, -32767};
    int16_t mag_min[3] = {32767, 32767, 32767};

    uint16_t sample_count = 0;
    if (MAG_MODE == 0x02)
        sample_count = 128;
    else if (MAG_MODE == 0x06)
        sample_count = 1500;

    for(uint16_t ii = 0; ii < sample_count; ii++)
    {
        app_mpu_read_magnetometer(&magn_values);

        //NRF_LOG_INFO("magRaw: %d %d %d", magn_values.x, magn_values.y, magn_values.z);

        if(magn_values.x > mag_max[0]) mag_max[0] = magn_values.x;
        if(magn_values.x < mag_min[0]) mag_min[0] = magn_values.x;
        
        if(magn_values.y > mag_max[1]) mag_max[1] = magn_values.y;
        if(magn_values.y < mag_min[1]) mag_min[1] = magn_values.y;
            
        if(magn_values.z > mag_max[2]) mag_max[2] = magn_values.z;
        if(magn_values.z < mag_min[2]) mag_min[2] = magn_values.z;

        if (MAG_MODE == 0x02) nrf_delay_ms(125);
        if (MAG_MODE == 0x06) nrf_delay_ms(12);
    }

    bias[0] = (mag_max[0] + mag_min[0]) / 2;  // get average x mag bias in counts
    bias[1] = (mag_max[1] + mag_min[1]) / 2;  // get average y mag bias in counts
    bias[2] = (mag_max[2] + mag_min[2]) / 2;  // get average z mag bias in counts

    //float bias_resolution = get_mag_resolution(MAG_OUTPUT_BITS::M16BITS);
    mag_bias[0] = (float)bias[0] * mag_resolution * mag_bias_factory[0];  // save mag biases in G for main program
    mag_bias[1] = (float)bias[1] * mag_resolution * mag_bias_factory[1];
    mag_bias[2] = (float)bias[2] * mag_resolution * mag_bias_factory[2];

    scale[0] = (float)(mag_max[0] - mag_min[0]) * mag_bias_factory[0] / 2;	// get average x axis max chord length in counts
    scale[1] = (float)(mag_max[1] - mag_min[1]) * mag_bias_factory[1] / 2;	// get average y axis max chord length in counts
    scale[2] = (float)(mag_max[2] - mag_min[2]) * mag_bias_factory[2] / 2;	// get average z axis max chord length in counts

    float avg_rad = scale[0] + scale[1] + scale[2];
    avg_rad /= 3.0;

    mag_scale[0] = avg_rad / ((float)scale[0]);
    mag_scale[1] = avg_rad / ((float)scale[1]);
    mag_scale[2] = avg_rad / ((float)scale[2]);
}

void autoOffsets(void)
{
    double acc[3] = {0};
    double gyr[3] = {0};
    int16_t buffer[7] = {0};

    for(int j=0; j<10; j++){  // Allow to get stable values
        read_accel_gyro(buffer);
        nrf_delay_ms(14);
    }
   
    for (int i=0; i<50; i++) {
        read_accel_gyro(buffer);
        //acc[0] += (double)buffer[0];
        //acc[1] += (double)buffer[1];
        //acc[2] += (double)buffer[2];
        gyr[0] += (double)buffer[3];
        gyr[1] += (double)buffer[4];
        gyr[2] += (double)buffer[5];
        nrf_delay_ms(14);
    }
    //acc_bias[0] = acc[0] / 50;
    //acc_bias[1] = acc[1] / 50;
    //acc_bias[2] = acc[2] / 50;

    //if (acc_bias[2] > 0)
    //  acc_bias[2] -= 16384.0;
    //else
    //  acc_bias[2] += 16384.0;

    gyro_bias[0] = gyr[0] / 50;
    gyro_bias[1] = gyr[1] / 50;
    gyro_bias[2] = gyr[2] / 50;
}

void setupIMU(bool verbose)
{
    twi_master_init();                /* Initializes TWI */

    acc_resolution = getAccelResolution(D_ACCEL_FULL_SCALE);
    gyro_resolution = getGyroResolution(D_GYRO_FULL_SCALE);

    initICM20948();
#if defined(D_USE_MADGWICK)
    initAK09916();
#endif

    if (!isCalibrated()) {
        NRF_LOG_INFO("Set your device to the upright position and don't move it. Calibrating ...");
        nrf_delay_ms(1000);
        autoOffsets();

        NRF_LOG_INFO("Saving offsets");
        saveCalibration(acc_bias, gyro_bias, mag_bias, mag_scale, 0);
        nrf_delay_ms(1000);
        NRF_LOG_INFO("Done!");
    }

    loadCalibration(acc_bias, gyro_bias, mag_bias, mag_scale, &b_left_handed_flag);

    if (verbose) {
        NRF_LOG_INFO("accel bias: %d %d %d", 
            (int)(acc_bias[0]), (int)(acc_bias[1]), (int)(acc_bias[2]));
        NRF_LOG_INFO("gyro bias: %d %d %d", 
            (int)(gyro_bias[0]), (int)(gyro_bias[1]), (int)(gyro_bias[2]));

        NRF_LOG_INFO("B bias : "NRF_LOG_FLOAT_MARKER2" "NRF_LOG_FLOAT_MARKER2" "NRF_LOG_FLOAT_MARKER2,
                NRF_LOG_FLOAT2(mag_bias[0]),
                NRF_LOG_FLOAT2(mag_bias[1]),
                NRF_LOG_FLOAT2(mag_bias[2]));
        NRF_LOG_INFO("B scale: "NRF_LOG_FLOAT_MARKER2" "NRF_LOG_FLOAT_MARKER2" "NRF_LOG_FLOAT_MARKER2,
                NRF_LOG_FLOAT2(mag_scale[0]),
                NRF_LOG_FLOAT2(mag_scale[1]),
                NRF_LOG_FLOAT2(mag_scale[2]));
        NRF_LOG_INFO(" ");
    }
}

