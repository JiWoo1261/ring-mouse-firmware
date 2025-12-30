#include <string.h>

#include "nrf_sdh.h"
#include "uicr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


void saveCalibration(float accBias[3], float gyroBias[3], float magBias[3], float magScale[3], bool left_handed)
{
  NRF_UICR_Type tmp;
  //If SD(softdevice) is running, return true
  uint8_t sd_enabled = nrf_sdh_is_enabled();
  
  //If SD is enabled, stop it.
  if(sd_enabled) {
    nrf_sdh_disable_request();
  }

  /************ UICR update *******************************/
  memcpy(&tmp, NRF_UICR, sizeof(NRF_UICR_Type));
  
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
  
  NRF_NVMC->ERASEUICR = NVMC_ERASEUICR_ERASEUICR_Erase << NVMC_ERASEUICR_ERASEUICR_Pos;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}

  tmp.CUSTOMER[0] = 0x01; /* calibrated flag */
  tmp.CUSTOMER[1] = 0xffff0000 | (int16_t)accBias[0];
  tmp.CUSTOMER[2] = 0xffff0000 | (int16_t)accBias[1];
  tmp.CUSTOMER[3] = 0xffff0000 | (int16_t)accBias[2];
  tmp.CUSTOMER[4] = 0xffff0000 | (int16_t)gyroBias[0];
  tmp.CUSTOMER[5] = 0xffff0000 | (int16_t)gyroBias[1];
  tmp.CUSTOMER[6] = 0xffff0000 | (int16_t)gyroBias[2];
  tmp.CUSTOMER[7] = (int32_t)(magBias[0]*1000);
  tmp.CUSTOMER[8] = (int32_t)(magBias[1]*1000);
  tmp.CUSTOMER[9] = (int32_t)(magBias[2]*1000);
  tmp.CUSTOMER[10] = (int32_t)(magScale[0]*1000);
  tmp.CUSTOMER[11] = (int32_t)(magScale[1]*1000);
  tmp.CUSTOMER[12] = (int32_t)(magScale[2]*1000);
  tmp.CUSTOMER[14] = (uint32_t)(left_handed);

  memcpy(NRF_UICR, &tmp, sizeof(NRF_UICR_Type));
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
  
  NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
  while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
  /************ UICR update end *******************************/

  //Reboot if SD was enabled
  if(sd_enabled) {
    NVIC_SystemReset();
  }
}


void loadCalibration(float accBias[3], float gyroBias[3], float magBias[3], float magScale[3], bool *left_handed)
{
  accBias[0] = (float)(int16_t)(NRF_UICR->CUSTOMER[1] & 0x0000ffff);
  accBias[1] = (float)(int16_t)(NRF_UICR->CUSTOMER[2] & 0x0000ffff);
  accBias[2] = (float)(int16_t)(NRF_UICR->CUSTOMER[3] & 0x0000ffff);
  gyroBias[0] = (float)(int16_t)(NRF_UICR->CUSTOMER[4] & 0x0000ffff);
  gyroBias[1] = (float)(int16_t)(NRF_UICR->CUSTOMER[5] & 0x0000ffff);
  gyroBias[2] = (float)(int16_t)(NRF_UICR->CUSTOMER[6] & 0x0000ffff);
  magBias[0] = ((float)(int32_t)NRF_UICR->CUSTOMER[7])/1000.f;
  magBias[1] = ((float)(int32_t)NRF_UICR->CUSTOMER[8])/1000.f;
  magBias[2] = ((float)(int32_t)NRF_UICR->CUSTOMER[9])/1000.f;
  magScale[0] = ((float)(int32_t)NRF_UICR->CUSTOMER[10])/1000.f;
  magScale[1] = ((float)(int32_t)NRF_UICR->CUSTOMER[11])/1000.f;
  magScale[2] = ((float)(int32_t)NRF_UICR->CUSTOMER[12])/1000.f;
  *left_handed = (bool)NRF_UICR->CUSTOMER[14];
}


bool isCalibrated(void)
{
  return (NRF_UICR->CUSTOMER[0] == 0x01);
}

