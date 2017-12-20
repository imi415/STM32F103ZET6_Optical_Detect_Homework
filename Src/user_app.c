#include "gpio.h"
#include "i2c.h"

#include "user_board.h"

#include "user_oled_12864.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include "user_app.h"

OLED_HandleTypeDef oled1;
OLED_HandleTypeDef oled2;

VL53L0X_Dev_t vl;
VL53L0X_DEV vlp = & vl;
VL53L0X_DeviceInfo_t vl_info;
VL53L0X_RangingMeasurementData_t vl_data;
uint32_t vl_ref_spad_count;
uint8_t vl_is_aperture_spads;
uint8_t vl_vhv_settings;
uint8_t vl_phase_cal;
FixPoint1616_t vl_limit_check_current;

uint8_t i2c_counter = 0;

void setup(void) {
  VL53L0X_Init(vlp, &hi2c1, 0x52);
  OLED_Init(&oled1, &hi2c1, 0x78);
  OLED_Init(&oled2, &hi2c2, 0x78);

  OLED_Clear(&oled1);
  OLED_Clear(&oled2);

  VL53L0X_DataInit(vlp);
  VL53L0X_GetDeviceInfo(vlp, & vl_info);

  OLED_String_Display(&oled1, 0, 0, "INIT");
  OLED_String_Display(&oled2, 0, 0, "INIT");

  vl_print_info();
  vl_measure_init();

  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
}

void loop(void) {
  HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
  vl_range_once();
  HAL_Delay(10);
  HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
  HAL_Delay(190);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  led_stat();
	if (hi2c -> Instance == oled1.Instance -> Instance && oled1.Lock == HAL_LOCKED) {
		oled1.TxFlag = 1;
	}
  else if (hi2c -> Instance == vlp -> Instance -> Instance && vlp -> Lock == HAL_LOCKED) {
    vlp -> TxFlag = 1;
  }
  else if (hi2c -> Instance == oled2.Instance -> Instance && oled2.Lock == HAL_LOCKED) {
    oled2.TxFlag = 1;
  }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef * hi2c) {
  led_stat();
  if (hi2c -> Instance == I2C1 && vlp -> Lock == HAL_LOCKED) {
    vlp -> RxFlag = 1;
  }
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  led_stat();
  if (hi2c -> Instance == I2C1 && vlp -> Lock == HAL_LOCKED) {
    vlp -> MemTxFlag = 1;
  }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  led_stat();
  if (hi2c -> Instance == I2C1 && vlp -> Lock == HAL_LOCKED) {
    vlp -> MemRxFlag = 1;
  }
}

void led_stat(void) {
  i2c_counter ++;
  if (i2c_counter == 100) {
    i2c_counter = 0;
    HAL_GPIO_TogglePin(LED_1_GPIO_Port, LED_1_Pin);
  }
}

void vl_print_info(void) {
  char buf[50];
  snprintf(buf, 50, "%s", vl_info.Name);
  OLED_String_Display(&oled1, 0, 0, buf);
  snprintf(buf, 50, "Type: %s", vl_info.Type);
  OLED_String_Display(&oled1, 0, 1, buf);
  snprintf(buf, 50, "ID: %s", vl_info.ProductId);
  OLED_String_Display(&oled1, 0, 2, buf);
  snprintf(buf, 50, "Ver: %d.%d", vl_info.ProductRevisionMajor, vl_info.ProductRevisionMinor);
  OLED_String_Display(&oled1, 0, 3, buf);
  OLED_String_Display(&oled1, 0, 4, "2014211844");
}

void vl_measure_init(void) {
  VL53L0X_Error stat = VL53L0X_ERROR_NONE;
  if (stat == VL53L0X_ERROR_NONE) {
    stat = VL53L0X_StaticInit(vlp);
    OLED_String_Display(&oled2, 0, 0, "SI   ");
  }

  if (stat == VL53L0X_ERROR_NONE) {
    stat = VL53L0X_PerformRefCalibration(vlp, &vl_vhv_settings, &vl_phase_cal);
    OLED_String_Display(&oled2, 0, 0, "PRC  ");
  }

  if (stat == VL53L0X_ERROR_NONE) {
    stat = VL53L0X_PerformRefSpadManagement(vlp, &vl_ref_spad_count, &vl_is_aperture_spads);
    OLED_String_Display(&oled2, 0, 0, "PRSM ");
  }

  if (stat == VL53L0X_ERROR_NONE) {
    stat = VL53L0X_SetDeviceMode(vlp, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    OLED_String_Display(&oled2, 0, 0, "SDM  ");
  }

  if (stat == VL53L0X_ERROR_NONE) {
    stat = VL53L0X_SetLimitCheckEnable(vlp, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    OLED_String_Display(&oled2, 0, 0, "SLCE1");
  }

  if (stat == VL53L0X_ERROR_NONE) {
    stat = VL53L0X_SetLimitCheckEnable(vlp, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    OLED_String_Display(&oled2, 0, 0, "SLCE2");
  }

  if (stat == VL53L0X_ERROR_NONE) {
    stat = VL53L0X_SetLimitCheckEnable(vlp, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    OLED_String_Display(&oled2, 0, 0, "SLCE3");
  }

  if (stat == VL53L0X_ERROR_NONE) {
    stat = VL53L0X_SetLimitCheckValue(vlp, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)(1.5*0.023*65536));
    OLED_String_Display(&oled2, 0, 0, "SLCE4");
  }
  OLED_String_Display(&oled2, 0, 0, "STBY ");
}

void vl_range_once(void) {
  VL53L0X_Error stat;
  char buf[50];
  stat = VL53L0X_PerformSingleRangingMeasurement(vlp, &vl_data);
  VL53L0X_GetLimitCheckCurrent(vlp, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &vl_limit_check_current);
  snprintf(buf, 50, "RI_THRESH: %ld", vl_limit_check_current);
  OLED_String_Display(&oled2, 0, 1, buf);
  if (stat != VL53L0X_ERROR_NONE) return;
  snprintf(buf, 50, "Dist: %imm    ", vl_data.RangeMilliMeter);
  OLED_String_Display(&oled2, 0, 2, buf);
  snprintf(buf, 50, "RTT Count: %d", vl_data.EffectiveSpadRtnCount);
  OLED_String_Display(&oled2, 0, 3, buf);
}
