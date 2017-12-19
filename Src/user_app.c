#include "gpio.h"
#include "i2c.h"

#include "user_board.h"

#include "user_oled_12864.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"

#include "user_app.h"

OLED_HandleTypeDef oled;
VL53L0X_Dev_t vl;
VL53L0X_DEV vlp = & vl;
VL53L0X_DeviceInfo_t vl_info;
uint8_t i2c_counter = 0;

void setup(void) {
  VL53L0X_Init(vlp, &hi2c1, 0x52);
  VL53L0X_DataInit(vlp);
  VL53L0X_GetDeviceInfo(vlp, & vl_info);

  OLED_Init(&oled);
  OLED_Clear(&oled);
  OLED_String_Display(&oled, 0, 0, "INIT");
  OLED_Clear(&oled);
  vl_print_info();
  HAL_Delay(5000);
  OLED_Clear(&oled);
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
}

void loop(void) {
  HAL_GPIO_TogglePin(LED_2_GPIO_Port, LED_2_Pin);
  HAL_Delay(500);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  led_stat();
	if (hi2c -> Instance == BOARD_OLED_I2C_INTERFACE.Instance && oled.Lock == HAL_LOCKED) {
		oled.TxFlag = 1;
	}
  else if (hi2c -> Instance == I2C1 && vlp -> Lock == HAL_LOCKED) {
      vlp -> TxFlag = 1;
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
  OLED_String_Display(&oled, 0, 0, buf);
  snprintf(buf, 50, "Type: %s", vl_info.Type);
  OLED_String_Display(&oled, 0, 1, buf);
  snprintf(buf, 50, "ID: %s", vl_info.ProductId);
  OLED_String_Display(&oled, 0, 2, buf);
  snprintf(buf, 50, "Ver: %d.%d", vl_info.ProductRevisionMajor, vl_info.ProductRevisionMinor);
  OLED_String_Display(&oled, 0, 3, buf);
}
