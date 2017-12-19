#include "vl53l0x_platform.h"
#include "vl53l0x_api.h"

VL53L0X_Error VL53L0X_Init(VL53L0X_DEV Dev, I2C_HandleTypeDef * hi2c, uint8_t addr) {
  __HAL_LOCK(Dev);
  Dev -> Instance = hi2c;
  Dev -> Address = addr;
  Dev -> TxFlag = 0;
  Dev -> RxFlag = 0;
  Dev -> MemTxFlag = 0;
  Dev -> MemRxFlag = 0;
  __HAL_UNLOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev) {
  __HAL_LOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev) {
  __HAL_UNLOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
  __HAL_LOCK(Dev);
  uint8_t buf[100] ={index, 0x00};
  for (uint8_t i = 0; i < sizeof(pdata); i ++) {
    buf[i + 1] = pdata[i];
  }
  HAL_I2C_Master_Transmit_DMA(Dev -> Instance, Dev -> Address, buf, count + 1);
  uint32_t tickStart = HAL_GetTick();
  while(!Dev -> TxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> TxFlag = 0;
  __HAL_UNLOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

// the ranging_sensor_comms.dll will take care of the page selection
VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count) {
  __HAL_LOCK(Dev);
  HAL_I2C_Master_Transmit_DMA(Dev -> Instance, Dev -> Address, &index, 0x01);
  uint32_t tickStart = HAL_GetTick();
  while(!Dev -> TxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> TxFlag = 0;
  HAL_I2C_Master_Receive_DMA(Dev -> Instance, Dev -> Address, pdata, count);
  tickStart = HAL_GetTick();
  while (!Dev -> RxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> RxFlag = 0;
  __HAL_UNLOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data) {
  __HAL_LOCK(Dev);
  uint8_t buf[2] = {index, data};
  HAL_I2C_Master_Transmit_DMA(Dev -> Instance, Dev -> Address, buf, 0x02);
  uint32_t tickStart = HAL_GetTick();
  while(!Dev -> TxFlag) {
    if(HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> TxFlag = 0;
  __HAL_UNLOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data) {
  __HAL_LOCK(Dev);
  uint8_t dl = data & 0xff;
  uint8_t dh = data >> 0x08;
  uint8_t buf[3] = {index, dh, dl};
  HAL_I2C_Master_Transmit_DMA(Dev -> Instance, Dev -> Address, buf, 0x03);
  uint32_t tickStart = HAL_GetTick();
  while(!Dev -> TxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> TxFlag = 0;
  __HAL_UNLOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data) {
  __HAL_LOCK(Dev);
  uint8_t buf[5] = {index, 0x00};
  for (uint8_t i = 1; i < 5; i ++) {
    buf[i] = (data) >> (0x08 * (4 - i)); // RShift data.
  }
  HAL_I2C_Master_Transmit_DMA(Dev -> Instance, Dev -> Address, buf, 0x05);
  uint32_t tickStart = HAL_GetTick();
  while(!Dev -> TxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> TxFlag = 0;
  __HAL_UNLOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData) {
  __HAL_LOCK(Dev);
  HAL_I2C_Master_Transmit_DMA(Dev -> Instance, Dev -> Address, &index, 0x01);
  uint32_t tickStart = HAL_GetTick();
  while(!Dev -> TxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> TxFlag = 0;
  uint8_t buf[2] = {0x00};
  HAL_I2C_Master_Receive_DMA(Dev -> Instance, Dev -> Address, buf, 0x01);
  tickStart = HAL_GetTick();
  while(!Dev -> RxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> RxFlag = 0;
  uint8_t tmp_byte = (buf[1] & AndData) | OrData;
  buf[0] = index;
  buf[1] = tmp_byte;
  HAL_I2C_Master_Transmit_DMA(Dev -> Instance, Dev -> Address, buf, 0x02);
  tickStart = HAL_GetTick();
  while(!Dev -> TxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> TxFlag = 0;
  __HAL_UNLOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data) {
  __HAL_LOCK(Dev);
  HAL_I2C_Master_Transmit_DMA(Dev -> Instance, Dev -> Address, &index, 0x01);
  uint32_t tickStart = HAL_GetTick();
  while(!Dev -> TxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> TxFlag = 0;
  HAL_I2C_Master_Receive_DMA(Dev -> Instance, Dev -> Address, data, 0x01);
  tickStart = HAL_GetTick();
  while(!Dev -> RxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> RxFlag = 0;
  __HAL_UNLOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data){
  __HAL_LOCK(Dev);
  HAL_I2C_Master_Transmit_DMA(Dev -> Instance, Dev -> Address, &index, 0x01);
  uint32_t tickStart = HAL_GetTick();
  while(!Dev -> TxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> TxFlag = 0;
  uint8_t buf[2] = {0x00};
  HAL_I2C_Master_Receive_DMA(Dev -> Instance, Dev -> Address, buf, 0x02);
  tickStart = HAL_GetTick();
  while(!Dev -> RxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      *data = 0x00;
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> RxFlag = 0;
  *data = (buf[0] << 0x08) | buf[1];
  __HAL_UNLOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error  VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data){
  __HAL_LOCK(Dev);
  HAL_I2C_Master_Transmit_DMA(Dev -> Instance, Dev -> Address, &index, 0x01);
  uint32_t tickStart = HAL_GetTick();
  while(!Dev -> TxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> TxFlag = 0;
  uint8_t buf[4] = {0x00};
  HAL_I2C_Master_Receive_DMA(Dev -> Instance, Dev -> Address, buf, 0x04);
  * data = 0x00;
  tickStart = HAL_GetTick();
  while(!Dev -> RxFlag) {
    if (HAL_GetTick() - tickStart > VL53L0X_MAX_DELAY_TICKS) {
      *data = 0x00;
      __HAL_UNLOCK(Dev);
      return VL53L0X_ERROR_TIME_OUT;
    }
  }
  Dev -> RxFlag = 0;
  for (uint8_t i = 0; i < 4; i ++) {
    * data |= buf[i] << (0x08 * (3 - i)); // RShift data.
  }
  __HAL_UNLOCK(Dev);
  return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev){
  return VL53L0X_ERROR_NONE;
}
