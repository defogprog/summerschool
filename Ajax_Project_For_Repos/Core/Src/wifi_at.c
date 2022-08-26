/*
 * wifi_at.c
 *
 *  Created on: Aug 24, 2022
 *      Author: cheburiek
 */

#include "wifi_at.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart6;
extern osSemaphoreId_t semUART_TX_WIFIHandle;
extern osSemaphoreId_t semUART_RX_WIFIHandle;

static char tx_buffer[128];
static char rx_buffer[128];

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  osSemaphoreRelease(semUART_TX_WIFIHandle);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  {
  osSemaphoreRelease(semUART_RX_WIFIHandle);
}

static void send(char *text) {
  osSemaphoreAcquire(semUART_TX_WIFIHandle, osWaitForever);
  HAL_UART_Transmit_IT(&huart6, (uint8_t*)text, strlen(text));
}

static void clear_buf() {
  memset(tx_buffer, sizeof(tx_buffer), 1);
}



/******************************************************************/
int wifi_init(void) {
  wifi_set_mode(WIFI_MODE_STA);
  send("ATE0\n");
  return 0;
}

int wifi_set_mode(wifi_mode_t mode) {
  clear_buf();
  sprintf(tx_buffer, "AT+CWMODE=%d\n", mode);
  send(tx_buffer);
  osDelay(3000);
  return 0;
}

int wifi_connect_to_ap(char *ap_name, char *pass, uint32_t tmt) {
  TickType_t tmt_time = xTaskGetTickCount() + tmt;
  uint32_t cnt = 0;

  sprintf(tx_buffer, "AT+CWJAP=%s,%s\n", ap_name, pass);
  send(tx_buffer);

  while (tmt_time >= xTaskGetTickCount()) {
    HAL_UART_Receive_IT(&huart6, (uint8_t*)&rx_buffer[cnt], 1);
    osStatus_t st = osSemaphoreAcquire(semUART_RX_WIFIHandle, tmt_time - xTaskGetTickCount());
    if (st == osErrorTimeout)
      return WIFI_ERR_TMT;

    if (rx_buffer[cnt++] == '\n') {
      if (strncmp(rx_buffer, "OK", 2) != 0) {
        return WIFI_ERR_GENEGAL;
      }
      break;
    }
  }
  return 0;
}
