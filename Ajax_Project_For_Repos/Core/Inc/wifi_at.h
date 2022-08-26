/*
 * wifi_at.h
 *
 *  Created on: Aug 24, 2022
 *      Author: cheburiek
 */

#ifndef INC_WIFI_AT_H_
#define INC_WIFI_AT_H_

#include <stdlib.h>
#include <stdint.h>

typedef enum {
  WIFI_OK = 0,
  WIFI_ERR_IO,
  WIFI_ERR_TMT,
  WIFI_ERR_GENEGAL
} wifi_status_t;

typedef enum {
  WIFI_MODE_NONE = 0,
  WIFI_MODE_STA = 1,
  WIFI_MODE_AP = 2,
  WIFI_MODE_HYBRID = 3,
} wifi_mode_t;

int wifi_init(void);

int wifi_set_mode(wifi_mode_t mode);

int wifi_connect_to_ap(char *ap_name, char *pass, uint32_t tmt);

#endif /* INC_WIFI_AT_H_ */
