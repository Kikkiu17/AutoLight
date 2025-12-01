#ifndef __USERHANDLERS_H__
#define __USERHANDLERS_H__

#include <string.h>
#include <inttypes.h>
#include <stdio.h>

#include "../ESP8266/esp8266.h"
#include "../settings.h"

Response_t HANDLER_SetTriggerDistance(Connection_t *conn, char *key_ptr);

#endif