#include "userhandlers.h"

uint32_t TRIGGER_DISTANCE;

Response_t HANDLER_SetTriggerDistance(Connection_t *conn, char *key_ptr)
{
    uint32_t size = 0;
    char *new_string_distance = WIFI_GetKeyValue(conn, key_ptr, &size);
    int32_t new_distance = bufferToInt(new_string_distance, size);

    if (new_distance == -1)
    {
        WIFI_SendResponse(conn, "500 Internal Server Error", "Distanza non valida", 19);
        return ERR;
    }

    TRIGGER_DISTANCE = new_distance;

    savedata.trigger_distance = new_distance;
    FLASH_WriteSaveData();
    return WIFI_SendResponse(conn, "200 OK", NULL, 0);
}