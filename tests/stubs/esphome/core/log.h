#pragma once
#define ESP_LOGW(...) ((void) 0)
#define ESP_LOGI(...) ((void) 0)
#define ESP_LOGD(...) ((void) 0)
// ESPHome's real macros require a TAG in the calling translation unit.
#define LOG_SENSOR(prefix, label, obj) ((void) (TAG), (void) (obj))
#define LOG_TEXT_SENSOR(prefix, label, obj) ((void) (TAG), (void) (obj))
