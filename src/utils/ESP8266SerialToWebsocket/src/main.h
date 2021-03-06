#pragma once

#include <Arduino.h>

#include <FS.h>
#ifdef USE_LITTLE_FS
#include <LittleFS.h>
#define FILESYSTEM LittleFS
#else
#define FILESYSTEM SPIFFS
#endif

void websocket_send(String & data, int num = -1);
void websocket_send(char const * data, int num = -1);
