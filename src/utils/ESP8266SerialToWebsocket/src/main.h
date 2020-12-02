#pragma once

#include <FS.h>
#ifdef USE_LITTLE_FS
#include <LittleFS.h>
#define FILESYSTEM LittleFS
#else
#define FILESYSTEM SPIFFS
#endif
