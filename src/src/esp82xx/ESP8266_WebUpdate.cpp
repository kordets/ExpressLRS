#include "debug_elrs.h"

//#define STATIC_IP_AP     "192.168.4.1"
//#define STATIC_IP_CLIENT "192.168.1.50"

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>

#if WIFI_MANAGER
#include <WiFiManager.h>
#endif /* WIFI_MANAGER */

#ifdef WIFI_AP_SSID
#define STASSID WIFI_AP_SSID
#else
#define STASSID "ExpressLRS AP"
#endif
#ifdef WIFI_AP_PSK
#define STAPSK WIFI_AP_PSK
#else
#define STAPSK "expresslrs"
#endif

#ifndef WIFI_TIMEOUT
#define WIFI_TIMEOUT 60 // default to 1min
#endif

MDNSResponder mdns;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

void BeginWebUpdate(void)
{
    const char *host = "elrs_rx";

    DEBUG_PRINTLN("Begin Webupdater");

    wifi_station_set_hostname(host);

    IPAddress addr;

#if WIFI_MANAGER
    WiFiManager wifiManager;
    //WiFiManagerParameter header("<p>Express LRS ESP82xx RX</p>");
    //wifiManager.addParameter(&header);

#ifdef STATIC_IP_AP
    /* Static portal IP (default: 192.168.4.1) */
    addr.fromString(STATIC_IP_CLIENT);
    wifiManager.setAPStaticIPConfig(addr,
                                    IPAddress(192,168,4,1),
                                    IPAddress(255,255,255,0));
#endif /* STATIC_IP_AP */
#ifdef STATIC_IP_CLIENT
    /* Static client IP */
    addr.fromString(STATIC_IP_CLIENT);
    wifiManager.setSTAStaticIPConfig(addr,
                                     IPAddress(192,168,1,1),
                                     IPAddress(255,255,255,0));
#endif /* STATIC_IP_CLIENT */

    wifiManager.setConfigPortalTimeout(WIFI_TIMEOUT);
    if (wifiManager.autoConnect(STASSID" ESP RX")) // start unsecure portal AP
    {
        addr = WiFi.localIP();
    }
    else
#elif defined(WIFI_SSID) && defined(WIFI_PSK)
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PSK);
  }
  uint32_t i = 0;
#define TIMEOUT (WIFI_TIMEOUT * 10)
  while (WiFi.status() != WL_CONNECTED)
      {
    delay(100);
    if (++i > TIMEOUT) {
      break;
    }
  }
  if (i < TIMEOUT) {
    addr = WiFi.localIP();
  } else
#endif /* WIFI_MANAGER */
    {
        // No wifi found, start AP
        WiFi.mode(WIFI_AP);
        WiFi.softAP(STASSID" ESP RX", STAPSK);
        addr = WiFi.softAPIP();
    }

    if (mdns.begin(host, addr))
    {
        mdns.addService("http", "tcp", 80);
        mdns.update();
    }

    httpUpdater.setup(&httpServer);
    httpServer.begin();

    DEBUG_PRINTF("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", host);
}

void HandleWebUpdate(void)
{
    httpServer.handleClient();
    mdns.update();
}
