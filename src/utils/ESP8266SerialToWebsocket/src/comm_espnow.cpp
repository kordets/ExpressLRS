#include "comm_espnow.h"
#include "main.h"
#include <ESP8266WiFi.h>
#include <espnow.h>


String espnow_init_info = "";


#if ESP_NOW

class CtrlSerialEspNow: public CtrlSerial
{
public:
  size_t available(void) {
    return 0; // not used
  }
  uint8_t read(void) {
    return 0; // not used
  }
  void write(uint8_t * buffer, size_t size) {
      esp_now_send(NULL, buffer, size);
      //websocket_send("MSP sent!");
  }
};

CtrlSerialEspNow esp_now_sender;

static void esp_now_recv_cb(uint8_t *mac_addr, uint8_t *data, uint8_t data_len)
{
  /* No data or peer is unknown => ignore */
  if (!data_len || !esp_now_is_peer_exist(mac_addr))
    return;

  websocket_send("ESP NOW message received!");

  // Pass data to ERLS
  // Note: accepts only correctly formatted MSP packets
  Serial.write((uint8_t*)data, data_len);
}

static void esp_now_send_cb(uint8_t *mac_addr, u8 status) {
#if 0
  String temp = "ESPNOW Sent: ";
  temp += (status ? "FAIL" : "SUCCESS");
  websocket_send(temp);
#endif
}

#endif // ESP_NOW


void espnow_init(uint32_t channel)
{
#if ESP_NOW
    espnow_init_info = "ESP NOW init... ";

    if (esp_now_init() != 0) {
        espnow_init_info += "ESP NOW init failed!";
        return;
    }
    esp_now_set_self_role(ESP_NOW_ROLE_COMBO);
    esp_now_register_send_cb(esp_now_send_cb);
    esp_now_register_recv_cb(esp_now_recv_cb);

#ifdef ESP_NOW_PEERS
    #define ESP_NOW_ETH_ALEN 6
    uint8_t peers[][ESP_NOW_ETH_ALEN] = ESP_NOW_PEERS;
    uint8_t num_peers = sizeof(peers) / ESP_NOW_ETH_ALEN;
    for (uint8_t iter = 0; iter < num_peers; iter++) {
        //esp_now_del_peer(peers[iter]);
        if (esp_now_add_peer(peers[iter], ESP_NOW_ROLE_COMBO, channel, NULL, 0) != 0) {
            espnow_init_info += ", PEER ";
            espnow_init_info += iter;
            espnow_init_info += " FAIL";
        }
    }
#endif // ESP_NOW_PEERS

    espnow_init_info += " - Init DONE!";
#endif // ESP_NOW
}

String & espnow_get_info()
{
    return espnow_init_info;
}

void espnow_send_msp(mspPacket_t &msp)
{
#if ESP_NOW
    MSP::sendPacket(&msp, &esp_now_sender);
#endif // ESP_NOW
}
