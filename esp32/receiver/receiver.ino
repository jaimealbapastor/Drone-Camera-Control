#include <esp_now.h>
#include <WiFi.h>
#include <sbus.h>

bfs::SbusTx sbus_tx(&Serial2, 16, 17, true);
bfs::SbusData data;

const uint8_t MAX_CHANNELS = 14;
uint16_t channels[MAX_CHANNELS];

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  memcpy(&channels, incomingData, sizeof(channels));
  // Serial.print("Bytes received: ");
  // Serial.println(len);
  // Serial.print("Ch: ");
  // for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
  //   Serial.print(channels[i], DEC);
  //   Serial.print(",");
  // }
  // Serial.println("");

  // memset(&data.ch, 0, sizeof(data.ch));
  memcpy(&data.ch, &channels, sizeof(channels));
  sbus_tx.data(data);
  sbus_tx.Write();
}

void setup() {
  // ESPNOW communication
  // Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    // Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  // SBUS communication
  sbus_tx.Begin();

  data.lost_frame = false;
  data.failsafe = false;
  data.ch17 = false;
  data.ch18 = false;
  memset(data.ch, 0, sizeof(data.ch));

  memset(&channels, 0, sizeof(channels));
}

void loop() {
}