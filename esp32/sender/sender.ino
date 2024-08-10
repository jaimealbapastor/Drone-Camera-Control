#include <esp_now.h>
#include <WiFi.h>


const uint8_t MAX_CHANNELS = 14;
uint16_t channels[MAX_CHANNELS];
uint8_t channel_i = 0;

const uint8_t broadcastAddress[] = { 0xcc, 0x7b, 0x5c, 0x25, 0x32, 0xd4 };
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print(F("\r\nLast Packet Send Status:\t"));
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void sendData2Receiver() {
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&channels, sizeof(channels));

  if (result == ESP_OK) {
    Serial.println(F("Sent with success"));
  } else {
    Serial.println(F("Error sending the data"));
  }
}

void setup() {

  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("Error initializing ESP-NOW"));
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println(F("Failed to add peer"));
    return;
  }
}

void loop() {
  static uint8_t buffer[2];  // Buffer to store incoming bytes
  static uint8_t buffer_i = 0;
  static bool data_coming = false;

  if (!data_coming && Serial.available() > 0 && Serial.read() == '\r')
    data_coming = true;

  // Receive from Serial
  if (data_coming) {
    while (Serial.available() > 0) {
      uint8_t in_byte = Serial.read();

      if (in_byte == '\n') {
        sendData2Receiver();
        memset(&channels, 0, sizeof(channels));
        channel_i = 0;
        data_coming = false;
        break;
      }

      buffer[buffer_i++] = in_byte;
      if (buffer_i == 2) {                                      // If we have received 2 bytes, we can form a uint16_t
        uint16_t receivedValue = (buffer[1] << 8) | buffer[0];  // Combine bytes
        channels[channel_i++] = receivedValue;
        buffer_i = 0;  // Reset index for next value
      }
    }
  }
}