#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <sbus.h>

uint16_t swap_endian(uint16_t val) {
    return (val << 8) | (val >> 8);
}

const char* ssid = "DIGIFIBRA-Pu5f";
const char* password = "UX5ysFfU6P";
WebServer server(80);

bfs::SbusTx sbus_tx(&Serial2, 16, 17, true);
bfs::SbusData data;
uint8_t i_data = 0;

// Function to handle the "/controller" path
void handleController() {
  if (server.method() != HTTP_POST) {
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  // Deserialize JSON array
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  if (error) {
    server.send(400, "text/plain", "Bad JSON");
    return;
  }

  // Check the size of the array
  if (!doc.is<JsonArray>() || doc.size() < 4) {
    server.send(400, "text/plain", "Invalid data");
    return;
  }

  JsonArray arr = doc.as<JsonArray>();

Serial.print("[");
  i_data = 0;
  for (JsonVariant value : arr) {
    uint16_t val = value.as<uint16_t>();
    data.ch[i_data++] = val;
    Serial.print(val, DEC);
    Serial.print(",");
  }
  Serial.println("]");
  sbus_tx.data(data);
  sbus_tx.Write();

  server.send(200, "text/plain", "Received");
}

void setupServer() {
  if (!Serial)
    Serial.begin(115200);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Print the IP address
  Serial.println(WiFi.localIP());

  // Define routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/controller", HTTP_POST, handleController);
  server.onNotFound(handleNotFound);

  // Start the server
  server.begin();
  Serial.println("HTTP server started");
}

void handleRoot() {
  server.send(200, "text/plain", "Welcome to the ESP32 Web Server!");
}

void handleNotFound() {
  server.send(404, "text/plain", "404: Not found");
}

void setup() {
  Serial.begin(115200);
  
  sbus_tx.Begin();

  data.lost_frame = false;
  data.failsafe = false;
  data.ch17 = false;
  data.ch18 = false;

  setupServer();
}

void loop() {
  server.handleClient();
}