#include <WiFi.h>
#include <WebServer.h>
#include <String.h>
#include <ArduinoJson.h>
#include "ibus.h"

const char* ssid = "DIGIFIBRA-Pu5f";
const char* password = "UX5ysFfU6P";
WebServer server(80);

IBusTx ibusTx;

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

  JsonArray data = doc.as<JsonArray>();
  ibusTx.prepareIbusPacket(&data, data.size());

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
  // Start Serial communication
  Serial.begin(115200);
  Serial2.begin(115200);
  ibusTx.setup();

  setupServer();
}

void loop() {
  server.handleClient();
  ibusTx.loop();
}