#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// Replace with your network credentials
const char* ssid = "DIGIFIBRA-Pu5f";
const char* password = "UX5ysFfU6P";

WiFiServer server(80);

void setup() {
  Serial.begin(115200);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server.begin();
}

void loop() {
  WiFiClient client = server.available();
  if (client) {
    Serial.println("New Client.");
    String currentLine = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        Serial.write(c);
        if (c == '\n') {
          if (currentLine.length() == 0) {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.println("<!DOCTYPE HTML>");
            client.println("<html>");
            client.println("<form action=\"/\" method=\"get\">");
            client.println("<input type=\"submit\" name=\"command\" value=\"LED_ON\">");
            client.println("<input type=\"submit\" name=\"command\" value=\"LED_OFF\">");
            client.println("</form>");
            client.println("</html>");
            client.println();
            break;
          } else {
            if (currentLine.startsWith("GET /?command=LED_ON")) {
              Serial.println("Command: LED_ON");
              // Handle LED_ON command
            } else if (currentLine.startsWith("GET /?command=LED_OFF")) {
              Serial.println("Command: LED_OFF");
              // Handle LED_OFF command
            }
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    client.stop();
    Serial.println("Client Disconnected.");
  }
}
