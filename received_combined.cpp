#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <HTTPClient.h>

// --- Wi-Fi credentials ---
const char* ssid = "AzzaniPhone";
const char* password = "Azzan123";

// --- Flask API endpoint ---
const char* serverURL = "http://172.20.10.9:5000/lora";  // Replace with your actual server URL

// --- LoRa configuration ---
#define SS 18
#define RST 14
#define DIO0 26

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi!");

  // Init LoRa
  SPI.begin(5, 19, 27, 18);  // SCK, MISO, MOSI, SS
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed. Check wiring!");
    while (1);
  }

  // Match transmitter LoRa settings
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setTxPower(14);

  Serial.println("LoRa Receiver ready!");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }

    Serial.println("Received via LoRa: " + received);

    // Forward to Flask API
    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      http.begin(serverURL);
      http.addHeader("Content-Type", "application/json");

      String jsonPayload = "{\"message\": \"" + received + "\"}";

      int httpCode = http.POST(jsonPayload);
      Serial.print("HTTP Response code: ");
      Serial.println(httpCode);

      http.end();
    } else {
      Serial.println("WiFi Disconnected!");
    }
  }
}
