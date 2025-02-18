#include <SPI.h>
#include <LoRa.h>

#define SS 18
#define RST 14
#define DIO0 26

#define LED_PIN 2   // LED to blink when a packet is received

void setup() {
    Serial.begin(115200);
    
    pinMode(LED_PIN, OUTPUT);  // Set LED pin as output
    digitalWrite(LED_PIN, LOW); // Ensure LED is OFF initially

    Serial.println("Initializing LoRa Receiver...");
    SPI.begin(5, 19, 27, 18);
    LoRa.setPins(SS, RST, DIO0);

    if (!LoRa.begin(433E6)) {
        Serial.println("LoRa init failed. Check wiring!");
        while (1);
    }

    Serial.println("LoRa Receiver Initialized Successfully!");
    
    // LoRa settings for better reception
    LoRa.setSpreadingFactor(9);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(5);
    LoRa.setPreambleLength(8);
}

void loop() {
    int packetSize = LoRa.parsePacket(); // Check if a packet is received
    if (packetSize) {
        digitalWrite(LED_PIN, HIGH);  // Turn LED ON when packet is received

        Serial.println("Packet Received!");

        // Read incoming data
        String receivedData = "";
        while (LoRa.available()) {
            receivedData += (char)LoRa.read();
        }

        Serial.println("Received Data: " + receivedData);
        
        digitalWrite(LED_PIN, LOW);   // Turn LED OFF after processing packet
        delay(100);  // Short delay to make blink visible
    }
}
