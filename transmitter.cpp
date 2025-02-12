//transmitter code

#include <SPI.h>
#include <LoRa.h>

#define SS 18
#define RST 14
#define DIO0 26




void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("Starting LoRa...");

    SPI.begin(5, 19, 27, 18);
    LoRa.setPins(SS, RST, DIO0);

    if (!LoRa.begin(920E6)) {
        Serial.println("LoRa init failed. Check wiring!");
        while (1);
    }

    Serial.println("LoRa Initialized Successfully!");
}

void loop() {

   LoRa.setSpreadingFactor(12);   // Max SF for best range
    LoRa.setSignalBandwidth(62.5E3);  // Narrow bandwidth for better range
    LoRa.setCodingRate4(8);   // Max error correction
    LoRa.setPreambleLength(12);  // Improve reception
    LoRa.setTxPower(20, PA_OUTPUT_PA_BOOST_PIN);  // Max transmission power

    Serial.println("Sending packet...");

    LoRa.beginPacket();
    LoRa.print("Hello from ESP32");
    LoRa.endPacket();  // Send the packet

    Serial.println("Packet Sent!");
    delay(2000);  // Send message every 2 seconds
}

