#include <SPI.h>
#include <LoRa.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#define SS 18
#define RST 14
#define DIO0 26
#define PH_PIN 34    
#define TDS_PIN 35   
#define EC_PIN 32    
#define DO_PIN 33    
#define LED_PIN 2   // LED Pin (Use GPIO2 for built-in LED on ESP32)

const float VREF = 3.3;
const int ADC_RES = 4095;
float ph_offset = 0.0;
float ec_offset = 0.0;
float tds_factor = 0.5;
float do_offset = 0.0;
// GPS Configuration
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
unsigned long lastPacketTime = 0;
const unsigned long packetInterval = 3000; // Send every 3 sec
void setup() {
    Serial.begin(115200);
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    pinMode(LED_PIN, OUTPUT);  // Set LED pin as output
    digitalWrite(LED_PIN, LOW); // Ensure LED is OFF initially
    Serial.println("Initializing LoRa...");
    SPI.begin(5, 19, 27, 18);
    LoRa.setPins(SS, RST, DIO0);
    if (!LoRa.begin(433E6)) {
        Serial.println("LoRa init failed. Check wiring!");
        while (1);
    }
    Serial.println("LoRa Initialized Successfully!");
    // LoRa transmission settings (optimized for speed)
    LoRa.setSpreadingFactor(9);  
    LoRa.setSignalBandwidth(125E3);  
    LoRa.setCodingRate4(5);  
    LoRa.setPreambleLength(8);  
    LoRa.setTxPower(14);  
}
void loop() {
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());  // Read GPS data in real-time
    }
    // Send data at a set interval
    if (millis() - lastPacketTime >= packetInterval) {
        lastPacketTime = millis();  // Update last send time
        
        // Prepare GPS data
        String gpsData = "";
        if (gps.location.isValid()) {
            gpsData = "Lat:" + String(gps.location.lat(), 6) + ",Lon:" + String(gps.location.lng(), 6) + ",Alt:" + String(gps.altitude.meters()) + ",Speed:" + String(gps.speed.kmph()) + ",Sat:" + String(gps.satellites.value());
        } else {
            gpsData = "GPS:NoFix";  // Include a marker that GPS has no fix
            Serial.println("GPS No Fix. Still sending sensor data...");
        }
        
        // Read all sensor values simultaneously
        int phAnalog = analogRead(PH_PIN);
        int tdsAnalog = analogRead(TDS_PIN);
        int ecAnalog = analogRead(EC_PIN);
        int doAnalog = analogRead(DO_PIN);
        
        // Convert to voltage & sensor values
        float pHValue = 3.5 * (phAnalog * (VREF / ADC_RES)) + ph_offset;
        float ecValue = ((ecAnalog * (VREF / ADC_RES)) - 0.4) / 0.19 + ec_offset;
        float tdsValue = ecValue * tds_factor;
        float doValue = ((doAnalog * (VREF / ADC_RES)) - 0.4) * 3.0 + do_offset;
        
        // Prepare data packet
        String dataToSend = "pH:" + String(pHValue) + ",EC:" + String(ecValue) + ",TDS:" + String(tdsValue) + ",DO:" + String(doValue) + "," + gpsData;
        Serial.println("Sending packet: " + dataToSend);
        
        digitalWrite(LED_PIN, HIGH);  // Turn LED ON before sending packet
        // Send LoRa packet
        LoRa.beginPacket();
        LoRa.print(dataToSend);
        LoRa.endPacket();
        digitalWrite(LED_PIN, LOW);   // Turn LED OFF after sending packet
        delay(100);  // Short delay to make blink visible
        
        Serial.println("Packet Sent!");
    }
}
