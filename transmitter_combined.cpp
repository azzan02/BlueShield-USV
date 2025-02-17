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
#define DO_PIN 33    // Dissolved Oxygen Sensor Pin

const float VREF = 3.3;
const int ADC_RES = 4095;
float ph_offset = 0.0;
float ec_offset = 0.0;
float tds_factor = 0.5;
float do_offset = 0.0; // Adjust based on sensor calibration

// GPS Configuration
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
    
    Serial.println("Starting LoRa...");
    SPI.begin(5, 19, 27, 18);
    LoRa.setPins(SS, RST, DIO0);

    if (!LoRa.begin(433E6)) {
        Serial.println("LoRa init failed. Check wiring!");
        while (1);
    }
    Serial.println("LoRa Initialized Successfully!");
}

void loop() {
    // Read sensor values
    int phAnalog = analogRead(PH_PIN);
    int tdsAnalog = analogRead(TDS_PIN);
    int ecAnalog = analogRead(EC_PIN);
    int doAnalog = analogRead(DO_PIN);

    // Convert analog readings to voltage
    float phVoltage = phAnalog * (VREF / ADC_RES);
    float tdsVoltage = tdsAnalog * (VREF / ADC_RES);
    float ecVoltage = ecAnalog * (VREF / ADC_RES);
    float doVoltage = doAnalog * (VREF / ADC_RES);

    // Convert voltage to meaningful sensor values
    float pHValue = 3.5 * phVoltage + ph_offset;
    float ecValue = (ecVoltage - 0.4) / 0.19 + ec_offset;
    float tdsValue = ecValue * tds_factor;
    float doValue = (doVoltage - 0.4) * 3.0 + do_offset; // Adjust based on sensor calibration

    // Read GPS data
    String gpsData = "";
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
    }
    
    if (gps.location.isValid()) {
        gpsData = "Lat:" + String(gps.location.lat(), 6) + ",Lon:" + String(gps.location.lng(), 6) + ",Alt:" + String(gps.altitude.meters()) + ",Speed:" + String(gps.speed.kmph()) + ",Satellites:" + String(gps.satellites.value()) + ",Date:" + String(gps.date.day()) + "/" + String(gps.date.month()) + "/" + String(gps.date.year()) + ",Time:" + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
    } else {
        gpsData = "GPS:NoFix";
    }

    // Prepare data packet
    String dataToSend = "pH:" + String(pHValue) + ",EC:" + String(ecValue) + ",TDS:" + String(tdsValue) + ",DO:" + String(doValue) + "," + gpsData;
    
    // LoRa settings for optimal range
    LoRa.setSpreadingFactor(12);
    LoRa.setSignalBandwidth(62.5E3);
    LoRa.setCodingRate4(8);
    LoRa.setPreambleLength(12);
    LoRa.setTxPower(20);

    Serial.println("Sending packet: " + dataToSend);
    
    // Transmit data over LoRa
    LoRa.beginPacket();
    LoRa.print(dataToSend);
    LoRa.endPacket();
    
    Serial.println("Packet Sent!");
    delay(2000);  // Send message every 2 seconds
}
