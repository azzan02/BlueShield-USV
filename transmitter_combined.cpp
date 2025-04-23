#include <SPI.h>
#include <LoRa.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ----- Pin Definitions -----
#define SS 18
#define RST 14
#define DIO0 26
#define PH_PIN 34    
#define TDS_PIN 32    // EC and TDS share the same analog pin
#define DO_PIN 33    
#define LED_PIN 2
#define ONE_WIRE_BUS 32  // DS18B20 for water temperature

// ----- Constants -----
const float VREF = 3.3;
const int ADC_RES = 4095;
unsigned long lastPacketTime = 0;
const unsigned long packetInterval = 3000;

// ----- pH Calibration Parameters -----
float readPH(float voltage) {
  float voltage_pH4 = 3.060;
  float voltage_pH10 = 1.948;
  float m = (10.0 - 4.0) / (voltage_pH10 - voltage_pH4);
  float b = 4.0 - m * voltage_pH4;
  return m * voltage + b;
}

// ----- EC/TDS Calibration Constants -----
namespace device {
  const float aref = 3.3;
}
namespace calibration {
  const float calVoltage = 0.0715;
  const float calEC = 84.0;
  const float ecSlope = calEC / calVoltage;
}
namespace sensor {
  float ec = 0.0;
  float tds = 0.0;
  float waterTemp = 25.0;
}

// ----- GPS -----
static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

// ----- Temp Sensor Setup -----
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void updateWaterTemp() {
  sensors.requestTemperatures();
  float temp = sensors.getTempCByIndex(0);
  if (temp != DEVICE_DISCONNECTED_C) {
    sensor::waterTemp = temp;
  } else {
    Serial.println("Error: DS18B20 not responding.");
  }
}

void readTDS() {
  int adcValue = analogRead(TDS_PIN);
  float voltage = adcValue * device::aref / ADC_RES;
  float rawEC = voltage * calibration::ecSlope;
  float tempCoef = 1.0 + 0.02 * (sensor::waterTemp - 25.0);
  sensor::ec = rawEC / tempCoef;
  sensor::tds = sensor::ec * 0.5;
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println("Initializing LoRa...");
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed. Check wiring!");
    while (1);
  }
  Serial.println("LoRa Initialized Successfully!");

  LoRa.setSpreadingFactor(9);  
  LoRa.setSignalBandwidth(125E3);  
  LoRa.setCodingRate4(5);  
  LoRa.setPreambleLength(8);  
  LoRa.setTxPower(14);

  analogReadResolution(12);
  sensors.begin();
  delay(500);
}

void loop() {
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (millis() - lastPacketTime >= packetInterval) {
    lastPacketTime = millis();

    String gpsData = "";
    if (gps.location.isValid()) {
      gpsData = "Lat:" + String(gps.location.lat(), 6) + ",Lon:" + String(gps.location.lng(), 6) +
                ",Alt:" + String(gps.altitude.meters()) + ",Speed:" + String(gps.speed.kmph()) +
                ",Sat:" + String(gps.satellites.value());
    } else {
      gpsData = "GPS:NoFix";
      Serial.println("GPS No Fix. Still sending sensor data...");
    }

    // Update sensors
    updateWaterTemp();
    readTDS();
    int phAnalog = analogRead(PH_PIN);
    float phVoltage = phAnalog * (VREF / ADC_RES);
    float pHValue = readPH(phVoltage);
    int doAnalog = analogRead(DO_PIN);
    float doValue = ((doAnalog * (VREF / ADC_RES)) - 0.4) * 3.0;

    // Prepare LoRa packet
    String dataToSend = "pH:" + String(pHValue, 2) +
                        ",EC:" + String(sensor::ec, 2) +
                        ",TDS:" + String(sensor::tds, 2) +
                        ",Temp:" + String(sensor::waterTemp, 2) +
                        ",DO:" + String(doValue, 2) + "," + gpsData;

    Serial.println("Sending packet: " + dataToSend);
    digitalWrite(LED_PIN, HIGH);
    LoRa.beginPacket();
    LoRa.print(dataToSend);
    LoRa.endPacket();
    digitalWrite(LED_PIN, LOW);
    delay(100);
    Serial.println("Packet Sent!");
  }
}
