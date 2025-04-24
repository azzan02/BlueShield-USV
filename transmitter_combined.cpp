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
#define TDS_PIN 32    
#define DO_PIN 33    
#define LED_PIN 2
#define ONE_WIRE_BUS 25  // DS18B20 for water temperature - MOVED to pin 25 to avoid conflict with TDS sensor

// ----- Constants -----
const float VREF = 3.3;
const int ADC_RES = 4095;
unsigned long lastPacketTime = 0;
const unsigned long packetInterval = 3000;

// ----- pH Calibration Parameters -----
// Calibration values from pH 10 and pH 4 solutions
const float pH_COEFFICIENT = -5.7980;  // Coefficient from calibration
const float pH_OFFSET = 21.3928;       // Offset from calibration

// ----- pH Sensor Variables -----
#define SAMPLE_SIZE 10  // Number of samples for averaging
float pH_samples[SAMPLE_SIZE];
int sample_index = 0;
unsigned long last_pH_reading = 0;
const unsigned long pH_interval = 20; // Read pH every 20ms for smoothing

float readPH(float voltage) {
  // Convert analog voltage to pH using the calibrated equation
  float phValue = pH_COEFFICIENT * voltage + pH_OFFSET;
  return phValue;
}

// Average pH readings to reduce noise
float getAveragePH() {
  float sum = 0;
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    sum += pH_samples[i];
  }
  return sum / SAMPLE_SIZE;
}

// ----- EC/TDS Calibration Constants -----
namespace device {
  const float aref = 3.3;
}

namespace calibration {
  // Updated with calibration results
  const float calVoltage = 0.036774;
  const float calEC = 84.00;
  const float kValue = 2284.215576;
  const float tdsFactor = 0.50;
  // Calculated slope for backwards compatibility
  const float ecSlope = calEC / calVoltage;
}

namespace sensor {
  float ec = 0.0;
  float tds = 0.0;
  float waterTemp = 25.0;
  
  // Variables for filtering EC/TDS readings
  const int EC_SAMPLE_SIZE = 5;
  float ec_samples[EC_SAMPLE_SIZE];
  int ec_sample_index = 0;
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
  if (temp != DEVICE_DISCONNECTED_C && temp > -127) {  // Added proper error check
    sensor::waterTemp = temp;
    Serial.print("Water temperature: ");
    Serial.println(sensor::waterTemp);
  } else {
    Serial.println("Error: DS18B20 not responding or invalid reading.");
  }
}

float getAverageEC() {
  float sum = 0;
  for (int i = 0; i < sensor::EC_SAMPLE_SIZE; i++) {
    sum += sensor::ec_samples[i];
  }
  return sum / sensor::EC_SAMPLE_SIZE;
}

void readTDS() {
  // Improved: Add multiple readings for averaging
  float voltage_sum = 0;
  for (int i = 0; i < 10; i++) {
    int adcValue = analogRead(TDS_PIN);
    voltage_sum += adcValue * device::aref / ADC_RES;
    delay(10);  // Small delay between readings
  }
  float voltage = voltage_sum / 10;  // Average of 10 readings
  
  // Calculate EC using calibrated kValue and temperature compensation
  float tempCoef = 1.0 + 0.02 * (sensor::waterTemp - 25.0);
  float rawEC = voltage * calibration::kValue;
  float compensatedEC = rawEC * tempCoef;
  
  // Store EC reading in rolling buffer
  sensor::ec_samples[sensor::ec_sample_index] = compensatedEC;
  sensor::ec_sample_index = (sensor::ec_sample_index + 1) % sensor::EC_SAMPLE_SIZE;
  
  // Get average EC
  sensor::ec = getAverageEC();
  
  // Calculate TDS from EC using calibrated TDS factor
  sensor::tds = sensor::ec * calibration::tdsFactor;
  
  Serial.print("Raw ADC: ");
  Serial.print(voltage);
  Serial.print("V, EC: ");
  Serial.print(sensor::ec);
  Serial.print(" μS/cm, TDS: ");
  Serial.print(sensor::tds);
  Serial.println(" ppm");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // Wait for serial to connect
  
  Serial.println("\nESP32 Water Quality Monitoring System");
  
  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Initialize pH sample array
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    pH_samples[i] = 7.0;  // Neutral pH as default
  }
  
  // Initialize EC sample array
  for (int i = 0; i < sensor::EC_SAMPLE_SIZE; i++) {
    sensor::ec_samples[i] = 0.0;
  }

  Serial.println("Initializing LoRa...");
  SPI.begin(5, 19, 27, 18);
  LoRa.setPins(SS, RST, DIO0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed. Check wiring!");
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(300);
      digitalWrite(LED_PIN, LOW);
      delay(300);
    }
  }
  Serial.println("LoRa Initialized Successfully!");

  LoRa.setSpreadingFactor(9);  
  LoRa.setSignalBandwidth(125E3);  
  LoRa.setCodingRate4(5);  
  LoRa.setPreambleLength(8);  
  LoRa.setTxPower(14);

  analogReadResolution(12);
  
  // Initialize temperature sensor
  sensors.begin();
  Serial.println("Initializing sensors...");
  updateWaterTemp();  // Get initial temperature reading
  
  // Test sensor readings
  Serial.println("Testing sensors...");
  int phAnalog = analogRead(PH_PIN);
  float phVoltage = phAnalog * (VREF / ADC_RES);
  Serial.print("pH raw analog: ");
  Serial.print(phAnalog);
  Serial.print(", Voltage: ");
  Serial.println(phVoltage);
  
  int tdsAnalog = analogRead(TDS_PIN);
  Serial.print("TDS raw analog: ");
  Serial.println(tdsAnalog);
  
  int doAnalog = analogRead(DO_PIN);
  Serial.print("DO raw analog: ");
  Serial.println(doAnalog);
  
  Serial.println("Setup complete.");
  delay(500);
}

void loop() {
  // Process GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  // Read pH values more frequently for better averaging
  if (millis() - last_pH_reading >= pH_interval) {
    last_pH_reading = millis();
    int phAnalog = analogRead(PH_PIN);
    float phVoltage = phAnalog * (VREF / ADC_RES);
    float ph = readPH(phVoltage);
    pH_samples[sample_index] = ph;
    sample_index = (sample_index + 1) % SAMPLE_SIZE;
  }

  // Send packet at specified interval
  if (millis() - lastPacketTime >= packetInterval) {
    lastPacketTime = millis();

    // Read sensors
    updateWaterTemp();
    readTDS();
    float pHValue = getAveragePH();
    
    // Read DO sensor
    int doAnalog = analogRead(DO_PIN);
    float doVoltage = doAnalog * (VREF / ADC_RES);
    // Adjusted DO calculation based on common conversion for DO sensors
    float doValue = ((doVoltage - 0.4) * 3.0);  // Adjust these values according to your DO sensor calibration
    
    // Format GPS data
    String gpsData = "";
    if (gps.location.isValid()) {
      gpsData = "Lat:" + String(gps.location.lat(), 6) + ",Lon:" + String(gps.location.lng(), 6) +
                ",Alt:" + String(gps.altitude.meters()) + ",Speed:" + String(gps.speed.kmph()) +
                ",Sat:" + String(gps.satellites.value());
    } else {
      gpsData = "GPS:NoFix";
      Serial.println("GPS No Fix. Still sending sensor data...");
    }

    // Prepare LoRa packet
    String dataToSend = "pH:" + String(pHValue, 2) +
                        ",EC:" + String(sensor::ec, 2) +
                        ",TDS:" + String(sensor::tds, 2) +
                        ",Temp:" + String(sensor::waterTemp, 2) +
                        ",DO:" + String(doValue, 2) + "," + gpsData;

    // Print data to Serial for debugging
    Serial.println("\n=== SENSOR DATA ===");
    Serial.println("pH: " + String(pHValue, 2));
    Serial.println("EC: " + String(sensor::ec, 2) + " μS/cm");
    Serial.println("TDS: " + String(sensor::tds, 2) + " ppm");
    Serial.println("Water Temp: " + String(sensor::waterTemp, 2) + "°C");
    Serial.println("DO: " + String(doValue, 2) + " mg/L");
    if (gps.location.isValid()) {
      Serial.println("GPS Location: " + String(gps.location.lat(), 6) + ", " + String(gps.location.lng(), 6));
    } else {
      Serial.println("GPS: No Fix");
    }
    Serial.println("==================");

    // Send data via LoRa
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
