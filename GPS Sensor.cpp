#include <HardwareSerial.h>
#include <TinyGPS++.h>

static const int RXPin = 16, TXPin = 17; // Adjust according to your ESP32
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

void setup() {
    Serial.begin(115200);
    gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);
}

void loop() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
        
        if (gps.location.isUpdated()) {
            Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
            Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
            Serial.print("Altitude: "); Serial.println(gps.altitude.meters());
            Serial.print("Speed: "); Serial.println(gps.speed.kmph());
            Serial.print("Satellites: "); Serial.println(gps.satellites.value());
            Serial.print("Date: "); Serial.print(gps.date.day()); Serial.print("/");
            Serial.print(gps.date.month()); Serial.print("/"); Serial.println(gps.date.year());
            Serial.print("Time: "); Serial.print(gps.time.hour()); Serial.print(":");
            Serial.print(gps.time.minute()); Serial.print(":"); Serial.println(gps.time.second());
            Serial.println();
        }
    }
}
