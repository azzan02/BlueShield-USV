#define PH_PIN 34   // pH sensor connected to GPIO34
#define TDS_PIN 35  // TDS sensor connected to GPIO35

const float VREF = 3.3;    // ESP32 ADC reference voltage
const int ADC_RES = 4095;  // ESP32 ADC resolution (12-bit)
float temperature = 25.0;  // Default temperature (change if using a temp sensor)

float ph_offset = 0.0;  // Adjust this after calibration
float tds_factor = 0.5; // Calibration factor for TDS sensor

void setup() {
    Serial.begin(115200);
}

void loop() {
    // Read analog values
    int phAnalog = analogRead(PH_PIN);
    int tdsAnalog = analogRead(TDS_PIN);

    // Convert ADC reading to voltage
    float phVoltage = phAnalog * (VREF / ADC_RES);
    float tdsVoltage = tdsAnalog * (VREF / ADC_RES);

    // Convert voltage to pH value (use calibrated equation)
    float pHValue = 3.5 * phVoltage + ph_offset;

    // Convert voltage to TDS (based on calibration)
    float ecValue = (tdsVoltage / VREF) * 1000;  // Convert to EC (µS/cm)
    float tdsValue = ecValue * tds_factor;  // Convert EC to TDS (ppm)

    // Print values
    Serial.print("pH Value: "); Serial.print(pHValue, 2);
    Serial.print(" | TDS Value: "); Serial.print(tdsValue, 2);
    Serial.println(" ppm");

    delay(1000);
}
