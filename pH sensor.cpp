#define PH_PIN 34  // ADC pin connected to pH sensor

float offset = 0.0;  // Adjust this after calibration

void setup() {
    Serial.begin(115200);
}

void loop() {
    int analogValue = analogRead(PH_PIN);  
    float voltage = analogValue * (3.3 / 4095.0); // Convert ADC reading to voltage
    float pHValue = 3.5 * voltage + offset;  // Convert voltage to pH using calibration

    Serial.print("Analog Value: "); Serial.print(analogValue);
    Serial.print(" | Voltage: "); Serial.print(voltage, 2);
    Serial.print(" | pH Value: "); Serial.println(pHValue, 2);

    delay(1000);
}
