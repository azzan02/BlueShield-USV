#include <SPI.h>
#include <LoRa.h>
#include <WiFi.h>
#include <PubSubClient.h> // MQTT Client library

// LoRa Module Pins
#define SS 18
#define RST 14
#define DIO0 26
#define LED_PIN 2   // LED to blink when a packet is received

// WiFi Configuration
const char* ssid = "AzzaniPhone";
const char* password = "Azzan123";

// Static IP Configuration (based on your network settings)
IPAddress staticIP(172, 20, 10, 20);   // Static IP for ESP32 (different from your PC's IP)
IPAddress gateway(172, 20, 10, 1);     // Gateway (from your network config)
IPAddress subnet(255, 255, 255, 0);    // Subnet mask
IPAddress dns(172, 20, 10, 1);         // DNS server (same as gateway for hotspot)

// MQTT Configuration
const char* mqtt_server = "172.20.10.9"; 
const int mqtt_port = 1883;
const char* mqtt_topic = "water_sensors/data";
const char* mqtt_client_id = "ESP32_LoRa_Receiver";
const char* mqtt_username = ""; // If authentication is required
const char* mqtt_password = ""; // If authentication is required

// Initialize WiFi and MQTT client
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Connection status tracking
unsigned long lastReconnectAttempt = 0;
const unsigned long reconnectInterval = 5000; // Attempt reconnect every 5 seconds
const int maxWiFiAttempts = 5; // Maximum number of WiFi connection attempts before giving up
const unsigned long wifiConnectionTimeout = 15000; // 15 seconds per connection attempt

// Status and operational variables
bool loraInitialized = false;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(100); // Wait for serial connection
    
    Serial.println("\n\n==== ESP32 LoRa to MQTT Gateway ====");
    
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    
    // Initialize LoRa
    initializeLoRa();
    
    // Initialize WiFi with robust connection handling
    setupWiFi();
    
    // Initialize MQTT
    mqttClient.setServer(mqtt_server, mqtt_port);
    
    // Attempt initial MQTT connection
    connectMQTT();
    
    Serial.println("Setup complete, starting main loop...");
    
    // Blink LED to indicate successful initialization
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

void loop() {
    // Handle WiFi connection monitoring
    checkWiFiConnection();
    
    // Ensure MQTT connection
    if (!mqttClient.connected()) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastReconnectAttempt > reconnectInterval) {
            lastReconnectAttempt = currentMillis;
            connectMQTT();
        }
    } else {
        mqttClient.loop(); // Process MQTT messages
    }
    
    // Only process LoRa packets if LoRa is initialized
    if (loraInitialized) {
        // Check for incoming LoRa packets
        int packetSize = LoRa.parsePacket();
        if (packetSize) {
            processLoRaPacket(packetSize);
        }
    } else {
        // Attempt to reinitialize LoRa if it failed during setup
        static unsigned long lastLoRaAttempt = 0;
        if (millis() - lastLoRaAttempt > 60000) { // Try every minute
            lastLoRaAttempt = millis();
            initializeLoRa();
        }
    }
}

void initializeLoRa() {
    Serial.println("Initializing LoRa Receiver...");
    SPI.begin(5, 19, 27, 18);
    LoRa.setPins(SS, RST, DIO0);
    
    // Attempt to initialize LoRa
    int attempts = 0;
    const int maxAttempts = 3;
    while (!loraInitialized && attempts < maxAttempts) {
        attempts++;
        Serial.print("LoRa init attempt ");
        Serial.print(attempts);
        Serial.print("/");
        Serial.print(maxAttempts);
        Serial.println("...");
        
        if (LoRa.begin(433E6)) {
            Serial.println("LoRa Receiver Initialized Successfully!");
            
            // Configure LoRa parameters
            LoRa.setSpreadingFactor(9);
            LoRa.setSignalBandwidth(125E3);
            LoRa.setCodingRate4(5);
            LoRa.setPreambleLength(8);
            
            loraInitialized = true;
            break;
        } else {
            Serial.println("LoRa init failed. Check wiring!");
            delay(2000); // Wait before retrying
        }
    }
    
    if (!loraInitialized) {
        Serial.println("Failed to initialize LoRa after multiple attempts");
    }
}

void processLoRaPacket(int packetSize) {
    digitalWrite(LED_PIN, HIGH);
    
    // Read signal strength
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    
    Serial.print("Packet Received! RSSI: ");
    Serial.print(rssi);
    Serial.print(" dBm, SNR: ");
    Serial.print(snr);
    Serial.println(" dB");
    
    // Read incoming data
    String receivedData = "";
    while (LoRa.available()) {
        receivedData += (char)LoRa.read();
    }
    Serial.println("Received Data: " + receivedData);
    
    // Add RSSI and SNR to the data
    String enrichedData = "{\"data\":\"" + receivedData + "\",\"rssi\":" + String(rssi) + ",\"snr\":" + String(snr) + "}";
    
    // Publish to MQTT
    if (mqttClient.connected()) {
        if (mqttClient.publish(mqtt_topic, enrichedData.c_str())) {
            Serial.println("Data published to MQTT topic: " + String(mqtt_topic));
        } else {
            Serial.println("Failed to publish to MQTT");
            // Force reconnect on next loop iteration
            mqttClient.disconnect();
            lastReconnectAttempt = 0;
        }
    } else {
        Serial.println("MQTT not connected. Data not published.");
    }
    
    digitalWrite(LED_PIN, LOW);
    delay(100);
}

void setupWiFi() {
    int attempts = 0;
    
    // Reset WiFi connection
    WiFi.disconnect(true);
    WiFi.mode(WIFI_STA);
    
    // Disable WiFi sleep mode to improve stability
    WiFi.setSleep(false);
    
    delay(1000);
    
    Serial.println("Connecting to WiFi network...");
    Serial.print("SSID: ");
    Serial.println(ssid);
    Serial.print("Setting static IP: ");
    Serial.println(staticIP.toString());
    
    while (WiFi.status() != WL_CONNECTED && attempts < maxWiFiAttempts) {
        attempts++;
        Serial.print("WiFi connection attempt ");
        Serial.print(attempts);
        Serial.print("/");
        Serial.print(maxWiFiAttempts);
        Serial.println("...");
        
        // Configure static IP before connecting
        if (!WiFi.config(staticIP, gateway, subnet, dns)) {
            Serial.println("Static IP configuration failed!");
        } else {
            Serial.println("Static IP configured successfully");
        }
        
        // Begin connection attempt
        WiFi.begin(ssid, password);
        
        unsigned long startAttemptTime = millis();
        unsigned long elapsedTime = 0;
        int dots = 0;
        
        // Wait for connection or timeout with progress display
        while (WiFi.status() != WL_CONNECTED && 
               elapsedTime < wifiConnectionTimeout) {
            delay(500);
            Serial.print(".");
            elapsedTime = millis() - startAttemptTime;
            dots++;
            
            // Print extra debugging info every 5 seconds
            if (dots % 10 == 0) {
                Serial.print("\nWiFi status: ");
                Serial.print(WiFi.status());
                Serial.print(" (");
                
                switch (WiFi.status()) {
                    case WL_IDLE_STATUS:
                        Serial.print("IDLE");
                        break;
                    case WL_NO_SSID_AVAIL:
                        Serial.print("NO SSID AVAILABLE");
                        break;
                    case WL_SCAN_COMPLETED:
                        Serial.print("SCAN COMPLETED");
                        break;
                    case WL_CONNECT_FAILED:
                        Serial.print("CONNECTION FAILED");
                        break;
                    case WL_CONNECTION_LOST:
                        Serial.print("CONNECTION LOST");
                        break;
                    case WL_DISCONNECTED:
                        Serial.print("DISCONNECTED");
                        break;
                    default:
                        Serial.print("UNKNOWN");
                }
                
                Serial.println(")");
                Serial.print("Continuing attempt: ");
            }
        }
        
        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("");
            Serial.println("WiFi connected successfully!");
            Serial.print("IP address: ");
            Serial.println(WiFi.localIP());
            return;
        } else {
            Serial.print("\nWiFi connection failed, status code: ");
            Serial.print(WiFi.status());
            
            // Print human-readable error
            switch (WiFi.status()) {
                case WL_IDLE_STATUS:
                    Serial.println(" (IDLE)");
                    break;
                case WL_NO_SSID_AVAIL:
                    Serial.println(" (NO SSID AVAILABLE)");
                    break;
                case WL_SCAN_COMPLETED:
                    Serial.println(" (SCAN COMPLETED)");
                    break;
                case WL_CONNECT_FAILED:
                    Serial.println(" (CONNECTION FAILED)");
                    break;
                case WL_CONNECTION_LOST:
                    Serial.println(" (CONNECTION LOST)");
                    break;
                case WL_DISCONNECTED:
                    Serial.println(" (DISCONNECTED)");
                    break;
                default:
                    Serial.println(" (UNKNOWN)");
            }
            
            // Wait before retrying
            Serial.println("Waiting before retry...");
            delay(3000);
            
            // Reset WiFi for next attempt
            WiFi.disconnect(true);
            delay(1000);
        }
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Failed to connect to WiFi after multiple attempts");
        // Fall back to creating an access point mode if client mode fails
        setupAccessPoint();
    }
}

void setupAccessPoint() {
    Serial.println("Setting up ESP32 as an Access Point...");
    
    WiFi.disconnect(true);
    delay(1000);
    WiFi.mode(WIFI_AP);
    
    // Create access point with name "ESP32-LoRa-Gateway"
    if(WiFi.softAP("ESP32-LoRa-Gateway", "password123")) {
        Serial.println("Access Point started successfully!");
        Serial.print("AP IP address: ");
        Serial.println(WiFi.softAPIP());
        
        // Blink LED to indicate AP mode
        for (int i = 0; i < 5; i++) {
            digitalWrite(LED_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            delay(200);
        }
    } else {
        Serial.println("Access Point setup failed!");
    }
}

void checkWiFiConnection() {
    static unsigned long lastWiFiCheck = 0;
    static bool wasConnected = false;
    
    // Check WiFi status every 30 seconds
    if (millis() - lastWiFiCheck > 30000) {
        lastWiFiCheck = millis();
        
        if (WiFi.getMode() == WIFI_STA) {  // Only check if in station mode
            if (WiFi.status() != WL_CONNECTED) {
                if (wasConnected) {
                    Serial.println("WiFi connection lost! Attempting to reconnect...");
                    wasConnected = false;
                }
                setupWiFi(); // Try to reconnect
            } else if (!wasConnected) {
                Serial.println("WiFi reconnected!");
                Serial.print("IP address: ");
                Serial.println(WiFi.localIP());
                wasConnected = true;
                
                // If WiFi reconnected, also try to reconnect MQTT
                connectMQTT();
            }
        }
    }
}

boolean connectMQTT() {
    // Don't try to connect if WiFi is not connected in station mode
    if (WiFi.getMode() != WIFI_STA || WiFi.status() != WL_CONNECTED) {
        Serial.println("Cannot connect to MQTT: WiFi not connected in station mode");
        return false;
    }
    
    Serial.print("Connecting to MQTT broker at ");
    Serial.print(mqtt_server);
    Serial.print(":");
    Serial.print(mqtt_port);
    Serial.println("...");
    
    // Set a longer timeout for MQTT connection attempts
    mqttClient.setSocketTimeout(30); // Increase timeout to 30 seconds
    
    // Set MQTT keep alive interval
    mqttClient.setKeepAlive(60);
    
    // Set client ID with random number to avoid duplicate client issues
    String clientId = mqtt_client_id;
    clientId += "-";
    clientId += String(random(0xffff), HEX);
    
    Serial.print("Using client ID: ");
    Serial.println(clientId);
    
    if (mqtt_username[0] != '\0') {
        // If username is provided, use credentials
        if (mqttClient.connect(clientId.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Connected to MQTT broker with authentication!");
            return true;
        }
    } else {
        // Connect without credentials
        Serial.println("Attempting connection without credentials...");
        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("Connected to MQTT broker!");
            return true;
        }
    }
    
    Serial.print("MQTT connection failed, error code: ");
    Serial.print(mqttClient.state());
    
    // Print human-readable MQTT error
    switch (mqttClient.state()) {
        case -4:
            Serial.println(" (Connection timeout)");
            break;
        case -3:
            Serial.println(" (Connection lost)");
            break;
        case -2:
            Serial.println(" (Connection failed)");
            Serial.println("This usually means the broker is not reachable. Check if:");
            Serial.println("1. The broker IP address is correct");
            Serial.println("2. The broker is running and accessible from your network");
            Serial.println("3. No firewall is blocking port 1883");
            break;
        case -1:
            Serial.println(" (Disconnected)");
            break;
        case 1:
            Serial.println(" (Bad protocol)");
            break;
        case 2:
            Serial.println(" (Bad client ID)");
            break;
        case 3:
            Serial.println(" (Unavailable)");
            break;
        case 4:
            Serial.println(" (Bad credentials)");
            break;
        case 5:
            Serial.println(" (Unauthorized)");
            break;
        default:
            Serial.println(" (Unknown error)");
    }
    
    return false;
}
