# BlueShield-USV
**Smart Water Surface Drone for Real-Time Water Quality Monitoring & Heavy Metal Prediction**

BlueShield is an interdisciplinary final year project focused on addressing the growing problem of freshwater contamination, especially due to **heavy metals** like **arsenic, barium, and lithium**. This project combines **embedded systems, IoT, full stack and machine learning** to build a fully functional **water surface drone** capable of monitoring water quality in real-time and predicting heavy metal concentrations.

---

## Problem Statement

Millions of people are exposed to contaminated water with harmful heavy metals, posing severe health risks such as organ damage, cancer, and neurological issues. Traditional water testing methods are slow, manual, and lack real-time capabilities. BlueShield aims to provide an **automated, scalable, and intelligent** solution for **real-time detection and reporting of water quality and contamination levels**.

---
### 1. [`transmitter_combined.cpp`](./transmitter_combined.cpp)
This is the main firmware running on the **ESP32** microcontroller located on the **USV (drone)**.

**Responsibilities:**
- Collects real-time data from **pH, EC, TDS, DO, and Temperature sensors**
- Acquires **GPS location** data
- Sends data over **LoRa (433 MHz)** in a compact, structured format
- Periodically transmits sensor readings to the receiver module
