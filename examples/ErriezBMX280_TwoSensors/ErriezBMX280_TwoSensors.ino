/*
 * MIT License
 *
 * Copyright (c) 2020 Erriez, modifications (c) 2025 Your Name/Org
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/*!
 * \file ErriezBMX280_TwoSensors.ino
 * \brief Example using two BMP/BME280 sensors on separate I2C buses (Wire and Wire1) on ESP32.
 * \details
 *   Requires an ESP32 board.
 *   Connect one sensor to the default I2C pins (Wire - typically GPIO 21/SDA, 22/SCL).
 *   Connect the second sensor to the secondary I2C pins (Wire1 - typically GPIO 32/SDA, 33/SCL, check your board).
 *   Assumes both sensors use I2C address 0x76 (possible because they are on different buses).
 * \details
 *     Source:          https://github.com/Erriez/ErriezBMX280
 *     Documentation:   https://erriez.github.io/ErriezBMX280
 */

#include <Wire.h>
#include <ErriezBMX280.h>

// Adjust sea level for altitude calculation
#define SEA_LEVEL_PRESSURE_HPA      1013.25 // Standard sea level pressure

// Define I2C address (assuming both sensors use 0x76, but on different buses)
#define BMX280_I2C_ADDR             0x76

// --- Sensor 1 (using default Wire) ---
ErriezBMX280 bmx_wire0(BMX280_I2C_ADDR); // Uses default Wire object (Wire)

// --- Sensor 2 (using Wire1) ---
// ESP32 has a second hardware I2C interface (Wire1)
// Default pins for Wire1 might vary, check your ESP32 board documentation.
// Common defaults are SDA=GPIO 32, SCL=GPIO 33 or SDA=GPIO 25, SCL=GPIO 26
// You might need to specify pins in Wire1.begin(), e.g., Wire1.begin(SDA1_PIN, SCL1_PIN);
ErriezBMX280 bmx_wire1(BMX280_I2C_ADDR, &Wire1); // Uses Wire1 object


void setup()
{
    // Initialize serial
    delay(500);
    Serial.begin(115200);
    while (!Serial) {
        ;
    }
    Serial.println(F("\nErriez BMP280/BME280 Two Sensors Example (ESP32)"));

    // --- Initialize I2C Bus 0 (Wire) ---
    Wire.begin(); // Use default SDA/SCL pins (e.g., GPIO 21, 22)
    Wire.setClock(400000);
    Serial.println(F("I2C Bus 0 (Wire) Initialized."));

    // --- Initialize Sensor 1 (Wire) ---
    Serial.print(F("Initializing Sensor 1 (Wire)... "));
    if (!bmx_wire0.begin()) {
        Serial.println(F("Error: Could not detect Sensor 1"));
        while(1) delay(100); // Halt on error
    } else {
        Serial.print(F("Detected "));
        Serial.print((bmx_wire0.getChipID() == CHIP_ID_BME280) ? "BME280" : "BMP280");
        Serial.println(F(" on Wire."));
        bmx_wire0.setSampling(); // Use default settings or customize
    }

    // --- Initialize I2C Bus 1 (Wire1) ---
    // Specify pins for Wire1 if not using defaults, e.g.:
    // int sda1_pin = 32;
    // int scl1_pin = 33;
    // Wire1.begin(sda1_pin, scl1_pin);
    Wire1.begin(); // Use default Wire1 pins for your board
    Wire1.setClock(400000);
    Serial.println(F("I2C Bus 1 (Wire1) Initialized."));

    // --- Initialize Sensor 2 (Wire1) ---
    Serial.print(F("Initializing Sensor 2 (Wire1)... "));
    if (!bmx_wire1.begin()) {
        Serial.println(F("Error: Could not detect Sensor 2"));
         while(1) delay(100); // Halt on error
    } else {
        Serial.print(F("Detected "));
        Serial.print((bmx_wire1.getChipID() == CHIP_ID_BME280) ? "BME280" : "BMP280");
        Serial.println(F(" on Wire1."));
        bmx_wire1.setSampling(); // Use default settings or customize
    }

    Serial.println(F("\nStarting measurements..."));
}

void loop()
{
    Serial.println(F("--- Sensor 1 (Wire) ---"));
    readAndPrintSensorData(bmx_wire0);

    Serial.println(F("--- Sensor 2 (Wire1) ---"));
    readAndPrintSensorData(bmx_wire1);

    Serial.println(F("--------------------------"));

    delay(5000); // Wait 5 seconds between readings
}

// Helper function to read and print data from a sensor object
void readAndPrintSensorData(ErriezBMX280 &sensor)
{
    float temperature = sensor.readTemperature();
    float pressure = sensor.readPressure();
    float altitude = sensor.readAltitude(SEA_LEVEL_PRESSURE_HPA);

    Serial.print(F("  Temperature: ")); Serial.print(temperature); Serial.println(F(" *C"));
    Serial.print(F("  Pressure:    ")); Serial.print(pressure / 100.0F); Serial.println(F(" hPa"));
    Serial.print(F("  Altitude:    ")); Serial.print(altitude); Serial.println(F(" m"));

    // Read humidity only if it's a BME280
    if (sensor.getChipID() == CHIP_ID_BME280) {
        float humidity = sensor.readHumidity();
        Serial.print(F("  Humidity:    ")); Serial.print(humidity); Serial.println(F(" %RH"));
    }
    Serial.println();
}
