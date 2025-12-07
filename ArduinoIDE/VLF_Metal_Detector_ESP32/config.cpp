#include "defs.h"
#include "params.h"
#include "config.h"
#include <Arduino.h>
#include <EEPROM.h>
#include <esp_task_wdt.h>

#include "driver/adc.h"
#include "esp_adc_cal.h"

unsigned long flashTimer;
extern CParameters params;
const float VOLTAGE_DIVIDER_RATIO = 3.5f; // (100k + 250k) / 100k

// Konstanten für die LED-Blink-Logik
const unsigned long LED_BLINK_ON_MS = 50;
const unsigned long LED_BLINK_OFF_MS = 100;
const unsigned long LED_BLINK_CYCLE_MS = 6000;


// --- Globale Variablen für Kalibrierung ---

// BatPin (GPIO 39) ist ADC1 Channel 3
#define ADC_CHANNEL_BAT ADC1_CHANNEL_3 
// Muss mit der Einstellung in setup() (VLF_Metal_Detector_ESP32.ino) übereinstimmen
#define ADC_ATTENUATION ADC_ATTEN_DB_11 
// Muss mit der Einstellung in setup() (VLF_Metal_Detector_ESP32.ino) übereinstimmen
#define ADC_BIT_WIDTH ADC_WIDTH_BIT_12 

static esp_adc_cal_characteristics_t g_adc_chars;
static bool g_adc_calibrated = false;


void LED() {
  if (millis() - flashTimer < LED_BLINK_ON_MS) { 
      digitalWrite(LedPin,HIGH);
      } 
      else if (millis() - flashTimer > LED_BLINK_OFF_MS) { 
      digitalWrite(LedPin,LOW);
      } 
      if (millis() - flashTimer > LED_BLINK_CYCLE_MS) { 
      flashTimer = millis(); 
      }
}

/**
 * @brief Initialisiert die ADC-Kalibrierungs-Charakteristiken.
 * Muss von setup() im Main .ino aufgerufen werden.
 */
void setup_adc_calibration() {
    // Kalibrierungs-Charakteristiken abrufen
    esp_adc_cal_value_t cal_type = esp_adc_cal_characterize(ADC_UNIT_1, 
                                                            ADC_ATTENUATION, 
                                                            ADC_BIT_WIDTH, 
                                                            1100, // Default Vref in mV
                                                            &g_adc_chars);
    
    Serial.print("DEBUG: ADC-Kalibrierungstyp: ");
    if (cal_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        Serial.println("eFuse Vref");
    } else if (cal_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        Serial.println("eFuse Two-Point");
    } else {
        Serial.println("Default Vref (1100mV)");
    }
    
    g_adc_calibrated = true;
}


/**
 * @brief Liest die Batteriespannung
 * Verwendet die kalibrierten ADC-Werte anstelle der linearen Annahme.
 */
int getBatteryVoltage_mV() {
    if (!g_adc_calibrated) {
        Serial.println("FEHLER: ADC-Kalibrierung nicht initialisiert!");
        return 0; 
    }

    // 1. Rohwert lesen
    int raw_adc = analogRead(BatPin); 

    // 2. Spannung berechnen (mit Kalibrierung)
    uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(raw_adc, &g_adc_chars);

    // 3. Spannungsteiler zurückrechnen
    float vbat_mv = (float)voltage_mv * VOLTAGE_DIVIDER_RATIO;

    // Rückgabe als Integer
    return (int)(vbat_mv); 
}