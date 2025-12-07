/**
 * @file VLF_Metal_Detector_ESP32.ino
 * @author Anhaltiner01
 * @brief Firmware für einen VLF/IB Metalldetektor basierend auf dem ESP32.
 * @version 2.0 (RTOS, Cleaned, Full Debug & Binary TX, AD7685 SPI Support)
 * @date 2025-09-27
 *
 * @copyright Copyright (c) 2025
 *
 * @details
 * Dieses Projekt implementiert einen VLF (Very Low Frequency) Metalldetektor
 * mit Hardware-Demodulation und BluetoothLE Datenübertragung und Parametrierung per App.
 * Dieses Konzept ist sehr energiesparend statt Verwendung eines AP mit Webserver.
 *
 * Die Firmware ist für den Betrieb mit einer DD-Spule ausgelegt.
 * Die Kernfunktionalität ist in zwei FreeRTOS-Tasks aufgeteilt, um eine stabile
 * Echtzeit-Signalverarbeitung zu gewährleisten:
 * 1.  **ADC-Task:** * Eine hochpriore Task, die kontinuierlich die analogen Signale der
 * Empfangsspule (X- und R-Komponenten) liest und digital filtert.
 * 2.  **Processing-Task:** * Eine niederpriore Task, die die gefilterten Daten empfängt,
 * daraus Phase und Stärke berechnet und die Ergebnisse über Bluetooth Low Energy (BLE)
 * an eine App sendet.
 *
 * Die Konfiguration und Kalibrierung des Detektors erfolgt ausschließlich über BLE.
 * Die phasenverschobenen Steuersignale für den externen Demodulator werden präzise
 * und non-blocking durch die MCPWM-Peripherie des ESP32 erzeugt.
 *
 * Diese Version implementiert umfassende Debug-Ausgaben über UART (115200 Baud) für alle
 * Aktionen und Fehler sowie die Datenübertragung an die App über ein binäres Protokoll.
 *
 * Diese Version unterstützt sowohl die internen ADC als auch
 * externe, hochauflösende AD7685 ADCs über einen Kompilierungsschalter.
 */

//====================================================================================
// Includes
//====================================================================================
#include <EEPROM.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <esp_task_wdt.h>
#include <SPI.h>
#include <cmath>

#include "esp_timer.h"
#include "driver/mcpwm_prelude.h"
#include "driver/dac_oneshot.h"
#include "soc/gpio_sig_map.h"

#include "defs.h"
#include "params.h"
#include "config.h"

//====================================================================================
// Befehls- & Protokoll-Definitionen
//====================================================================================
#ifndef USE_LEGACY_STRING_PROTOCOL
  #define CMD_HEADER             0xAB
  #define DATA_HEADER_MAIN       0xAC
  #define DATA_HEADER_SETTINGS   0xAD // Eigener Header für Einstellungs-Pakete
  #define CMD_SIGNAL_INC         0x10
  #define CMD_SIGNAL_DEC         0x11
  #define CMD_REFRESH_INC        0x20
  #define CMD_REFRESH_DEC        0x21
  #define CMD_PHASE_INC          0x30
  #define CMD_PHASE_DEC          0x31
  #define CMD_GROUND_X_INC       0x40
  #define CMD_GROUND_X_DEC       0x41
  #define CMD_GROUND_R_INC       0x42
  #define CMD_GROUND_R_DEC       0x43
  #define CMD_SET_CALIBRATE      0xA0
  #define CMD_SET_MAX_SIGNAL     0xA1
  #define CMD_SET_NOTCH          0xB0
  #define CMD_SAVE_TO_EEPROM     0xC0
  #define CMD_LOAD_FROM_EEPROM   0xC1
  #define CMD_LOAD_DEFAULTS      0xC2
  // Eigene Befehle zum Senden der aktuellen Einstellungen an die App
  #define CMD_PUSH_SENSITIVITY   0xD0
  #define CMD_PUSH_SAMPLERATE    0xD1
  #define CMD_PUSH_PHASESHIFT    0xD2
  #define CMD_PUSH_GROUND_X      0xD3
  #define CMD_PUSH_GROUND_R      0xD4
  #define CMD_PUSH_NOTCH         0xD5
  #define CMD_LED_LIGHTING_ON    0xE0
  #define CMD_LED_LIGHTING_OFF   0xE1
#endif

//====================================================================================
// Globale Objekte und Variablen
//====================================================================================
// --- Task & System-Parameter ---
const int ADC_QUEUE_LENGTH = 10;       //!< Maximale Anzahl an ADC-Messwerten, die zwischen Tasks gepuffert werden.
const int ADC_TASK_DELAY_MS = 5;       //!< Pause (in ms) der ADC-Task, bestimmt die Abtastrate (ca. 200 Hz).
const int EEPROM_SIZE = 512;           //!< Allokierte Speichergröße (in Bytes) für das EEPROM.
const int DAC_VREF_OUTPUT_LEVEL = 127; //!< DAC-Wert (0-255) für die 1.65V Vref (Mittelspannung).

// --- Protokoll- & App-Logik ---
const int VDI_VALUE_CLEAR = -99;       //!< Spezieller VDI-Wert, der "kein Ziel" signalisiert (für App-Anzeige).
const int MSG_CODE_CLEAR = 200;        //!< Nachrichten-Code an die App, um die Anzeige zurückzusetzen.
const int MSG_CODE_OVERLOAD = 201;     //!< Nachrichten-Code an die App, um "Overload" (Übersteuerung) zu melden.

// --- Signalverarbeitung ---
const float ADC_EMA_ALPHA = 0.2f;      //!< Glättungsfaktor (Alpha) für den EMA-Filter der ADC-Rohdaten.
const int PHASE_MIN_DEGREES = -90;     //!< Untere Grenze (in Grad) für die Phasenberechnung.
const int PHASE_MAX_DEGREES = 90;      //!< Obere Grenze (in Grad) für die Phasenberechnung.
const int MIN_READING_SAMPLES = 20;    //!< Minimale Puffergröße (Samples) für den Peak-Finder.
const int SAFE_MAX_SIGNAL_MIN = 100;   //!< Sicherer Mindestwert für m_maxSignal beim Laden aus EEPROM.
const int SAFE_MAX_SIGNAL_DEFAULT = 4000; //!< Veralteter Standardwert für m_maxSignal (wird aktuell nicht verwendet).

// --- BLE Sende-Delays ---
const int LEGACY_PACKET_DELAY_MS = 10;   //!< Kurze Pause (in ms) nach Senden eines Legacy-Pakets (String-Protokoll).
const int SETTINGS_PACKET_DELAY_MS = 50; //!< Pause (in ms) beim Senden von Einstellungs-Paketen (Binärprotokoll).

#ifndef ENABLE_INTERNAL_ADC
  SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);
#endif

TaskHandle_t adcTaskHandle = NULL;
TaskHandle_t processingTaskHandle = NULL;
QueueHandle_t adcQueue;
struct AdcData {
  float filtered_x;
  float filtered_r;
};

// Mutex zum Schutz aller geteilten Daten
SemaphoreHandle_t g_sharedDataMutex;
volatile bool g_rebootRequired = false;

#ifdef ENABLE_GROUND_TRACKING
  const unsigned long GROUND_TRACK_DELAY = 2000;
  const float GROUND_TRACK_SPEED = 0.005f;
  unsigned long lastMetalTime = 0;
#endif

const int WATCHDOG_TIMEOUT = 5;

// Definition der System-Zustände
enum DetectorState {
  STATE_IDLE,
  STATE_CALIBRATING_ZERO,
  STATE_CALIBRATING_MAX
};

DetectorState currentState = STATE_IDLE; // Startzustand
const int ZERO_CALIBRATION_SAMPLES = 1000;
const int MAX_CALIBRATION_SAMPLES = 2500;   // Sicherer Wert < 5 Sek.

// Variablen für den Kalibrierungsprozess
int calibrationSampleCount = 0;
uint64_t calibSumX = 0;
uint64_t calibSumR = 0;
uint16_t calibMaxX = 0;
uint16_t calibMaxR = 0;

bool ledStatus = 0;

#define PHYSICAL_BUFFER_SIZE 350  // Maximale physische Puffergröße für Sample-Rate

CParameters params;
// extern CParameters params;
int VDI = 0;
int MSG = 0;
char DataString[64];
BLECharacteristic *pCharacteristic_RX;
BLECharacteristic *pCharacteristic_TX;
bool deviceConnected = false;

// Flag für das Senden von Legacy-Einstellungen bei Verbindung
bool g_sendLegacySettings = false;

// Flag, um Auto-Ground-Tracking nach manuellem Eingriff zu sperren
bool g_manualGroundTuningActive = false;

// Variablen für das Throttling - Maximale Sendefrequenz
const unsigned long SEND_INTERVAL_MS = 100;
unsigned long lastSendTime = 0;

// Variablen, um den zuletzt gesendeten Zustand zu speichern
int lastSentVDI = -100;       // Mit ungültigem Wert initialisieren
int lastSentStrength = -1;    // Mit ungültigem Wert initialisieren
// Rausch-Toleranzschwellen für das Throttling (muss noch optimiert werden)
const int VDI_NOISE_THRESHOLD = 1;      // VDI muss sich um mehr als 1 ändern
const int STRENGTH_NOISE_THRESHOLD = 2; // Stärke muss sich um mehr als 2 ändern

/**
 * @brief Halbe Breite des Mittelungsfensters für den VDI-Wert.
 * @details 1 = 3 Samples (Peak-1, Peak, Peak+1). 
 * 2 = 5 Samples. 
 * Höhere Werte stabilisieren den VDI, machen ihn aber träger.
 */
const int VDI_AVERAGING_REACH = 2; // Mittelung über 5 Sampels

//====================================================================================
// Funktions- Definitionen
//====================================================================================
// Forward declarations
#ifndef USE_LEGACY_STRING_PROTOCOL
  void parseCommandPacket(const uint8_t* data, size_t length);
  void sendDataPacket(int8_t vdi, uint8_t msg, uint8_t strength);
#endif

// Forward declarations für die Legacy-Sende-Funktionen
#ifdef USE_LEGACY_STRING_PROTOCOL
  void sendLegacyPacket(char identifier, int value);
  void sendLegacyBatteryPacket();
#endif

extern TaskHandle_t adcTaskHandle;
void sendSettingsPacket(uint8_t setting_type, int16_t value);
void sendAllSettingsTask(void *pvParameters);
void IRAM_ATTR onAdcTimer(void* arg);

//====================================================================================
// BLE UUIDs und Klassen
//====================================================================================
#define SERVICE_UUID                 "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX       "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX       "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

/**
 * @class MyServerCallbacks
 * @brief Behandelt BLE-Server-Events wie Verbindungsaufbau und -abbau.
 */
class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t *param) {
    deviceConnected = true;
    digitalWrite(LedPin, HIGH);
    Serial.println("DEBUG: Bluetooth Client Connected");

    pServer->updateConnParams(param->connect.remote_bda, 80, 80, 0, 500);
    Serial.println("DEBUG: Requesting shorter connection timeout (5s).");

    #ifdef USE_LEGACY_STRING_PROTOCOL
      Serial.println("DEBUG: Waiting for App to enable Notifications...");
    #else
    // Starte eine Task, um alle Einstellungen zu senden, ohne den Callback zu blockieren.
    xTaskCreate(
        sendAllSettingsTask,    // Funktion der Task
        "SendSettings",         // Name der Task
        2048,                   // Stack-Größe
        NULL,                   // Task-Parameter
        1, 
        NULL                    // Task-Handle
    );
    #endif
  };

  void onDisconnect(BLEServer* pServer) {
  deviceConnected = false;
  xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
  ledStatus = 1;
  xSemaphoreGive(g_sharedDataMutex);

  // Licht aus bei Verbindungsabbruch (Sicherheitsfunktion)
    digitalWrite(LED_lighting, LOW); 

  Serial.println("DEBUG: Bluetooth Client Disconnected");
  BLEDevice::startAdvertising();
  Serial.println("DEBUG: Advertising restarted.");
  }
};

/**
 * @class MyCallbacks
 * @brief Behandelt Schreib-Events auf die RX-Charakteristik.
 */
class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    #ifdef USE_LEGACY_STRING_PROTOCOL
      // --- Code-Block für das alte String-Protokoll (Optimiert gegen Heap-Fragmentierung) ---
      // 1. std::string (vom BLE-Stack) holen, keine Arduino-String-Kopie
      std::string rxValueStd = pCharacteristic->getValue().c_str();

      // 2. C-String-Pointer und Länge holen
      const char* rxData = rxValueStd.c_str();
      size_t rxLen = rxValueStd.length();

      // 3. Debug-Ausgabe (verwende den C-String)
      Serial.printf("DEBUG RX (Legacy): Received string: %s (len %u)\n", rxData, rxLen);

      // --- Lokale Variablen/Flags für Aktionen außerhalb des Mutex ---
      int log_minSignal = -1;
      int log_readingSamples = -1;
      int log_XBias = -1;
      int log_RBias = -1;
      int log_phase = -1;
      bool do_save = false;
      bool do_calib_zero = false;
      bool do_calib_max = false;
      bool do_restart = false;
      bool manual_tuning_activated = false;
      
      // 4. Befehle parsen *bevor* der Mutex genommen wird.
      // Wir suchen nur nach einzelnen Zeichen.
      bool A_pressed = false;
      bool B_pressed = false;
      bool C_pressed = false;
      bool D_pressed = false;
      bool E_pressed = false;
      bool F_pressed = false;
      bool G_pressed = false;
      bool H_pressed = false;
      bool I_pressed = false;
      bool J_pressed = false;
      bool K_pressed = false;
      bool L_pressed = false;
      bool M_pressed = false; // Licht AN
      bool N_pressed = false; // Licht AUS
      bool S_pressed = false; // Für Speichern in EEPROM
      bool O_pressed = false; // Für Load (Open) aus EEPROM
      bool R_pressed = false; // Für Reset (Defaults) aus EEPROM

      // 5. Einzel-Scan-Schleife statt vieler 'indexOf'
      // Das ist viel schneller und allokiert keinen Speicher.
      for (size_t i = 0; i < rxLen; i++) {
          switch (rxData[i]) {
              case 'A': A_pressed = true; break;
              case 'B': B_pressed = true; break;
              case 'C': C_pressed = true; break;
              case 'D': D_pressed = true; break;
              case 'E': E_pressed = true; break;
              case 'F': F_pressed = true; break;
              case 'G': G_pressed = true; break;
              case 'H': H_pressed = true; break;
              case 'I': I_pressed = true; break;
              case 'J': J_pressed = true; break;
              case 'K': K_pressed = true; break;
              case 'L': L_pressed = true; break;
              case 'M': M_pressed = true; break;  // Cases für Licht
              case 'N': N_pressed = true; break;  // Cases für Licht
              case 'S': 
                  S_pressed = true;
                  Serial.println("DEBUG: Empfange 'S' (Speichern EEPROM) im Parser!");
                  break;
              case 'O': 
                  O_pressed = true; 
                  Serial.println("DEBUG: Empfange 'O' (Load EEPROM) im Parser!"); 
                  break;
              case 'R': 
                  R_pressed = true; 
                  Serial.println("DEBUG: Empfange 'R' (Defaults EEPROM) im Parser!"); 
                  break;
          }
      }

      // --- Mutex HIER NEHMEN (Jetzt nur noch für schnelle Zuweisungen) ---
      xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);

      // Aktuellen Status lesen, um Befehle ggf. abzuweisen
      DetectorState local_currentState = currentState;
      
      // 1. Sensitivität (A, B)
      if (B_pressed) { 
        if (params.m_minSignal < 255) { 
          params.m_minSignal++; 
          log_minSignal = params.m_minSignal;
        } 
      }
      if (A_pressed) { 
        if (params.m_minSignal > 0) { 
          params.m_minSignal--;
          log_minSignal = params.m_minSignal; 
        } 
      }

      // 2. Reading Samples (C, D)
      if (D_pressed) { 
        if (params.m_readingSamples < PHYSICAL_BUFFER_SIZE) { 
          params.m_readingSamples++;
          log_readingSamples = params.m_readingSamples; 
        } 
      }
      if (C_pressed) { 
        if (params.m_readingSamples > 20) { 
          params.m_readingSamples--;
          log_readingSamples = params.m_readingSamples; 
        } 
      }
      
      // 3. Phase (E, F) -> Löst Neustart aus
      if (F_pressed) { 
        if (params.m_Phase_Quadrature < 180) {
          params.m_Phase_Quadrature++;
          log_phase = params.m_Phase_Quadrature; 
          do_restart = true; 
        }
      }
      if (E_pressed) { 
        if (params.m_Phase_Quadrature > 0) {
          params.m_Phase_Quadrature--;
          log_phase = params.m_Phase_Quadrature; 
          do_restart = true; 
        }
      }
      
      // 4. Kalibrierung (K, L) -> Ändert Zustand
      if (K_pressed) { 
        currentState = STATE_CALIBRATING_ZERO;
        g_manualGroundTuningActive = false; // Auto-Tracking wieder erlauben
        do_calib_zero = true;
      }
      if (L_pressed) { 
        currentState = STATE_CALIBRATING_MAX;
        do_calib_max = true;
      }

      // LED Beleuchtung
      if (M_pressed) {
          digitalWrite(LED_lighting, HIGH);
          Serial.println("DEBUG: LED_lighting ON");
      }
      if (N_pressed) {
          digitalWrite(LED_lighting, LOW);
          Serial.println("DEBUG: LED_lighting OFF");
      }

      // 5. Manuelles Ground-Tuning (G, H, I, J)
      // NEU: Prüfe, ob das Gerät im Leerlauf ist
      if (local_currentState == STATE_IDLE) 
      {
        if (G_pressed) { params.m_XBias++; log_XBias = params.m_XBias; }
        if (H_pressed) { params.m_XBias--; log_XBias = params.m_XBias; }
        if (I_pressed) { params.m_RBias++; log_RBias = params.m_RBias; }
        if (J_pressed) { params.m_RBias--; log_RBias = params.m_RBias; }
      }

      if (G_pressed || H_pressed || I_pressed || J_pressed) {
        // Setze das 'manual tuning' Flag nur, wenn der Befehl
        // auch im richtigen Status (IDLE) ausgeführt wurde.
        if (local_currentState == STATE_IDLE && !g_manualGroundTuningActive) {
          g_manualGroundTuningActive = true;
          manual_tuning_activated = true; // Flag für Log-Meldung
        }
      }

      // 6. Speichern (S) oder Laden (O) oder Reset (R) ---
      if (S_pressed || O_pressed || R_pressed) {
        Serial.println("DEBUG: Starte Speichern/Load/Reset Logik EEPROM...");
        
        if (S_pressed) {
          do_save = true;
          Serial.println("DEBUG: Parameters stored in EEPROM via App.");
        }
        if (O_pressed) {
          params.loadParams(); // Lädt Werte aus EEPROM in RAM
          Serial.println("DEBUG: Parameters loaded from EEPROM via App.");
        }
        if (R_pressed) {
          params.setDefaults(); // Setzt RAM und EEPROM auf Standard
          Serial.println("DEBUG: Defaults loaded via App.");
        }

        // TRICK: Wir füllen die Log-Variablen mit den neuen Werten.
        // Dadurch sendet der Code unten (nach dem Mutex) automatisch 
        // die neuen Werte an die App zurück!
        log_minSignal = params.m_minSignal;
        log_readingSamples = params.m_readingSamples;
        log_phase = params.m_Phase_Quadrature;
        log_XBias = params.m_XBias;
        log_RBias = params.m_RBias;
        
        // Phase hat sich geändert -> Neustart nötig
        if (O_pressed) do_restart = true; // Bei Load sicherheitshalber Neustart (Phase könnte anders sein)
        if (R_pressed) do_restart = true; // Bei Defaults sicherheitshalber Neustart
      }

      // --- Mutex HIER FREIGEBEN ---
      xSemaphoreGive(g_sharedDataMutex);
      
      // --- Aktionen NACH dem Mutex (langsame I/O) ---
      // (Dieser Teil bleibt unverändert)

      if (log_minSignal != -1) { 
        Serial.printf("DEBUG: Sensitivity set to %d\n", log_minSignal);
        sendLegacyPacket('g', log_minSignal);
      }
      if (log_readingSamples != -1) { 
        Serial.printf("DEBUG: ReadingSamples set to %d\n", log_readingSamples);
        sendLegacyPacket('r', log_readingSamples);
      }
      if (log_XBias != -1) { 
        Serial.printf("DEBUG: m_XBias manually set to %d\n", log_XBias);
        sendLegacyPacket('x', log_XBias);
      }
      if (log_RBias != -1) { 
        Serial.printf("DEBUG: m_RBias manually set to %d\n", log_RBias);
        sendLegacyPacket('y', log_RBias);
      }
      if (manual_tuning_activated) { 
        Serial.println("DEBUG: Manual ground tuning activated. Auto-tracking permanently disabled.");
      }

      // Kalibrierungs-Globals setzen (nachdem der State-Wechsel sicher ist)
      if (do_calib_zero) { 
        Serial.println("DEBUG: Ground Balance Calibration initiated.");
        digitalWrite(LedPin, LOW); 
        calibSumX = 0;
        calibSumR = 0;
        calibrationSampleCount = 0;
        Serial.println("DEBUG: Auto-Tracking re-enabled.");
      }
      if (do_calib_max) { 
        Serial.println("DEBUG: Max Signal Calibration initiated.");
        digitalWrite(LedPin, LOW); 
        calibMaxX = -1;
        calibMaxR = -1;
        calibrationSampleCount = 0;
      }

      // Speichern (erfordert erneutes Nehmen des Mutex, da saveParams() liest)
      if (do_save) {
        Serial.println("DEBUG: Saving all params to EEPROM (Legacy command)...");
        xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
        params.saveParams();
        xSemaphoreGive(g_sharedDataMutex);
      }

      // Neustart (als letztes, mit sicherem Flag)
      if (do_restart) {
        Serial.printf("DEBUG: QuadraturePhase set to %d, flagging restart...\n", log_phase);
        g_rebootRequired = true; // Globales Flag für sauberen Neustart in der main_logic_task
      }

    #else
      // --- Code-Block für das Binär-Protokoll (UNVERÄNDERT) ---
      String value = pCharacteristic->getValue();
      if (value.length() > 0) {
        // Die Mutex-Logik ist in parseCommandPacket() implementiert
        parseCommandPacket((const uint8_t*)value.c_str(), value.length());
      }
    #endif
  }
};

/**
 * @class My2902Callbacks
 * @brief Behandelt das Aktivieren/Deaktivieren von Notifications durch die App.
 * @details Wird aufgerufen, wenn die App sich für Benachrichtigungen registriert.
 */
class My2902Callbacks : public BLEDescriptorCallbacks {
    void onWrite(BLEDescriptor *pDescriptor) {
        uint8_t* pValue = pDescriptor->getValue();
        if (pDescriptor->getLength() >= 2) {
            // 0x01 = Notify enabled, 0x00 = Notify disabled
            if (pValue[0] == 0x01 && pValue[1] == 0x00) {
                Serial.println("DEBUG: App has enabled Notifications.");
                
                // JETZT ist die App bereit, die Einstellungen zu empfangen.
                #ifdef USE_LEGACY_STRING_PROTOCOL
                  g_sendLegacySettings = true; 
                #endif
            } else if (pValue[0] == 0x00 && pValue[1] == 0x00) {
                Serial.println("DEBUG: App has disabled Notifications.");
            }
        }
    }
};

//====================================================================================
// Gedrosselte Sende-Funktion (Throttling)
//====================================================================================
#ifdef USE_LEGACY_STRING_PROTOCOL
/**
 * @brief Sendet ein einzelnes Einstellungs-Paket im Legacy-Format.
 * @param identifier Der Kennbuchstabe (z.B. 'g', 'r', 'x').
 * @param value Der zu sendende Wert.
 */
void sendLegacyPacket(char identifier, int value) {
  if (!deviceConnected) return;

  char blePacket[20];
  snprintf(blePacket, sizeof(blePacket), "%c,%d", identifier, value);
  pCharacteristic_TX->setValue(blePacket);
  pCharacteristic_TX->notify();
  // Eine sehr kurze Verzögerung, um den BLE-Stack nicht zu überfluten
  vTaskDelay(pdMS_TO_TICKS(LEGACY_PACKET_DELAY_MS)); 
}

/**
 * @brief Sendet das Batterie-Paket (Sonderformat).
 */
void sendLegacyBatteryPacket() {
  if (!deviceConnected) return;

  // Wert Thread-sicher lesen
  xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
  int local_Battery = params.m_Battery;
  xSemaphoreGive(g_sharedDataMutex);

  char blePacket[20];
  
  /// 1. Wandle Millivolt (mV) in "Hundertstel Volt" um.
  float hundredths_of_volt = (float)local_Battery / 10.0f;

  // 2. Runde den Gleitkommawert mathematisch korrekt.
  int rounded_hundredths = (int)roundf(hundredths_of_volt);

  // 3. Teile den gerundeten Wert in Ganzzahl und Dezimalstellen auf.
  int batt_whole = rounded_hundredths / 100;
  int batt_decimals_100th = rounded_hundredths % 100;

  // Das Format "%02d" stellt sicher, dass "5" als "05" gesendet wird
  snprintf(blePacket, sizeof(blePacket), "b,%d.%02d", batt_whole, batt_decimals_100th);

  pCharacteristic_TX->setValue(blePacket);
  pCharacteristic_TX->notify();
  vTaskDelay(pdMS_TO_TICKS(LEGACY_PACKET_DELAY_MS));
}
#endif

/**
 * @brief Sendet Daten gedrosselt UND nur bei Zustandsänderung über BLE, aber mit Heartbeat
 * @details Prüft zuerst, ob sich VDI oder Stärke geändert haben. Sendet alle Daten als separate,
 * mit <kennung,wert> eingerahmte Pakete, wenn das Throttling-Gate öffnet.
 * Diese Version ist für das Parsen in AI2 (vermeidet \n -> wird für Listen genutzt) optimiert.
 */
void sendThrottledBleData(int vdi, int msg, int strength) {
  unsigned long now = millis();
  // Heartbeat: Erzwinge Senden alle 500ms, auch wenn sich nichts ändert.
  // Das hält den Graphen in der Web-App am Laufen.
  bool forceSend = (now - lastSendTime > 500);

  // 1. Prüfung: Nur fortfahren, wenn sich die Daten geändert haben ODER Heartbeat fällig ist
  if (!forceSend && msg == 0 &&
      abs(vdi - lastSentVDI) < VDI_NOISE_THRESHOLD &&
      abs(strength - lastSentStrength) < STRENGTH_NOISE_THRESHOLD)
  {
    return; // Keine Änderung & kein Heartbeat -> Abbruch
  }

  // 2. Prüfung (Zeitdrossel für normale Änderungen)
  // Verhindert Überflutung, wenn sich Werte sehr schnell ändern
  if (!forceSend && msg == 0 && (now - lastSendTime < SEND_INTERVAL_MS)) {
    return; 
  }
  
  lastSendTime = now;
  lastSentVDI = vdi;
  lastSentStrength = strength;

  #ifdef USE_LEGACY_STRING_PROTOCOL
    char blePacket[20];
    
    // Status-Nachrichten (z.B. Clear/Overload)
    if (msg != 0) {
        snprintf(blePacket, sizeof(blePacket), "m,%d", msg);
        pCharacteristic_TX->setValue(blePacket);
        pCharacteristic_TX->notify();
        vTaskDelay(pdMS_TO_TICKS(LEGACY_PACKET_DELAY_MS));
    }

    // VDI-Wert
    snprintf(blePacket, sizeof(blePacket), "v,%d", vdi);
    pCharacteristic_TX->setValue(blePacket);
    pCharacteristic_TX->notify();
    vTaskDelay(pdMS_TO_TICKS(LEGACY_PACKET_DELAY_MS));

    // Signalstärke
    snprintf(blePacket, sizeof(blePacket), "s,%d", strength);
    pCharacteristic_TX->setValue(blePacket);
    pCharacteristic_TX->notify();
    vTaskDelay(pdMS_TO_TICKS(LEGACY_PACKET_DELAY_MS));

  #else
    sendDataPacket(vdi, msg, strength);
  #endif
}

void setup_precise_adc_timer() {
    const esp_timer_create_args_t timer_args = {
        .callback = &onAdcTimer,
        .name = "adc_precision_timer"
    };

    esp_timer_handle_t adc_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &adc_timer));

    // Startet den Timer periodisch
    // 5 ms = 5000 Mikrosekunden
    // Wir nutzen hier fest 5000us entsprechend Ihrer ADC_TASK_DELAY_MS Konstante
    ESP_ERROR_CHECK(esp_timer_start_periodic(adc_timer, 5000)); 
    
    Serial.println("DEBUG: High-resolution ADC timer started (200 Hz).");
}

//====================================================================================
// FreeRTOS Tasks
//====================================================================================
/**
 * @brief Hochpriore Task für ADC-Abtastung mit Oversampling und digitaler Filterung.
 * @details Verwendet einen Hardware-Timer für jitterfreies Timing (ulTaskNotifyTake).
 * Führt ein Oversampling (Mittelwertbildung) durch, um das Signalrauschen zu minimieren,
 * bevor der EMA-Filter angewendet wird.
 * * @param pvParameters Nicht verwendete Task-Parameter.
 */
void adc_sampling_task(void *pvParameters) {
  Serial.println("DEBUG: ADC Sampling Task started (Oversampling Enabled)");
  
  // Watchdog registrieren
  if(esp_task_wdt_add(NULL) != ESP_OK){ 
    Serial.println("ERROR: Failed to add ADC task to WDT");
  }

  // --- Initialisierung ---
  #ifndef ENABLE_INTERNAL_ADC
    // Konfiguration für externe SPI ADCs (AD7685)
    pinMode(ADC1_CS_PIN, OUTPUT);
    pinMode(ADC2_CS_PIN, OUTPUT);
    digitalWrite(ADC1_CS_PIN, HIGH); // Chip Select inaktiv (High)
    digitalWrite(ADC2_CS_PIN, HIGH); // Chip Select inaktiv (High)
    
    // SPI Bus initialisieren
    SPI.begin(SPI_SCK_PIN, SPI_MISO_PIN, -1, -1);
    Serial.println("DEBUG: External ADC (AD7685) mode selected.");
  #else
    // Konfiguration für interne ADCs
    pinMode(Rx_XPin, INPUT);
    pinMode(Rx_RPin, INPUT);
    Serial.println("DEBUG: Internal ADC mode selected.");
  #endif

  // Anzahl der Messungen pro Durchlauf (Boxcar-Filter)
  // 16 Messungen erhöhen die theoretische Auflösung um 2 Bit (Faktor 4 im SNR)
  const int OVERSAMPLING_COUNT = 16;

  AdcData dataToSend;
  const float alpha = ADC_EMA_ALPHA;
  
  // Initialisiere Filter mit dem Mittelwert (Bias) passend zur Bit-Breite
  float ema_x = (float)ADC_DEFAULT_BIAS;
  float ema_r = (float)ADC_DEFAULT_BIAS;

  // --- Hauptschleife ---
  for (;;) {
    // 1. WARTEN auf den präzisen Timer-Impuls (alle 5ms)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 2. Watchdog Reset
    esp_task_wdt_reset();

    // 3. Oversampling-Messung durchführen
    // Wir nutzen 32-Bit Integer, um Überläufe beim Summieren zu vermeiden
    // (bei externem ADC: 65535 * 16 = ~1 Mio, passt locker in uint32_t)
    uint32_t sum_x = 0;
    uint32_t sum_r = 0;

    #ifdef ENABLE_INTERNAL_ADC
      // --- Interne ADCs lesen ---
      for (int i = 0; i < OVERSAMPLING_COUNT; i++) {
        sum_x += analogRead(Rx_XPin);
        sum_r += analogRead(Rx_RPin);
      }
    #else
      // --- Externe ADCs (SPI) lesen ---
      // Wir sperren den Bus für die gesamte Dauer der 16 Messungen
      SPI.beginTransaction(spiSettings);
      
      for (int i = 0; i < OVERSAMPLING_COUNT; i++) {
        // Kanal X lesen
        // CS Low startet beim AD7685 meist die Konversion oder framed den Transfer
        digitalWrite(ADC1_CS_PIN, LOW);
        sum_x += SPI.transfer16(0x0000);
        digitalWrite(ADC1_CS_PIN, HIGH);
        
        // Kanal R lesen
        digitalWrite(ADC2_CS_PIN, LOW);
        sum_r += SPI.transfer16(0x0000);
        digitalWrite(ADC2_CS_PIN, HIGH);
      }
      
      SPI.endTransaction();
    #endif

    // Mittelwert berechnen (Integer-Division)
    // Hier erhalten wir einen rauschärmeren "Rohwert"
    uint16_t raw_x = (uint16_t)(sum_x / OVERSAMPLING_COUNT);
    uint16_t raw_r = (uint16_t)(sum_r / OVERSAMPLING_COUNT);

    // 4. Filterung (Exponential Moving Average)
    // Der EMA glättet nun den bereits vor-gemittelten Wert
    dataToSend.filtered_x = (alpha * raw_x) + ((1 - alpha) * ema_x);
    dataToSend.filtered_r = (alpha * raw_r) + ((1 - alpha) * ema_r);
    
    // Filterzustand aktualisieren
    ema_x = dataToSend.filtered_x;
    ema_r = dataToSend.filtered_r;

    // 5. Daten senden
    xQueueSend(adcQueue, &dataToSend, 0);
  }
}

/**
 * @brief Niederpriore Task für die Haupt-Signalverarbeitung und Logik.
 * @param pvParameters Nicht verwendete Task-Parameter.
 *
 * @details Diese Task wartet blockierend, bis neue Daten von der `adc_sampling_task`
 * in der `adcQueue` verfügbar sind.
 * Nach dem Empfang führt sie die komplette Signalverarbeitungs-Pipeline durch:
 * 1.  (Optional) Automatischer Bodenabgleich (Ground Tracking).
 * 2.  Anwendung des Nullabgleichs (Bias).
 * 3.  Berechnung von Phasenwinkel und Signalstärke (JETZT MIT HOHER PRÄZISION).
 * 4.  Pflege eines Ringpuffers zur Erkennung von Signalspitzen (Peaks).
 * 5.  Wenn ein valider Peak erkannt wird, wird der VDI-Wert berechnet,
 * optional durch den Notch-Filter geprüft und das Ergebnis über BLE gesendet.
 * 6.  Behandelt außerdem Kalibrierungsanfragen und Status-Updates.
 */
void main_logic_task(void *pvParameters) {
  Serial.println("DEBUG: Main Logic Task started (High Precision Mode)");
  
  if(esp_task_wdt_add(NULL) != ESP_OK){ 
    Serial.println("ERROR: Failed to add Main task to WDT");
  }

  unsigned long lastBatteryCheck = 0; // Zeitstempel für Batterie

  AdcData receivedData;
  
  // OPTIMIERUNG: Arrays als 'static' definiert, um den Task-Stack nicht zu sprengen.
  // OPTIMIERUNG: 'float' für Winkel statt char -> 0.1 Grad Auflösung statt 1 Grad.
  // OPTIMIERUNG: 'uint16_t' für Stärke statt char -> Kein Abschneiden bei > 255.
  static uint16_t strenth[PHYSICAL_BUFFER_SIZE] = {0};
  static float angle[PHYSICAL_BUFFER_SIZE] = {0.0f};
  
  int index = 0;
  bool enoughSamples = false;
  
  for (;;) {
    if (xQueueReceive(adcQueue, &receivedData, portMAX_DELAY) == pdPASS) {
      esp_task_wdt_reset();
      //vTaskDelay(pdMS_TO_TICKS(1));  // Kurze Pause, um den Watchdog nicht auszulösen

      // --- ZUSTANDS-MASCHINE ---
      
      // Hole den aktuellen Zustand Thread-sicher
      DetectorState local_currentState;
      xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
      local_currentState = currentState;
      xSemaphoreGive(g_sharedDataMutex);
      
      switch (local_currentState) { // Verwende die lokale Kopie
        
        case STATE_CALIBRATING_ZERO:
          // LED bei jedem Schleifendurchlauf umschalten für Blink-Effekt
          digitalWrite(LedPin, !digitalRead(LedPin));
          calibSumX += receivedData.filtered_x;
          calibSumR += receivedData.filtered_r;
          calibrationSampleCount++;

          if (calibrationSampleCount >= ZERO_CALIBRATION_SAMPLES) {
            
            // 1. Berechnete Werte lokal speichern
            uint16_t new_XBias = calibSumX / ZERO_CALIBRATION_SAMPLES;
            uint16_t new_RBias = calibSumR / ZERO_CALIBRATION_SAMPLES;

            // 2. Ergebnisse Thread-sicher in die globalen Parameter schreiben
            xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
            params.m_XBias = new_XBias;
            params.m_RBias = new_RBias;    
            currentState = STATE_IDLE; // Zustand Thread-sicher zurücksetzen
            xSemaphoreGive(g_sharedDataMutex);

            // 3. EEPROM-Schreiben (langsam) außerhalb des Mutex
            EEPROMWriteUInt16(4, new_XBias);
            EEPROMWriteUInt16(6, new_RBias);
            EEPROM.commit();
            
            Serial.printf("DEBUG: m_XBias set to %d\n", new_XBias);
            Serial.printf("DEBUG: m_RBias set to %d\n", new_RBias);
            Serial.println("DEBUG: Ground Balance Calibration Done.");

            // Sende die aktualisierten Werte an die App
            #ifdef USE_LEGACY_STRING_PROTOCOL
              sendLegacyPacket('x', new_XBias); // Lokale Werte verwenden
              sendLegacyPacket('y', new_RBias); // Lokale Werte verwenden
            #else
              sendSettingsPacket(CMD_PUSH_GROUND_X, new_XBias);
              sendSettingsPacket(CMD_PUSH_GROUND_R, new_RBias);
            #endif

            digitalWrite(LedPin, deviceConnected ? HIGH : LOW);
          }
          break;

        case STATE_CALIBRATING_MAX:
          // LED bei jedem Schleifendurchlauf umschalten für Blink-Effekt
          digitalWrite(LedPin, !digitalRead(LedPin));
          if((uint16_t)receivedData.filtered_x > calibMaxX) calibMaxX = (uint16_t)receivedData.filtered_x;
          if((uint16_t)receivedData.filtered_r > calibMaxR) calibMaxR = (uint16_t)receivedData.filtered_r;
          calibrationSampleCount++;

          if (calibrationSampleCount >= MAX_CALIBRATION_SAMPLES) {
            
            // 1. Berechneten Wert lokal speichern
            uint16_t new_maxSignal = min(calibMaxX, calibMaxR) - 20;

            // 2. Ergebnisse Thread-sicher in die globalen Parameter schreiben
            xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
            params.m_maxSignal = new_maxSignal;
            currentState = STATE_IDLE; // Zustand Thread-sicher zurücksetzen
            xSemaphoreGive(g_sharedDataMutex);

            // 3. EEPROM-Schreiben (langsam) außerhalb des Mutex
            EEPROMWriteUInt16(8, new_maxSignal);
            EEPROM.commit(); 
            
            Serial.printf("DEBUG: maxSignal set to %d\n", new_maxSignal);
            Serial.println("DEBUG: Max Signal Calibration Done.");
            
            // LED-Zustand nach Kalibrierung wiederherstellen
            digitalWrite(LedPin, deviceConnected ? HIGH : LOW);
          }
          break;

        case STATE_IDLE:
        default:
          // --- Komplette Signalverarbeitung im Normalbetrieb ---
          
          // Lokale Variablen für den Parameter-Cache
          int local_XBias, local_RBias, local_minSignal, local_maxSignal;
          int local_readingSamples, local_sigPolarity;
          int local_notchLower, local_notchUpper;
          bool local_manualGroundTuning;
          bool local_ledStatus;

          // Mutex holen, alle Werte kopieren, Mutex sofort freigeben
          xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
          local_XBias = params.m_XBias;
          local_RBias = params.m_RBias;
          local_minSignal = params.m_minSignal;
          local_maxSignal = params.m_maxSignal;
          local_readingSamples = params.m_readingSamples;
          local_sigPolarity = params.m_sigPolarity;
          local_notchLower = params.m_notchLowerBound;
          local_notchUpper = params.m_notchUpperBound;
          local_manualGroundTuning = g_manualGroundTuningActive;
          local_ledStatus = ledStatus;
          xSemaphoreGive(g_sharedDataMutex);
          // --- Ab hier NUR NOCH LOKALE VARIABLEN verwenden! ---

          #ifdef ENABLE_GROUND_TRACKING
            float raw_x_gt = receivedData.filtered_x - local_XBias; // LOKAL
            float raw_r_gt = receivedData.filtered_r - local_RBias; // LOKAL
            // OPTIMIERUNG: hypotf ist schneller und sicherer als sqrt(x*x + y*y)
            float current_strength = hypotf(raw_x_gt, raw_r_gt);

            if (current_strength >= local_minSignal) { // LOKAL
              lastMetalTime = millis();
            }
            
            // Führe Auto-Tracking nur aus, wenn 2s vergangen sind UND es nicht manuell deaktiviert wurde
            if (!local_manualGroundTuning && (millis() - lastMetalTime > GROUND_TRACK_DELAY)) { 
              // Werte lokal berechnen
              int new_XBias = (GROUND_TRACK_SPEED * receivedData.filtered_x) + ((1 - GROUND_TRACK_SPEED) * (float)local_XBias);
              int new_RBias = (GROUND_TRACK_SPEED * receivedData.filtered_r) + ((1 - GROUND_TRACK_SPEED) * (float)local_RBias);
              
              // NEU: Berechneten Wert Thread-sicher zurückschreiben
              xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
              params.m_XBias = new_XBias;
              params.m_RBias = new_RBias;
              xSemaphoreGive(g_sharedDataMutex);
            }
          #endif
          
          if (local_ledStatus == 1) LED(); // LOKAL

          // --- Regelmäßige Batteriemessung ---
          if (millis() - lastBatteryCheck > 5000) { // Alle 5 Sekunden
            lastBatteryCheck = millis();
            int newBatteryValue = getBatteryVoltage_mV(); // Wert holen

            // NEU: Wert Thread-sicher speichern
            xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
            params.m_Battery = newBatteryValue;
            xSemaphoreGive(g_sharedDataMutex);

            #ifdef USE_LEGACY_STRING_PROTOCOL
                sendLegacyBatteryPacket();
            #endif
          }

          // Sende alle Einstellungen bei (Neu-)Verbindung
          #ifdef USE_LEGACY_STRING_PROTOCOL
            if (g_sendLegacySettings && deviceConnected) {
              Serial.println("DEBUG: Sending all legacy settings on connect...");
              struct {
                int minSignal;
                int readingSamples;
                int phaseQuadrature;
                int xBias;
                int rBias;
              } local_legacy_params;

              xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
              local_legacy_params.minSignal = params.m_minSignal;
              local_legacy_params.readingSamples = params.m_readingSamples;
              local_legacy_params.phaseQuadrature = params.m_Phase_Quadrature;
              local_legacy_params.xBias = params.m_XBias;
              local_legacy_params.rBias = params.m_RBias;
              xSemaphoreGive(g_sharedDataMutex);

              sendLegacyPacket('g', local_legacy_params.minSignal);
              sendLegacyPacket('r', local_legacy_params.readingSamples);
              sendLegacyPacket('p', local_legacy_params.phaseQuadrature);
              sendLegacyPacket('x', local_legacy_params.xBias);
              sendLegacyPacket('y', local_legacy_params.rBias);
              sendLegacyBatteryPacket(); 
              
              g_sendLegacySettings = false; // Flag zurücksetzen
              Serial.println("DEBUG: Done sending legacy settings.");
            }
          #endif

          float X = receivedData.filtered_x;
          float R = receivedData.filtered_r;

          // Statische Variable, um den letzten Overload-Zustand zu speichern
          static bool isOverloaded = false;
          
          // "NOT-BREMSE" IN DER OVERLOAD-FALLE (OPTIMIERT)
          if(X >= local_maxSignal || R >= local_maxSignal) { // LOKAL
            if (!isOverloaded) { // Nur senden, wenn der Zustand NEU ist
              isOverloaded = true;
              // Sende über die Drossel-Funktion. msg=201 ignoriert Timer.
              sendThrottledBleData(VDI_VALUE_CLEAR, MSG_CODE_OVERLOAD, 0);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
          } 
          else {
            isOverloaded = false;
          }
          
          X = X - local_XBias; // LOKAL
          R = R - local_RBias; // LOKAL

          if( (local_sigPolarity<0 && R>0) || (local_sigPolarity>0 && R<0) ) { // LOKAL
            X = 0.0F;
            R = 0.0F;
          }

          // OPTIMIERUNG: Berechnung der Phase als float behalten
          float phaseVal = ( (X == 0.0F && R == 0.0F) ? 0.0F : atan2(X, R) * 57.29578F );
          
          if(phaseVal < (float)PHASE_MIN_DEGREES) phaseVal = (float)PHASE_MIN_DEGREES;
          else if(phaseVal > (float)PHASE_MAX_DEGREES) phaseVal = (float)PHASE_MAX_DEGREES;
          
          // Map -90..+90 to 0..180 für den Buffer, aber als float speichern für Präzision
          angle[index] = phaseVal - (float)PHASE_MIN_DEGREES;
          
          // OPTIMIERUNG: hypotf statt sqrt
          strenth[index] = (uint16_t)hypotf(X, R);
          
          index++;
          if(index >= local_readingSamples) { // Logische Puffergröße verwenden (LOKAL)
            enoughSamples = true;
            index = 0; 
          }
          if(!enoughSamples) continue;
          
          uint16_t maxVal = 0; // Jetzt uint16
          int maxIndex = -1;
          
          for(int n(0); n<local_readingSamples; n++) { // Logische Puffergröße verwenden (LOKAL)
            if(strenth[n] > maxVal) { 
              maxVal = strenth[n];
              maxIndex = n; 
            }
          }

          if(maxVal < local_minSignal) { // LOKAL
            // Wenn der letzte gesendete Wert *kein* "Clear"-Paket war
            // (d.h. VDI war nicht -99), sende jetzt ein "Clear".
            if (lastSentVDI != VDI_VALUE_CLEAR) {
              // msg=200 ignoriert den Timer und setzt lastSentVDI = -99
              sendThrottledBleData(VDI_VALUE_CLEAR, MSG_CODE_CLEAR, 0);
            }
            continue;
          }

          int dist = index - maxIndex;
          if(dist < 0) dist = local_readingSamples + dist; // Logische Puffergröße verwenden (LOKAL)
          if(dist != local_readingSamples/2) continue;     // Logische Puffergröße verwenden (LOKAL)

          // DIESER BEREICH PROFITIERT JETZT VON DEN FLOAT/UINT16 ARRAYS
          float angleAvg = 0.0F;
          float strengthAvg = 0.0F;
          int avgSamples = 0;
          int n = maxIndex - VDI_AVERAGING_REACH;
          
          while(avgSamples < (VDI_AVERAGING_REACH*2 + 1)) {
            if(n<0) n += local_readingSamples; // Logische Puffergröße verwenden (LOKAL)
            else if(n>=local_readingSamples) n %= local_readingSamples; // Logische Puffergröße verwenden (LOKAL)
            
            // Map 0..180 back to -90..+90, diesmal mit float Präzision aus dem Array
            angleAvg    += (angle[n] + (float)PHASE_MIN_DEGREES);
            strengthAvg += (float)strenth[n];
            
            avgSamples++;
            n++;
          }

          // Das Endergebnis runden wir erst ganz zum Schluss
          int finalPhase = (int)round(angleAvg / avgSamples);
          int finalStrength = (int)round(strengthAvg / avgSamples);

          bool isNotched = false;
          if (local_notchLower < local_notchUpper) { // LOKAL
            if (finalPhase >= local_notchLower && finalPhase <= local_notchUpper) { // LOKAL
              isNotched = true;
            }
          }
          
          if (!isNotched) {
            // --- Nutze die Throttling-Funktion ---
            sendThrottledBleData(finalPhase, 0, finalStrength);
          }
          
          // --- Neustart-Prüfung ---
          if (g_rebootRequired) {
              Serial.println("INFO: Reboot flag detected. Restarting cleanly...");
              vTaskDelay(pdMS_TO_TICKS(200)); // Zeit für letzte BLE-Pakete
              ESP.restart();
          }
          break;
      }
    }
  }
}

#ifndef USE_LEGACY_STRING_PROTOCOL
  /**
  * @brief Baut das binäre Datenpaket zusammen und sendet es über BLE.
  * @param vdi Der zu sendende VDI-Wert.
  * @param msg Der zu sendende Nachrichten-Code.
  * @param strength Die zu sendende Signalstärke.
  */
  void sendDataPacket(int8_t vdi, uint8_t msg, uint8_t strength) {
    if (!deviceConnected) return;

    // Wert Thread-sicher lesen
    xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
    uint16_t battery_mv = params.m_Battery; // Ist bereits mV * 10, muss ggf. angepasst werden
    xSemaphoreGive(g_sharedDataMutex);

    uint8_t packet[8];
    packet[0] = DATA_HEADER_MAIN;
    packet[1] = vdi;
    packet[2] = msg;
    packet[3] = strength;           // Der 'strength'-Parameter wird nun verwendet
    packet[4] = battery_mv & 0xFF;  // Verwende lokale Kopie
    packet[5] = (battery_mv >> 8) & 0xFF;
    packet[6] = 0x00; // Reserviert

    uint8_t checksum = 0;
    for (int i = 0; i < 7; i++) {
        checksum ^= packet[i];
    }
    packet[7] = checksum;

    pCharacteristic_TX->setValue(packet, sizeof(packet));
  pCharacteristic_TX->notify();
    #ifdef DEBUG_TX_PACKETS
    Serial.printf("DEBUG TX: H=0x%02X, VDI=%d, MSG=%d, STR=%d, BATT=%dmV, CS=0x%02X\n",
                  packet[0], packet[1], packet[2], packet[3], battery_mv, packet[7]);
    #endif
  }

  /**
  * @brief Validiert und verarbeitet ankommende binäre Befehlspakete.
  * @details Optimiert, um den Mutex nur für den reinen Parameterzugriff zu sperren.
  * Alle langsamen Aktionen (Serial-IO, Task-Erstellung) erfolgen außerhalb der Sperre.
  * @param data Zeiger auf die empfangenen Bytes.
  * @param length Die Anzahl der empfangenen Bytes.
  */
  void parseCommandPacket(const uint8_t* data, size_t length) {
    // ... (Paket-Validierungs-Code bleibt unverändert) ...
    if (length < 4) { Serial.println("ERROR RX: Packet too short!"); return; }
    if (data[0] != CMD_HEADER) { Serial.println("ERROR RX: Invalid header!"); return; }
    // ... (Checksummen-Prüfung etc.) ...
    
    uint8_t command = data[1];
    // Annahme: data_len wurde durch die Validierung gesetzt
    // uint8_t data_len = data[2]; 

    // --- Lokale Variablen/Flags für Aktionen außerhalb des Mutex ---
    String log_message = "";
    bool restart_required = false;
    bool send_settings_required = false;
    bool calib_zero_triggered = false;
    bool calib_max_triggered = false;
    bool led_on_triggered = false;  // Flag für Beleuchtung
    bool led_off_triggered = false; // Flag für Beleuchtung
    uint16_t local_val = 0; // Für Log-Ausgaben

    // --- Mutex HIER NEHMEN ---
    xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);

    switch (command) {

      case CMD_SIGNAL_INC:
        if (params.m_minSignal < 255) params.m_minSignal++; // Ggf. an ADC_MAX_VALUE anpassen
        local_val = params.m_minSignal;
        log_message = "DEBUG: Sensitivity set to " + String(local_val);
        break;
      case CMD_SIGNAL_DEC:
        if (params.m_minSignal > 0) params.m_minSignal--;
        local_val = params.m_minSignal;
        log_message = "DEBUG: Sensitivity set to " + String(local_val);
        break;

      case CMD_REFRESH_INC:
        if (params.m_readingSamples < PHYSICAL_BUFFER_SIZE) {
          params.m_readingSamples++;
        }
        local_val = params.m_readingSamples;
        log_message = "DEBUG: ReadingSamples set to " + String(local_val);
        break;
      case CMD_REFRESH_DEC:
        if (params.m_readingSamples > MIN_READING_SAMPLES) {
          params.m_readingSamples--;
        }
        local_val = params.m_readingSamples;
        log_message = "DEBUG: ReadingSamples set to " + String(local_val);
        break;

      case CMD_PHASE_INC:
        if (params.m_Phase_Quadrature < 180) params.m_Phase_Quadrature++;
        local_val = params.m_Phase_Quadrature;
        log_message = "DEBUG: QuadraturePhase set to " + String(local_val) + ", flagging restart...";
        restart_required = true;
        break;

      case CMD_PHASE_DEC:
        if (params.m_Phase_Quadrature > 0) params.m_Phase_Quadrature--;
        local_val = params.m_Phase_Quadrature;
        log_message = "DEBUG: QuadraturePhase set to " + String(local_val) + ", flagging restart...";
        restart_required = true;
        break;

      case CMD_GROUND_X_INC:
        if(currentState == STATE_IDLE) params.m_XBias++;
        local_val = params.m_XBias;
        if (!g_manualGroundTuningActive) { 
          g_manualGroundTuningActive = true;
          log_message = "DEBUG: Manual ground tuning activated. Auto-tracking permanently disabled.\n";
        }
        log_message += "DEBUG: m_XBias manually set to " + String(local_val);
        break;

      case CMD_GROUND_X_DEC:
        if(currentState == STATE_IDLE) params.m_XBias--;
        local_val = params.m_XBias;
         if (!g_manualGroundTuningActive) {
          g_manualGroundTuningActive = true;
          log_message = "DEBUG: Manual ground tuning activated. Auto-tracking permanently disabled.\n";
        }
        log_message += "DEBUG: m_XBias manually set to " + String(local_val);
        break;

      case CMD_GROUND_R_INC:
        if(currentState == STATE_IDLE) params.m_RBias++;
        local_val = params.m_RBias;
        if (!g_manualGroundTuningActive) {
          g_manualGroundTuningActive = true;
          log_message = "DEBUG: Manual ground tuning activated. Auto-tracking permanently disabled.\n";
        }
        log_message += "DEBUG: m_RBias manually set to " + String(local_val);
        break;

      case CMD_GROUND_R_DEC:
        if(currentState == STATE_IDLE) params.m_RBias--;
        local_val = params.m_RBias;
        if (!g_manualGroundTuningActive) {
          g_manualGroundTuningActive = true;
          log_message = "DEBUG: Manual ground tuning activated. Auto-tracking permanently disabled.\n";
        }
        log_message += "DEBUG: m_RBias manually set to " + String(local_val);
        break;

      case CMD_SET_CALIBRATE:
        currentState = STATE_CALIBRATING_ZERO;
        g_manualGroundTuningActive = false; // Auto-Tracking wieder erlauben
        calib_zero_triggered = true; // Flag für Aktionen außerhalb
        log_message = "DEBUG: Ground Balance Calibration initiated.\nDEBUG: Auto-Tracking re-enabled.";
        break;

      case CMD_SET_MAX_SIGNAL:
        currentState = STATE_CALIBRATING_MAX;
        calib_max_triggered = true; // Flag für Aktionen außerhalb
        log_message = "DEBUG: Max Signal Calibration initiated.";
        break;

      // Integration in den Switch-Block
      case CMD_LED_LIGHTING_ON:
        led_on_triggered = true;
        log_message = "DEBUG: LED_lighting ON (Binary CMD)";
        break;

      case CMD_LED_LIGHTING_OFF:
        led_off_triggered = true;
        log_message = "DEBUG: LED_lighting OFF (Binary CMD)";
        break;

      case CMD_SET_NOTCH:
        // Annahme: data_len wurde geprüft
        // if (data_len == 2) { ... }
        params.m_notchLowerBound = data[3];
        params.m_notchUpperBound = data[4];
        log_message = "DEBUG: Notch set to " + String(data[3]) + " - " + String(data[4]);
        break;

      case CMD_SAVE_TO_EEPROM:
        // saveParams() liest alle params-Member, muss innerhalb des Mutex bleiben.
        params.saveParams();
        log_message = "DEBUG: Saving all params to EEPROM...";
        break;

      case CMD_LOAD_FROM_EEPROM:
        // loadParams() schreibt alle params-Member, muss innerhalb des Mutex bleiben.
        params.loadParams();
        send_settings_required = true; // Flag für Task-Erstellung
        log_message = "DEBUG: Loading all params from EEPROM...";
        break;

      case CMD_LOAD_DEFAULTS:
        // setDefaults() schreibt alle params-Member, muss innerhalb des Mutex bleiben.
        params.setDefaults();
        send_settings_required = true; // Flag für Task-Erstellung
        log_message = "DEBUG: Loading default params...";
        break;

      default:
        log_message = "ERROR RX: Unknown command!";
        break;
    }
    
    // --- Mutex HIER FREIGEBEN ---
    xSemaphoreGive(g_sharedDataMutex);

    // --- Aktionen NACH dem Mutex (langsame I/O) ---

    if (led_on_triggered) digitalWrite(LED_lighting, HIGH);
    if (led_off_triggered) digitalWrite(LED_lighting, LOW);

    // 1. Logging
    if (log_message != "") {
      Serial.println(log_message);
    }

    // 2. Kalibrierungs-Globals setzen
    if (calib_zero_triggered) {
      digitalWrite(LedPin, LOW);
      calibSumX = 0;
      calibSumR = 0;
      calibrationSampleCount = 0;
    }
    if (calib_max_triggered) {
      digitalWrite(LedPin, LOW);
      calibMaxX = -1;
      calibMaxR = -1;
      calibrationSampleCount = 0;
    }

    // 3. Neue Tasks starten
    if (send_settings_required) {
      xTaskCreate(sendAllSettingsTask, "SendSettings", 2048, NULL, 1, NULL);
    }

    // 4. Neustart (mit sicherem Flag)
    if (restart_required) {
      g_rebootRequired = true; // Globales Flag für sauberen Neustart in der main_logic_task
    }
  }
#endif

/**
 * @brief Konfiguriert die MCPWM-Peripherie für die Signalerzeugung.
 * @details Verwendet die neue MCPWM-Treiber-API und den UP-DOWN Zählmodus,
 * um drei phasen-synchronisierte PWM-Signale effizient zu erzeugen:
 * 1.  Das Tx-Signal für die Sendespule (0° Phase).
 * 2.  Das In-Phase-Referenzsignal (0° Phase) für den Demodulator.
 * 3.  Das Quadratur-Referenzsignal (90° Phase) für den Demodulator.
 */
 #define MCPWM0_GEN0_OUT_IDX 147  // Manuelle Definition als Workaround, müsste eigentlich in #include "soc/gpio_sig_map.h" enthalten sein

void setupMCPWM() {
  Serial.println("DEBUG: Configuring MCPWM with new API...");
  // 1. Timer-Konfiguration (für alle Signale identisch)
  mcpwm_timer_handle_t timer = NULL;
  mcpwm_timer_config_t timer_config = {};
  timer_config.group_id = 0;
  timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
  timer_config.resolution_hz = 1000000;
  timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP; // Zurück zum UP-Modus
  timer_config.period_ticks = 1000000UL / params.m_CoilFrequency;
  ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

  // --- Operator A für das 0°-Signal (Tx und Phase_R) ---
  mcpwm_oper_handle_t oper_a = NULL;
  mcpwm_operator_config_t operator_a_config = {};
  operator_a_config.group_id = 0;
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_a_config, &oper_a));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_a, timer));
  // Comparator für 180° (halbe Periode)
  mcpwm_cmpr_handle_t comparator_180_deg = NULL;
  mcpwm_comparator_config_t comp_config = {};
  comp_config.flags.update_cmp_on_tez = true;
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper_a, &comp_config, &comparator_180_deg));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_180_deg, timer_config.period_ticks / 2));

  // Generator für Tx_CoilPin
  mcpwm_gen_handle_t gen_tx = NULL;
  mcpwm_generator_config_t gen_a_config = {};
  gen_a_config.gen_gpio_num = Tx_CoilPin;
  ESP_ERROR_CHECK(mcpwm_new_generator(oper_a, &gen_a_config, &gen_tx));

  // Aktionen für 0°-Signal: HIGH bei 0, LOW bei 180°
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_tx, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_tx, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_180_deg, MCPWM_GEN_ACTION_LOW)));


  // --- Operator B für das 90°-Signal (Phase_X) ---
  mcpwm_oper_handle_t oper_b = NULL;
  mcpwm_operator_config_t operator_b_config = {};
  operator_b_config.group_id = 0;
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_b_config, &oper_b));
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_b, timer));

  // Dynamische Phase für X-Kanal
  // Comparatoren für Start- und Stopp-Zeitpunkt des Quadratur-Signals
  mcpwm_cmpr_handle_t comparator_start_tick = NULL;
  mcpwm_cmpr_handle_t comparator_stop_tick = NULL;
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper_b, &comp_config, &comparator_start_tick));
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper_b, &comp_config, &comparator_stop_tick));

  // Berechne die Tick-Werte basierend auf params.m_Phase_Quadrature (0-180 Grad) 
  // Der Winkel wird auf die 360-Grad-Periode des Timers abgebildet.
  uint32_t start_tick = (uint32_t)((params.m_Phase_Quadrature / 360.0f) * timer_config.period_ticks);
  
  // Der Stopp-Tick ist 180 Grad (halbe Periode) nach dem Start-Tick
  uint32_t stop_tick = start_tick + (timer_config.period_ticks / 2);

  // Da params.m_Phase_Quadrature < 180, wird stop_tick < period_ticks sein.
  // Ein "Wrap-Around" (Überlauf) findet nicht statt.

  Serial.printf("DEBUG: Phase_Quadrature set to %d degrees.\n", params.m_Phase_Quadrature);
  Serial.printf("DEBUG: MCPWM Start Tick: %d, Stop Tick: %d\n", start_tick, stop_tick);

  // Setze die berechneten Comparator-Werte
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_start_tick, start_tick));
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator_stop_tick, stop_tick));

  // Generator für Phase_X_Pin
  mcpwm_gen_handle_t gen_phase_x = NULL;
  mcpwm_generator_config_t gen_b_config = {};
  gen_b_config.gen_gpio_num = Phase_X_Pin; // 
  ESP_ERROR_CHECK(mcpwm_new_generator(oper_b, &gen_b_config, &gen_phase_x));

  // Aktionen für das Phasen-Signal:
  // Bei 0 (Timer-Start) -> LOW
  // Bei start_tick        -> HIGH
  // Bei stop_tick         -> LOW
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(gen_phase_x, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_phase_x, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_start_tick, MCPWM_GEN_ACTION_HIGH)));
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(gen_phase_x, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator_stop_tick, MCPWM_GEN_ACTION_LOW)));

  // --- Finale Schritte ---
  // Timer starten (startet beide Operatoren)
  ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

  // Signal von Tx_CoilPin (erzeugt von Operator A) auch auf den Phase_R_Pin leiten
  // Hinweis: Diese Konstante sollte jetzt funktionieren, da der Generator A von Operator 0 in Gruppe 0 immer dieselbe ID hat.
  pinMatrixOutAttach(Phase_R_Pin, MCPWM0_GEN0_OUT_IDX, false, false);

  Serial.println("DEBUG: MCPWM setup complete with new API.");
}

void setup()
{
  Serial.begin(115200);

  // Mutex erstellen
  g_sharedDataMutex = xSemaphoreCreateMutex();
  if (g_sharedDataMutex == NULL) {
    Serial.println("FATAL ERROR: Konnte g_sharedDataMutex nicht erstellen!");
    // Hier könnten Sie den ESP anhalten, da ein Weiterbetrieb unsicher ist
    while(1) { vTaskDelay(1000); }
  }

  Serial.println("\n\n--- VLF Metal Detector Booting ---");
  Serial.println("DEBUG: Initializing Task Watchdog...");
  // Konfiguration für den Watchdog definieren
  esp_task_wdt_config_t twdt_config = {
    .timeout_ms = WATCHDOG_TIMEOUT * 1000,
    .idle_core_mask = (1 << 0) | (1 << 1),
    .trigger_panic = true,
  };

  // Prüfen, ob der Watchdog bereits initialisiert ist
  Serial.println("DEBUG: Configuring Task Watchdog...");
  // Wir versuchen ZUERST, neu zu konfigurieren.
  esp_err_t wdt_status = esp_task_wdt_reconfigure(&twdt_config);

  if (wdt_status == ESP_ERR_INVALID_STATE) {
    // Fehler: WDT war nicht initialisiert. Jetzt initialisieren.
    Serial.println("DEBUG: WDT not initialized. Initializing...");
    if (esp_task_wdt_init(&twdt_config) != ESP_OK) {
        Serial.println("ERROR: WDT Init failed!");
    }
  } 
  else if (wdt_status != ESP_OK) {
    // Ein anderer Fehler bei der Rekonfiguration
    Serial.println("ERROR: WDT Reconfigure failed!");
  } 
  else {
    Serial.println("DEBUG: WDT was already running and is now reconfigured.");
  }

  BLEDevice::init("ESP32 Metal Detector");
  BLEServer *pServer = BLEDevice::createServer();
  
  if (!pServer) { Serial.println("ERROR: Failed to create BLE Server!"); return; }
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  if (!pService) { Serial.println("ERROR: Failed to create BLE Service!"); return; }
  pCharacteristic_RX = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);

  if (!pCharacteristic_RX) { Serial.println("ERROR: Failed to create RX Characteristic!"); return; }
  pCharacteristic_RX->setCallbacks(new MyCallbacks());
  pCharacteristic_TX = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);

  if (!pCharacteristic_TX) { Serial.println("ERROR: Failed to create TX Characteristic!"); return; }
  
  // Erstelle den Deskriptor und weise ihm den Callback zu
  BLE2902* p2902 = new BLE2902();
  p2902->setCallbacks(new My2902Callbacks());
  pCharacteristic_TX->addDescriptor(p2902);

  pService->start();

  // Explizites Advertising-Objekt erstellen und konfigurieren
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);

  // Fügt die Service UUID zum Haupt-Paket hinzu
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0); // Für schnellere Verbindungserkennung (optional)
  BLEDevice::startAdvertising();
  Serial.println("DEBUG: BLE Server and Advertising started.");

  analogSetWidth(12); 
  analogSetAttenuation(ADC_11db);
  pinMode(Rx_XPin, INPUT);
  pinMode(Rx_RPin, INPUT);
  pinMode(LedPin, OUTPUT);
  pinMode(BatPin, INPUT);
  // Beleuchtungspin konfigurieren
  pinMode(LED_lighting, OUTPUT);
  digitalWrite(LED_lighting, LOW); // Startzustand: Aus

  digitalWrite(LedPin, LOW);
  if(!EEPROM.begin(EEPROM_SIZE)){ Serial.println("ERROR: Failed to initialise EEPROM!"); }
  params.loadParams();

  // Prüfen, ob nach dem Laden aus dem EEPROM ein unsicherer Wert für maxSignal vorliegt.
  // Ein Wert von 0 oder sehr klein löst die Overload-Falle aus.
  if (params.m_maxSignal < SAFE_MAX_SIGNAL_MIN || params.m_maxSignal > ADC_MAX_VALUE) {
    Serial.printf("WARNUNG: Unsicherer m_maxSignal-Wert (%u) nach EEPROM-Laden erkannt!\n", params.m_maxSignal);
    params.m_maxSignal = (ADC_MAX_VALUE * 0.95f); // Sicherer Standardwert
    Serial.printf("INFO: m_maxSignal wurde auf einen sicheren Standardwert (%u) gesetzt.\n", params.m_maxSignal);
  }

  // Prüfen, ob m_readingSamples nach dem EEPROM-Laden gültig ist.
  // Muss größer als ein Minimum (z.B. 20) und kleiner/gleich dem physischen Puffer sein.
  if (params.m_readingSamples < MIN_READING_SAMPLES || params.m_readingSamples > PHYSICAL_BUFFER_SIZE) {
    Serial.printf("WARNUNG: Unsicherer m_readingSamples-Wert (%d) erkannt!\n", params.m_readingSamples);
    params.m_readingSamples = PHYSICAL_BUFFER_SIZE; // Setzt auf sicheren Maximalwert
    Serial.printf("INFO: m_readingSamples wurde auf %d gesetzt.\n", params.m_readingSamples);
  }

  Serial.println("DEBUG: Performing initial battery check...");

  // Zuerst die Kalibrierung durchführen
  setup_adc_calibration(); 

  // Führe eine erste Messung durch und speichere sie Thread-sicher
  int initialBatteryValue = getBatteryVoltage_mV();
  xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
  params.m_Battery = initialBatteryValue;
  xSemaphoreGive(g_sharedDataMutex);
  Serial.printf("DEBUG: Initial battery voltage: %d mV\n", initialBatteryValue);

  dac_oneshot_handle_t dac_handle;
  dac_oneshot_config_t dac_config = { .chan_id = DAC_CHAN_1 };
  if(dac_oneshot_new_channel(&dac_config, &dac_handle) != ESP_OK){ Serial.println("ERROR: Failed to create DAC channel!"); }
  
  if(dac_oneshot_output_voltage(dac_handle, DAC_VREF_OUTPUT_LEVEL) != ESP_OK){ Serial.println("ERROR: Failed to output DAC voltage!"); }
  setupMCPWM();
  Serial.println("\n--- Boot Finish Print Info ---");
  Serial.printf("Hardware Version: %s\n", HARDWARE_VERSION);
  Serial.printf("Firmware Version: %s\n", FIRMWARE_VERSION);

  adcQueue = xQueueCreate(ADC_QUEUE_LENGTH, sizeof(AdcData));
  if (adcQueue != NULL) {
    // 1. Tasks erstellen (Handle wird hier zugewiesen)
    if(xTaskCreatePinnedToCore(adc_sampling_task, "ADC Sampler", 4096, NULL, 1, &adcTaskHandle, 1) != pdPASS){ 
      Serial.println("ERROR: Failed to create ADC Task!");
    }
    
    if(xTaskCreatePinnedToCore(main_logic_task, "Main Logic", 4096, NULL, 2, &processingTaskHandle, 1) != pdPASS){ 
      Serial.println("ERROR: Failed to create Main Task!");
    }

    // 2. JETZT den präzisen Timer starten (da adcTaskHandle nun gültig ist)
    setup_precise_adc_timer(); 
  } 
  else {
    Serial.println("ERROR: Failed to create ADC Queue!");
  }
  ledStatus = 1;
}

void loop()
{   
  vTaskDelay(pdMS_TO_TICKS(1000));
}

//====================================================================================
// Funktionen
//====================================================================================
#ifndef USE_LEGACY_STRING_PROTOCOL
/**
 * @brief Baut ein spezielles Einstellungs-Paket zusammen und sendet es.
 * @param setting_type Der Befehl, der den Typ der Einstellung angibt (z.B. CMD_PUSH_SENSITIVITY).
 * @param value Der 16-bit Wert der Einstellung.
 */
void sendSettingsPacket(uint8_t setting_type, int16_t value) {
    if (!deviceConnected) return;
    uint8_t packet[8];
    packet[0] = DATA_HEADER_SETTINGS; // Neuer Header zur Unterscheidung
    packet[1] = setting_type;
    packet[2] = value & 0xFF;         // Low-Byte
    packet[3] = (value >> 8) & 0xFF;  // High-Byte
    packet[4] = 0x00; // Reserviert
    packet[5] = 0x00; // Reserviert
    packet[6] = 0x00; // Reserviert

    uint8_t checksum = 0;
    for (int i = 0; i < 7; i++) {
        checksum ^= packet[i];
    }
    packet[7] = checksum;

    pCharacteristic_TX->setValue(packet, sizeof(packet));
    pCharacteristic_TX->notify();
    #ifdef DEBUG_TX_PACKETS
    Serial.printf("DEBUG TX SETTINGS: H=0x%02X, TYPE=0x%02X, VAL=%d, CS=0x%02X\n",
                  packet[0], packet[1], value, packet[7]);
    #endif
}
#endif

#ifndef USE_LEGACY_STRING_PROTOCOL
/**
 * @brief Eine Task, die alle aktuellen Einstellungen an die App sendet.
 * Wird als eigene Task ausgeführt, um den BLE-Callback nicht zu blockieren.
 * @param pvParameters Nicht verwendet.
 */
void sendAllSettingsTask(void *pvParameters) {
    Serial.println("DEBUG: Starting task to send all settings...");
    vTaskDelay(pdMS_TO_TICKS(500)); 

    // --- Snapshot-Struktur ---
    struct {
        uint16_t minSignal;
        uint16_t readingSamples;
        uint16_t phaseQuadrature;
        uint16_t xBias;
        uint16_t rBias;
        uint16_t notchLower;
        uint16_t notchUpper;
    } local_params;

    // 1. Mutex nehmen, alle Werte kopieren
    xSemaphoreTake(g_sharedDataMutex, portMAX_DELAY);
    local_params.minSignal = params.m_minSignal;
    local_params.readingSamples = params.m_readingSamples;
    local_params.phaseQuadrature = params.m_Phase_Quadrature;
    local_params.xBias = params.m_XBias;
    local_params.rBias = params.m_RBias;
    local_params.notchLower = params.m_notchLowerBound;
    local_params.notchUpper = params.m_notchUpperBound;
    // 2. Mutex SOFORT wieder freigeben
    xSemaphoreGive(g_sharedDataMutex);

    // 3. Senden und Verzögern OHNE Mutex-Sperre
    Serial.println("DEBUG: Sending settings snapshot...");
    sendSettingsPacket(CMD_PUSH_SENSITIVITY, local_params.minSignal);
    vTaskDelay(pdMS_TO_TICKS(SETTINGS_PACKET_DELAY_MS)); 

    sendSettingsPacket(CMD_PUSH_SAMPLERATE, local_params.readingSamples);
    vTaskDelay(pdMS_TO_TICKS(SETTINGS_PACKET_DELAY_MS));

    sendSettingsPacket(CMD_PUSH_PHASESHIFT, local_params.phaseQuadrature);
    vTaskDelay(pdMS_TO_TICKS(SETTINGS_PACKET_DELAY_MS));

    sendSettingsPacket(CMD_PUSH_GROUND_X, local_params.xBias);
    vTaskDelay(pdMS_TO_TICKS(SETTINGS_PACKET_DELAY_MS));

    sendSettingsPacket(CMD_PUSH_GROUND_R, local_params.rBias);
    vTaskDelay(pdMS_TO_TICKS(SETTINGS_PACKET_DELAY_MS));

    int16_t notch_val = (uint8_t)local_params.notchUpper << 8 |
                        (uint8_t)local_params.notchLower;
    sendSettingsPacket(CMD_PUSH_NOTCH, notch_val);

    Serial.println("DEBUG: Finished sending all settings.");
    vTaskDelete(NULL);
}
#endif

//====================================================================================
// ISR Funktionen
//====================================================================================
/**
 * @brief Timer-Callback (ISR-Kontext!)
 * Feuert präzise alle 5ms und weckt die ADC-Task auf.
 */
void IRAM_ATTR onAdcTimer(void* arg) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Sendet eine "Benachrichtigung" an die ADC-Task, um sie zu entsperren
    if (adcTaskHandle != NULL) {
        vTaskNotifyGiveFromISR(adcTaskHandle, &xHigherPriorityTaskWoken);
    }
    
    // Erzwingt einen sofortigen Kontextwechsel, falls die ADC-Task nun die höchste Priorität hat
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}