#ifndef MD_DEFS_H
#define MD_DEFS_H

#define HARDWARE_VERSION "V1.0"
#define FIRMWARE_VERSION "V2.0"

//====================================================================================
// Feature- & Kompatibilitäts-Schalter
//====================================================================================
/**
 * @brief Kompatibilitätsschalter für das Kommunikationsprotokoll.
 * @details Wenn diese Zeile aktiviert ist, verwendet die Firmware das alte, String-basierte
 * Protokoll für die Kompatibilität mit älteren Apps.
 Wenn sie auskommentiert ist,
 * wird das neue, effiziente Binärprotokoll verwendet.
 */

#define USE_LEGACY_STRING_PROTOCOL

// Kommentieren Sie diese Zeile aus, um die externen AD7685 ADCs via SPI zu verwenden.
#define ENABLE_INTERNAL_ADC

// Automatische "Ground" Anpassung aktiv - Anpassung wenn in zwei Sekunden kein Metall detektiert wurde
#define ENABLE_GROUND_TRACKING

// Debugging gesendeter Pakete aktiv (nur für Binärprotokoll)
#ifndef USE_LEGACY_STRING_PROTOCOL
  #define DEBUG_TX_PACKETS
#endif

//====================================================================================
// Konstanten
//====================================================================================
// --- ADC-Auflösungskonstanten ---
#define INTERNAL_ADC_MAX_VALUE 4095 // 12-Bit für esp_adc_cal

#ifdef ENABLE_INTERNAL_ADC
  #define ADC_MAX_VALUE INTERNAL_ADC_MAX_VALUE  // 12-Bit
  #define ADC_DEFAULT_BIAS 2048                 // 12-Bit Mitte
#else
  #define ADC_MAX_VALUE 65535       // 16-Bit
  #define ADC_DEFAULT_BIAS 32768    // 16-Bit Mitte
#endif

//====================================================================================
// Pin-Definitionen laut Schaltplan
//====================================================================================
#define Rx_XPin      34  // SENSOR_VP
#define Rx_RPin      35  // SENSOR_VN
#define Tx_CoilPin   25
#define VrefPin      26
#define Phase_X_Pin  32
#define Phase_R_Pin  33
#define LedPin       2
#define BatPin       39
#define LED_lighting 4  // Pin für Beleuchtungs-LED

// Pins für externe ADCs (SPI)
#define SPI_SCK_PIN   18
#define SPI_MISO_PIN  19
#define ADC1_CS_PIN   5  // Chip Select für X-Kanal
#define ADC2_CS_PIN   17 // Chip Select für R-Kanal

#endif