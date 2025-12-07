#ifndef MD_PARAMS_H
#define MD_PARAMS_H

#include <cstdint>

#define EEPROM_MAGIC_BYTE 0xAB // Ein "magischer" Wert
#define EEPROM_VERSION    0x02 // Erhöhe dies, wenn du Parameter änderst

/**
 * @class CParameters
 * @brief Verwaltet alle einstellbaren Parameter und speichert/lädt sie aus dem EEPROM.
 */
class CParameters
{
public:
	/** @brief Konstruktor für die Parameter-Klasse. */
	CParameters();

	/** @brief Setzt alle Parameter auf ihre Standardwerte und speichert sie im EEPROM. */
	void setDefaults();

	/** @brief Lädt alle Parameter aus dem EEPROM in die Klassenmember. */
	void loadParams();

  /** @brief Speichert die aktuellen Parameter-Werte ins EEPROM. */
	void saveParams();

	/** @brief Minimale Signalstärke, um einen Peak auszulösen. (eeprom loc 2) */
	uint16_t m_minSignal; 		        
	/** @brief Kalibrierter Nullpunkt (Bias) für den X-Kanal. (eeprom loc 4) */
	uint16_t m_XBias; 			          
	/** @brief Kalibrierter Nullpunkt (Bias) für den R-Kanal. (eeprom loc 6) */
	uint16_t m_RBias; 			          
	/** @brief Maximales ADC-Signal vor Overload-Warnung. (eeprom loc 8) */
  uint16_t m_maxSignal;		        
	/** @brief Legacy-Parameter. (eeprom loc 10) */
	uint16_t m_readingSamples;  	    
	/** @brief Polarität des R-Signals. (eeprom loc 12) */
  int m_sigPolarity;    	    
	/** @brief Phasenverschiebung (Quadratur). (eeprom loc 14) */
  uint16_t m_Phase_Quadrature;     
	/** @brief Phasenverschiebung (In-Phase). (eeprom loc 16) */
  uint16_t m_Phase_InPhase;        
	/** @brief Betriebsfrequenz der Spule. (eeprom loc 18) */
  uint16_t m_CoilFrequency;        
	/** @brief Batteriespannung (z.B. 370 für 3.70V). (eeprom loc 20) */
  uint16_t m_Battery;              

    // Parameter für Notch-Filter
	/** @brief Untere Grenze für den Notch-Filter. (eeprom loc 22) */
  uint16_t m_notchLowerBound;      
	/** @brief Obere Grenze für den Notch-Filter. (eeprom loc 24) */
  uint16_t m_notchUpperBound;      
};

void EEPROMWriteUInt16(int p_address, uint16_t p_value);
uint16_t EEPROMReadUInt16(int p_address);

void EEPROMWriteInt(int p_address, int p_value);
int EEPROMReadInt(int p_address);

#endif