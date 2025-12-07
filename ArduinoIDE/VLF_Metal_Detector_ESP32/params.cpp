#include <Arduino.h>
#include <EEPROM.h>
#include "params.h"
#include "defs.h"

void EEPROMWriteUInt16(int p_address, uint16_t p_value)
{
  unsigned char lowByte = ((p_value >> 0) & 0xFF);
  unsigned char highByte = ((p_value >> 8) & 0xFF);
  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

uint16_t EEPROMReadUInt16(int p_address)
{
  unsigned char lowByte = EEPROM.read(p_address);
  unsigned char highByte = EEPROM.read(p_address + 1);
  // Explizit zu uint16_t casten
  return (uint16_t)((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

// Implementierungen für die Int-Funktionen (nötig für m_sigPolarity)
void EEPROMWriteInt(int p_address, int p_value)
{
  // Annahme: "int" ist 16-bit für EEPROM-Speicherung
  unsigned char lowByte = ((p_value >> 0) & 0xFF);
  unsigned char highByte = ((p_value >> 8) & 0xFF);
  EEPROM.write(p_address, lowByte);
  EEPROM.write(p_address + 1, highByte);
}

int EEPROMReadInt(int p_address)
{
  // Annahme: "int" ist 16-bit für EEPROM-Speicherung
  unsigned char lowByte = EEPROM.read(p_address);
  unsigned char highByte = EEPROM.read(p_address + 1);
  // Muss als (int16_t) behandelt werden, um das Vorzeichen zu erhalten
  return (int)((int16_t)((highByte << 8) | lowByte));
}

CParameters::CParameters()
{
  // Der Konstruktor kann leer bleiben. Die Initialisierung
  // erfolgt sauber über loadParams() in setup().
}

void CParameters::setDefaults()
{
  m_minSignal        = 20;

  m_XBias            = ADC_DEFAULT_BIAS;
  m_RBias            = ADC_DEFAULT_BIAS;
  m_maxSignal        = (ADC_MAX_VALUE * 0.95f); // z.B. 95% des Maximalsignals

  m_readingSamples   = 20;       
  m_sigPolarity      = 1;
  m_Phase_Quadrature = 90;
  m_Phase_InPhase    = 0;
  m_CoilFrequency    = 8333;
  m_Battery          = 0;
  m_notchLowerBound  = 0;
  m_notchUpperBound  = 0;

  EEPROM.write(0, EEPROM_MAGIC_BYTE);
  EEPROM.write(1, EEPROM_VERSION);
  EEPROMWriteUInt16(2, m_minSignal);
  EEPROMWriteUInt16(4, m_XBias);
  EEPROMWriteUInt16(6, m_RBias);
  EEPROMWriteUInt16(8, m_maxSignal);
  EEPROMWriteUInt16(10, m_readingSamples);                   
  EEPROMWriteInt(12, m_sigPolarity);
  EEPROMWriteUInt16(14, m_Phase_Quadrature);
  EEPROMWriteUInt16(16, m_Phase_InPhase);
  EEPROMWriteUInt16(18, m_CoilFrequency);
  EEPROMWriteUInt16(20, m_Battery);
  EEPROMWriteUInt16(22, m_notchLowerBound);
  EEPROMWriteUInt16(24, m_notchUpperBound);
  EEPROM.commit();
};

void CParameters::loadParams()
{
  // NEU: Prüfe Magic Byte und Version
  if (EEPROM.read(0) != EEPROM_MAGIC_BYTE || EEPROM.read(1) != EEPROM_VERSION)
  {
    // Wenn die Werte nicht stimmen (leeres EEPROM oder alte Version),
    // lade die Standardwerte und speichere sie sofort.
    setDefaults();
  }
  else
  {
    // Nur wenn die Bytes stimmen, lade die gespeicherten Werte
    m_minSignal         = EEPROMReadUInt16(2);
    m_XBias             = EEPROMReadUInt16(4);
    m_RBias             = EEPROMReadUInt16(6);
    m_maxSignal         = EEPROMReadUInt16(8);
    m_readingSamples    = EEPROMReadUInt16(10);
    m_sigPolarity       = EEPROMReadInt(12);
    m_Phase_Quadrature  = EEPROMReadUInt16(14);
    m_Phase_InPhase     = EEPROMReadUInt16(16);
    m_CoilFrequency     = EEPROMReadUInt16(18);
    m_Battery           = EEPROMReadUInt16(20);
    m_notchLowerBound   = EEPROMReadUInt16(22);
    m_notchUpperBound   = EEPROMReadUInt16(24);
  }
}

void CParameters::saveParams()
{
  // Speichere die *aktuellen* Werte der Member-Variablen ins EEPROM
  EEPROM.write(0, EEPROM_MAGIC_BYTE);
  EEPROM.write(1, EEPROM_VERSION);
  EEPROMWriteUInt16(2, m_minSignal);
  EEPROMWriteUInt16(4, m_XBias);
  EEPROMWriteUInt16(6, m_RBias);
  EEPROMWriteUInt16(8, m_maxSignal);
  EEPROMWriteUInt16(10, m_readingSamples);                   
  EEPROMWriteInt(12, m_sigPolarity);        
  EEPROMWriteUInt16(14, m_Phase_Quadrature);
  EEPROMWriteUInt16(16, m_Phase_InPhase);
  EEPROMWriteUInt16(18, m_CoilFrequency);
  // EEPROMWriteInt(20, m_Battery); // Die Batterie-Spannung sollte nicht gespeichert werden
  EEPROMWriteUInt16(22, m_notchLowerBound);
  EEPROMWriteUInt16(24, m_notchUpperBound);
  EEPROM.commit();
}