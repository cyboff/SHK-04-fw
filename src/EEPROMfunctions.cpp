// EEPROM functions and definitions not included in standart <EEPROM.h> library
#include <Arduino.h>
#include <EEPROM.h>

#include "Variables.h"
#include "EEPROMfunctions.h"
volatile uint16_t modbusID = 0;

volatile uint16_t actualSpeed = 0; // array index
volatile uint32_t modbusSpeed = 0; // default 19200
volatile uint16_t actualFormat = 0;
volatile uint16_t modbusFormat = 0;

boolean dataSent = false;
int sendNextLn = 0;
volatile uint16_t io_state = 0;
unsigned long exectime = 0;
unsigned long pulsetime = 0;

volatile uint16_t windowBegin = 0, windowEnd = 0, positionOffset = 0, positionMode = 0, analogOutMode = 0;
volatile uint16_t filterPosition = 0, filterOn = 0, filterOff = 0;

volatile uint16_t thre256 = 0, thre = 0, thre1 = 0, thre2 = 0;
volatile uint16_t set = 0, pga = 0, pga1 = 0, pga2 = 0;

// diagnosis
volatile uint16_t celsius = 0; // internal temp in deg of Celsius
volatile uint16_t temp = 0;    // internal ADC Temp channel value
volatile uint16_t max_temperature = 0;
volatile uint16_t total_runtime = 0;

// EEPROM

// Write a unsigned int (two bytes) value to eeprom
void eeprom_writeInt(uint16_t address, uint16_t value)
{

  EEPROM.write(address, value % 256);     // LSB
  EEPROM.write(address + 1, value / 256); // MSB
#if defined(__IMXRT1062__)                // Teensy 4.0
  asm("DSB");
#endif
}

void eeprom_updateInt(uint16_t address, uint16_t value)
{

  EEPROM.update(address, value % 256);     // LSB
  EEPROM.update(address + 1, value / 256); // MSB
#if defined(__IMXRT1062__)                 // Teensy 4.0
  asm("DSB");
#endif
}

// read a unsigned int (two bytes) value from eeprom
uint16_t eeprom_readInt(uint16_t address)
{

  return EEPROM.read(address) + EEPROM.read(address + 1) * 256;
// return EEPROM.read(address) | EEPROM.read(address + 1) << 8;
#if defined(__IMXRT1062__) // Teensy 4.0
  asm("DSB");
#endif
}

void EEPROM_init()
{
  EEPROM.begin();

  if (
      eeprom_readInt(EE_ADDR_MODEL_TYPE) == MODEL_TYPE &&
      eeprom_readInt(EE_ADDR_MODEL_SERIAL_NUMBER) == MODEL_SERIAL_NUMBER &&
      eeprom_readInt(EE_ADDR_FW_VERSION) == FW_VERSION)
  {

    // loads in ram the eeprom config
    config_loadFromEEPROM();
  }
  else
  {
    config_writeDefaultsToEEPROM();
    config_loadFromEEPROM();
  }
#if defined(__IMXRT1062__) // Teensy 4.0
  asm("DSB");
#endif
}

void config_loadFromEEPROM()
{
  // loads in ram the eeprom config
  modbusID = eeprom_readInt(EE_ADDR_modbus_ID);
  modbusSpeed = eeprom_readInt(EE_ADDR_modbus_Speed) * 100; // speed/100 to fit 115200 in WORD
  modbusFormat = eeprom_readInt(EE_ADDR_modbus_Format);

  set = eeprom_readInt(EE_ADDR_set);
  pga1 = eeprom_readInt(EE_ADDR_gain_set1);
  thre1 = eeprom_readInt(EE_ADDR_threshold_set1);
  pga2 = eeprom_readInt(EE_ADDR_gain_set2);
  thre2 = eeprom_readInt(EE_ADDR_threshold_set2);

  windowBegin = eeprom_readInt(EE_ADDR_window_begin);
  windowEnd = eeprom_readInt(EE_ADDR_window_end);
  positionMode = eeprom_readInt(EE_ADDR_position_mode);
  analogOutMode = eeprom_readInt(EE_ADDR_analog_out_mode);
  positionOffset = eeprom_readInt(EE_ADDR_position_offset);

  filterPosition = eeprom_readInt(EE_ADDR_filter_position);
  filterOn = eeprom_readInt(EE_ADDR_filter_on);
  filterOff = eeprom_readInt(EE_ADDR_filter_off);

  max_temperature = eeprom_readInt(EE_ADDR_max_temperature);
  total_runtime = eeprom_readInt(EE_ADDR_total_runtime);
#if defined(__IMXRT1062__) // Teensy 4.0
  asm("DSB");
#endif
}

void config_writeDefaultsToEEPROM()
{ // flash is empty or program version changed
  // writes sign codes
  eeprom_writeInt(EE_ADDR_MODEL_TYPE, MODEL_TYPE);
  eeprom_writeInt(EE_ADDR_MODEL_SERIAL_NUMBER, MODEL_SERIAL_NUMBER);
  eeprom_writeInt(EE_ADDR_FW_VERSION, FW_VERSION);

  // save defaults to eeprom
  eeprom_writeInt(EE_ADDR_modbus_ID, DEFAULT_MODBUS_ID);
  eeprom_writeInt(EE_ADDR_modbus_Speed, DEFAULT_MODBUS_SPEED / 100); // speed/100 to fit 115200 in WORD
  eeprom_writeInt(EE_ADDR_modbus_Format, DEFAULT_MODBUS_FORMAT);

  eeprom_writeInt(EE_ADDR_set, DEFAULT_SET);
  eeprom_writeInt(EE_ADDR_gain_set1, DEFAULT_GAIN_SET1);
  eeprom_writeInt(EE_ADDR_threshold_set1, DEFAULT_THRESHOLD_SET1);
  eeprom_writeInt(EE_ADDR_gain_set2, DEFAULT_GAIN_SET2);
  eeprom_writeInt(EE_ADDR_threshold_set2, DEFAULT_THRESHOLD_SET2);

  eeprom_writeInt(EE_ADDR_window_begin, DEFAULT_WINDOW_BEGIN);
  eeprom_writeInt(EE_ADDR_window_end, DEFAULT_WINDOW_END);
  eeprom_writeInt(EE_ADDR_position_mode, DEFAULT_POSITION_MODE);
  eeprom_writeInt(EE_ADDR_analog_out_mode, DEFAULT_ANALOG_OUT_MODE);
  eeprom_writeInt(EE_ADDR_position_offset, DEFAULT_POSITION_OFFSET);

  eeprom_writeInt(EE_ADDR_filter_position, DEFAULT_FILTER_POSITION);
  eeprom_writeInt(EE_ADDR_filter_on, DEFAULT_FILTER_ON);
  eeprom_writeInt(EE_ADDR_filter_off, DEFAULT_FILTER_OFF);

  eeprom_writeInt(EE_ADDR_max_temperature, max_temperature);
  eeprom_writeInt(EE_ADDR_total_runtime, total_runtime);
#if defined(__IMXRT1062__) // Teensy 4.0
  asm("DSB");
#endif
}

void reset_writeDefaultsToEEPROM()
{ // reset all values (except calibration values) to fact. defaults
  // writes sign codes
  // eeprom_writeInt(EE_ADDR_MODEL_TYPE, MODEL_TYPE);
  // eeprom_writeInt(EE_ADDR_MODEL_SERIAL_NUMBER, MODEL_SERIAL_NUMBER);
  // eeprom_writeInt(EE_ADDR_FW_VERSION, FW_VERSION);

  // save defaults to eeprom
  eeprom_writeInt(EE_ADDR_modbus_ID, DEFAULT_MODBUS_ID);
  eeprom_writeInt(EE_ADDR_modbus_Speed, DEFAULT_MODBUS_SPEED / 100); // speed/100 to fit 115200 in WORD
  eeprom_writeInt(EE_ADDR_modbus_Format, DEFAULT_MODBUS_FORMAT);

  eeprom_writeInt(EE_ADDR_set, DEFAULT_SET);
  eeprom_writeInt(EE_ADDR_gain_set1, DEFAULT_GAIN_SET1);
  eeprom_writeInt(EE_ADDR_threshold_set1, DEFAULT_THRESHOLD_SET1);
  eeprom_writeInt(EE_ADDR_gain_set2, DEFAULT_GAIN_SET2);
  eeprom_writeInt(EE_ADDR_threshold_set2, DEFAULT_THRESHOLD_SET2);

  eeprom_writeInt(EE_ADDR_window_begin, DEFAULT_WINDOW_BEGIN);
  eeprom_writeInt(EE_ADDR_window_end, DEFAULT_WINDOW_END);
  eeprom_writeInt(EE_ADDR_position_mode, DEFAULT_POSITION_MODE);
  eeprom_writeInt(EE_ADDR_analog_out_mode, DEFAULT_ANALOG_OUT_MODE);
  // eeprom_writeInt(EE_ADDR_position_offset, DEFAULT_POSITION_OFFSET);

  eeprom_writeInt(EE_ADDR_filter_position, DEFAULT_FILTER_POSITION);
  eeprom_writeInt(EE_ADDR_filter_on, DEFAULT_FILTER_ON);
  eeprom_writeInt(EE_ADDR_filter_off, DEFAULT_FILTER_OFF);

// eeprom_writeInt(EE_ADDR_max_temperature, max_temperature);
// eeprom_writeInt(EE_ADDR_total_runtime, total_runtime);
#if defined(__IMXRT1062__) // Teensy 4.0
  asm("DSB");
#endif
}
