// EEPROM functions and definitions not included in standart <EEPROM.h> library
#include <Arduino.h>
// #include <EEPROM.h>
#include <Wire.h>

#include "Variables.h"
#include "EEPROMfunctions.h"

#define ADDRESS_OFFSET 100  // offset for EEPROM address, when there is problem with EEPROM 
// EEPROM

// Write a unsigned int (two bytes) value to eeprom
void eeprom_writeInt(uint16_t address, uint16_t value)
{
  uint16_t addressoffset = address + ADDRESS_OFFSET;
  //   __disable_irq();
  //   EEPROM.write(address, value & 0xFF);   // LSB
  //   EEPROM.write(address + 1, (value >> 8) & 0xFF); // MSB
  // #if defined(SERIAL_DEBUG)
  //   Serial.printf("EEwr a: %u w: %u r: %u\n", address, value, eeprom_readInt(address));
  // #endif
  //   __enable_irq();

#if defined(SERIAL_DEBUG)
  Serial.printf("EEwr addr: %u before: %u", address, eeprom_readInt(address));
#endif
  // using 24LC512 i2c EEPROM address 0x50
  Wire.beginTransmission(0x50);
  int error = Wire.endTransmission();
  if (!error)
  {
    Wire.beginTransmission(0x50);
    Wire.write((addressoffset >> 8) & 0xFF); // 24LC512 is using 16bit address, need send only for first byte of data, pointer address is incrementing automaticaly
    Wire.write(addressoffset & 0xFF);
    Wire.write((value >> 8) & 0xFF); // write MSB of data
    Wire.write(value & 0xFF);        // write LSB of data next
    Wire.endTransmission();
    delay(10); // wait for writing to EEPROM
  }

#if defined(SERIAL_DEBUG)
  Serial.printf("  write: %u after: %u\n", value, eeprom_readInt(address));
#endif
}

// read a unsigned int (two bytes) value from eeprom
uint16_t eeprom_readInt(uint16_t address)
{
  uint16_t addressoffset = address + ADDRESS_OFFSET;
  unsigned int data[2] = {0};
  // return EEPROM.read(address) | (EEPROM.read(address + 1) << 8);
  Wire.beginTransmission(0x50);
  int error = Wire.endTransmission();
  if (!error) // 24LC512 is responding
  {
    // Start I2C transmission
    Wire.beginTransmission(0x50);
    Wire.write(((addressoffset >> 8) & 0xFF)); // MSB of address 24LC512 is using 16bit address, need send only for first byte of data, pointer address is incrementing automaticaly
    Wire.write((addressoffset & 0xFF));        // LSB of address

    // Stop I2C transmission
    Wire.endTransmission();

    // Request 2 byte of data
    Wire.requestFrom(0x50, 2);

    // Read 2 bytes of data
    // temp msb, temp lsb
    if (Wire.available() == 2)
    {
      data[0] = Wire.read(); // read MSB of value
      data[1] = Wire.read(); // read LSB of value
    }
  }
// #if defined(SERIAL_DEBUG)
//   Serial.printf("EEread addr: %u data: %u\n", address, ((data[0] << 8) | data[1]));
// #endif

  return (data[0] << 8) | data[1];
}

void EEPROM_init()
{
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
  gainOffset = eeprom_readInt(EE_ADDR_gain_offset); // 0 to 255

  windowBegin = eeprom_readInt(EE_ADDR_window_begin);
  windowEnd = eeprom_readInt(EE_ADDR_window_end);
  positionMode = eeprom_readInt(EE_ADDR_position_mode);
  analogOutMode = eeprom_readInt(EE_ADDR_analog_out_mode);
  positionOffset = eeprom_readInt(EE_ADDR_position_offset);

  filterPosition = eeprom_readInt(EE_ADDR_filter_position);
  filterOn = eeprom_readInt(EE_ADDR_filter_on);
  filterOff = eeprom_readInt(EE_ADDR_filter_off);

  max_temperature = (uint8_t)eeprom_readInt(EE_ADDR_max_temperature);
  total_runtime = eeprom_readInt(EE_ADDR_total_runtime);
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
  eeprom_writeInt(EE_ADDR_modbus_Format, (uint16_t)DEFAULT_MODBUS_FORMAT);

  eeprom_writeInt(EE_ADDR_set, DEFAULT_SET);
  eeprom_writeInt(EE_ADDR_gain_set1, DEFAULT_GAIN_SET1);
  eeprom_writeInt(EE_ADDR_threshold_set1, DEFAULT_THRESHOLD_SET1);
  eeprom_writeInt(EE_ADDR_gain_set2, DEFAULT_GAIN_SET2);
  eeprom_writeInt(EE_ADDR_threshold_set2, DEFAULT_THRESHOLD_SET2);
  eeprom_writeInt(EE_ADDR_gain_offset, DEFAULT_GAIN_OFFSET);

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
  // eeprom_writeInt(EE_ADDR_gain_offset, DEFAULT_GAIN_OFFSET);

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
}
