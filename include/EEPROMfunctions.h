// EEPROM functions and definitions not included in standart <EEPROM.h> library
#ifndef __EEPROMfunctions_H
#define __EEPROMfunctions_H

// EEPROM functions
// Write a unsigned int (two bytes) value to eeprom
void eeprom_writeInt(uint16_t address, uint16_t value);
void eeprom_updateInt(uint16_t address, uint16_t value);
// read a unsigned int (two bytes) value from eeprom
uint16_t eeprom_readInt(uint16_t address);
void EEPROM_init();
void config_loadFromEEPROM();
void config_writeDefaultsToEEPROM();
void reset_writeDefaultsToEEPROM();

#endif