// firmware for SHK-04 board with Teensy 4.0
#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>

// for i2c
#include <Wire.h>

// for ADC
#include <ADC.h>
#include <DMAChannel.h>

// for SPI
#include <SPI.h>

// for filters
#include <Bounce2.h>

// for ModBus
#include "SimpleModbusSlave.h"

#include "Variables.h"
#include "EEPROMfunctions.h"
#include "DisplayMenus.h"

uint16_t holdingRegs[TOTAL_REGS_SIZE] = {0}; // function 3 and 16 register array

volatile uint16_t modbusID = 0;

volatile uint16_t actualSpeed = 0; // array index
volatile uint32_t modbusSpeed = 0; // default 19200
volatile uint16_t actualFormat = 0;
volatile uint16_t modbusFormat = 0;

volatile uint16_t io_state = 0;

volatile uint16_t windowBegin = 0, windowEnd = 0, positionOffset = 0, positionMode = 0, analogOutMode = 0;
volatile uint16_t filterPosition = 0, filterOn = 0, filterOff = 0;

volatile uint16_t thre256 = 0, thre = 0, thre1 = 0, thre2 = 0;
volatile uint16_t set = 0, pga = 0, pga1 = 0, pga2 = 0, gainOffset = 0, oldgainOffset = 0;

// diagnosis
volatile uint8_t celsius = 0; // internal temp in deg of Celsius
volatile uint16_t temp = 0;   // internal ADC Temp channel value
volatile uint8_t max_temperature = 0;
volatile uint16_t total_runtime = 0;

// Keyboard times
#define BTN_DEBOUNCE_TIME 200 // debounce time (*500us) to prevent flickering when pressing or releasing the button
#define BTN_HOLD_TIME 3000    // holding period (*500us) how long to wait for press+hold event

// create am instance of the LED display library:
LedDisplay myDisplay = LedDisplay(LED_DATA_PIN, LED_RS_PIN, LED_CLK_PIN, LED_EN_PIN, LED_RST_PIN, DISPLAY_LENGHT);
int brightness = 15; // screen brightness

// filters
Bounce filterOnOff = Bounce();
float sma = 0;

// configure ADC
#define ADC0_AVERAGING 1
#define ANALOG_BUFFER_SIZE 500

ADC *adc = new ADC(); // adc object
DMAChannel adc0_dma;

DMAMEM static volatile uint16_t __attribute__((aligned(32))) adc0_buf[ANALOG_BUFFER_SIZE]; // buffer 1...
volatile uint8_t adc_data[ANALOG_BUFFER_SIZE] = {0};                                       // ADC_0 9-bit resolution for differential - sign + 8 bit
volatile uint16_t value_buffer[TOTAL_ERRORS - AN_VALUES] = {0};
volatile boolean adc0_busy = false;
unsigned int freq = 1000000; // PDB frequency
// unsigned int freq = 400000;
volatile int adc0Value = 0;         // analog value
volatile int analogBufferIndex = 0; // analog buffer pointer
volatile int delayOffset = 0;
// sensor variables

int hmdThresholdHyst = 13;
boolean dataSent;
int sendNextLn;
unsigned long exectime = 0;
unsigned long pulsetime = 0;

// Timers
IntervalTimer timer500us; // timer for motor speed and various timeouts
IntervalTimer timerDelay; // timer for precise offset

volatile int motorPulseIndex = 0;
volatile long motorTimeOld = 0;
volatile long motorTimeNow = 0;
volatile int motorTimeDiff = 0;
volatile int peakValue = 0, peakValueOut = 0;
volatile int positionValue = 0;

// Buttons
volatile int BtnPressedATimeout = 0;
volatile int BtnPressedBTimeout = 0;
volatile int BtnPressedCTimeout = 0;
volatile int BtnPressedDTimeout = 0;
volatile boolean BtnReleasedA = true;
volatile boolean BtnReleasedB = true;
volatile boolean BtnReleasedC = true;
volatile boolean BtnReleasedD = true;

// check and load proper settings
// void InitVariables();
void checkSET();
void checkTEST();
void checkALARM();

// SPI send 2 x 16 bit value
void updateSPI(int valueAN1, int valueAN2);

// INTERRUPT ROUTINES

// button interrupts
//*****************************************************************
void checkButtonA();
void checkButtonB();
void checkButtonC();
void checkButtonD();

// Timer interrupts
void timer500us_isr(void);

// motor (from HALL sensor) interrupt
void motor_isr(void);

void callback_delay_isr();
void adc0_dma_isr(void);
void updateResults();

// exponential moving average
long approxSimpleMovingAverage(int new_value, int period);

void checkSTATUS();
void checkModbus();

void setup()
{
#if defined(SERIAL_DEBUG)
  Serial.begin(19200);
  while (!Serial && millis() < 5000)
    ;
  Serial.println("Starting...");
  Serial.println();
#endif

  Wire.begin(); // initialize Teensy as I2C master
  Wire.setClock(400000);
  // initial setup AD5144A
  // check if chip is responding
  Wire.beginTransmission(0x2B);
  int error = Wire.endTransmission();
  if (!error)
  {
    Wire.beginTransmission(0x2B);
    Wire.write(0x18); // write to all RDAC channels
    Wire.write(127);  // set wiper to the middle
    Wire.endTransmission();
  }

  //  initialize LEDs and I/O

  // pinMode(LED_BUILTIN, OUTPUT); //conflicts with SPI_CLK!!!
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_SIGNAL, OUTPUT);
  pinMode(LED_ALARM, OUTPUT);

  pinMode(OUT_SIGNAL_NEG, OUTPUT);
  pinMode(OUT_ALARM_NEG, OUTPUT);

  digitalWrite(LED_POWER, HIGH);
  digitalWrite(LED_SIGNAL, LOW); // connected as OUT_SIGNAL
  digitalWrite(OUT_SIGNAL_NEG, HIGH);
  digitalWrite(LED_ALARM, HIGH);
  digitalWrite(OUT_ALARM_NEG, LOW); // opposite to LED_ALARM

  pinMode(PIN_BTN_A, INPUT_PULLUP);
  pinMode(PIN_BTN_B, INPUT_PULLUP);
  pinMode(PIN_BTN_C, INPUT_PULLUP);
  pinMode(PIN_BTN_D, INPUT_PULLUP);

  pinMode(TEST_IN, INPUT_PULLUP);
  pinMode(SET_IN, INPUT_PULLUP);

  pinMode(LASER, OUTPUT);
  pinMode(IR_LED, OUTPUT);

  // initialize the LED display library:
  myDisplay.begin();
  // set the brightness of the display:
  myDisplay.setBrightness(brightness);

  // use wrapper for myDisplay.print
  displayPrint("Starting");
  delay(500);

  EEPROM_init();

  checkSET();

  // initialize filters
  pinMode(FILTER_PIN, OUTPUT);
  digitalWrite(FILTER_PIN, LOW);
  filterOnOff.attach(FILTER_PIN);
  filterOnOff.interval(filterOn);

  // initialize SPI

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  SPI.begin();
  updateSPI(0, 0);

  // enable serial communication

  holdingRegs[REG_SIZE] = TOTAL_REGS_SIZE;
  // holdingRegs[REG_SIZE] = AN_VALUES;

  holdingRegs[MB_MODEL_TYPE] = MODEL_TYPE;
  holdingRegs[MB_MODEL_SERIAL_NUMBER] = MODEL_SERIAL_NUMBER;
  holdingRegs[MB_FW_VERSION] = FW_VERSION;

  holdingRegs[EXEC_TIME] = 0;

  // Serial1.begin(modbusSpeed);
  // Serial1.flush();
  // Serial1.end();

  // modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, TOTAL_REGS_SIZE, 0);
  modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, 1080, 0); // max address is 1080 in old SDIS protocol

  // initialize ADC

  pinMode(ANALOG_INPUT, INPUT_DISABLE);

  adc->adc0->setAveraging(ADC0_AVERAGING); // set number of averages
  adc->adc0->setResolution(12);            // set bits of resolution - 9 bit for differential

  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED_16BITS, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  // see the documentation for more information
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed
  // it can be ADC_VERY_LOW_SPEED, ADC_LOW_SPEED, ADC_MED_SPEED, ADC_HIGH_SPEED or ADC_VERY_HIGH_SPEED
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

  // Lets setup Analog 0 dma
  adc0_dma.begin();
  adc0_dma.source((volatile uint16_t &)ADC1_R0);
  adc0_dma.destinationBuffer((uint16_t *)adc0_buf, sizeof(adc0_buf));
  adc0_dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);
  adc0_dma.interruptAtCompletion();
  adc0_dma.disableOnCompletion();
  adc0_dma.attachInterrupt(&adc0_dma_isr);

  // Teensy 4.0 uses tempmon instead of ADC
  // tempmon_init();
  // tempmon_Start();

  // motor configuration and startup
  pinMode(MOTOR_ENABLE, OUTPUT);
  digitalWrite(MOTOR_ENABLE, LOW); // motor enable
  pinMode(MOTOR_CLK, OUTPUT);      // motor output pulses

  pinMode(MOTOR_ALARM, INPUT_PULLUP); // motor input pulses
  // NVIC_SET_PRIORITY(IRQ_PORTD, 16);   // pin 14 - PortD, see schematic of Teensy 3.2
  // NVIC_SET_PRIORITY(IRQ_GPIO6789,16);  // Teensy 4.0

  if (positionOffset < 1000) // depends on motor HALL sensors & mirror position - choose the best to have no timing issues
    attachInterrupt(digitalPinToInterrupt(MOTOR_ALARM), motor_isr, FALLING);
  else
    attachInterrupt(digitalPinToInterrupt(MOTOR_ALARM), motor_isr, RISING);

  // motor slow start
  // timer500us.priority(0);
  // timerDelay.priority(16);

  timer500us.begin(timer500us_isr, 50000 / 10); // motor output pulses slowly going to 500us
  for (int speed = 20; speed <= 100; speed++)
  {
    // timer500us.end();
    timer500us.update(50000 / speed); // motor output pulses slowly going to 500us
    displayPrint("Mot=%3d%%", speed);
    delay(150);
  }

  // buttons
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_A), checkButtonA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_B), checkButtonB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_C), checkButtonC, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_BTN_D), checkButtonD, CHANGE);

  for (int i = 0; i < ANALOG_BUFFER_SIZE; i++)
  {
    adc0_buf[i] = 0;
    adc_data[i] = 0;
  }
  // Teensy 4.0 not working without clearing cache
  if ((uint32_t)adc0_buf >= 0x20200000u)
    arm_dcache_flush_delete((void *)adc0_buf, sizeof(adc0_buf));

  adc->adc0->enableDMA();
  // adc0_dma.enable();
  adc->adc0->startSingleRead(ANALOG_INPUT);
  adc->adc0->startQuadTimer(freq); // check ADC_Module::startPDB() in ADC_Module.cpp for //NVIC_ENABLE_IRQ(IRQ_PDB);
}

void loop()
{

  // #if defined(SERIAL_DEBUG)
  //   // print all used EEPROM words
  //   if (!testTimeout)
  //   {
  //     Serial.print("EEreadL: ");
  //     for (int i = 0; i < 21; i++)
  //     {
  //       Serial.printf("%u ", EEPROM.read(i * 2) | EEPROM.read(i * 2 + 1) << 8);
  //     }
  //     Serial.println();
  //     Serial.print("hRegL: ");
  //     for (int i = 0; i < TOTAL_REGS_SIZE; i++)
  //     {
  //       Serial.printf("%u ", holdingRegs[i]);
  //     }
  //     Serial.println();

  //     testTimeout = 5000; // 5 sec
  //   }
  // #endif

  // check SET
  checkSET();
  // check TEST
  checkTEST();
  // check ALARMS and WARNINGS
  checkALARM();

  // check IO STATUS
  checkSTATUS();

  // update modbus
  checkModbus();

  // show info on LED display
  displayMenu();
}

//******************************************************************
// check SET and load proper settings
void checkSET()
{
  uint16_t oldpga = pga; // save old value to update RDAC only if pga changed

  switch (set) // RELAY = 3 (REL1 || REL2), MAN1 = 1, MAN2 = 2
  {
  case 1:
    pga = pga1;
    thre = thre1;
    setDispIndex = 0; // MAN1
    break;
  case 2:
    pga = pga2;
    thre = thre2;
    setDispIndex = 1; // MAN2
    break;
  case 3: // REL
    if (digitalRead(SET_IN))
    {
      pga = pga1;
      thre = thre1;
      setDispIndex = 3; // REL1
    }
    else
    {
      pga = pga2;
      thre = thre2;
      setDispIndex = 4; // REL2
    }
    break;
  default:
    break;
  }
  thre256 = map(thre, 0, 100, 0, 255);

  if (oldpga != pga)
  { // update gain: AD5144A RDAC2+RDAC4
    uint16_t pga256 = map(pga, 0, 100, 0, 255);

    Wire.beginTransmission(0x2B);
    int error = Wire.endTransmission();
    if (!error)
    {
      Wire.beginTransmission(0x2B);
      Wire.write(0x11);   // write to RDAC2 (command 1: C0=1; A3,A2,A1 = 0; A0 = 1)
      Wire.write(pga256); // pga in 8 bit
      Wire.write(0x13);   // write to RDAC4 (command 1: C0=1; A3,A2 = 0; A1,A0 = 1)
      Wire.write(pga256); // pga in 8 bit
      Wire.endTransmission();
    }
  }
   
  //gainOffset = 127;

  if (oldgainOffset != gainOffset)
  { // update gain offset: AD5144A RDAC1+RDAC3

    Wire.beginTransmission(0x2B);
    int error = Wire.endTransmission();
    if (!error)
    {
      Wire.beginTransmission(0x2B);
      Wire.write(0x10);       // write to RDAC1
      Wire.write(gainOffset); // pga in 8 bit
      Wire.write(0x12);       // write to RDAC3
      Wire.write(gainOffset); // pga in 8 bit
      Wire.endTransmission();
      oldgainOffset = gainOffset;
    }
  }
}

void checkTEST()
{
  if (!digitalRead(TEST_IN))
    extTest = true;
  else
    extTest = false;

  if (extTest || intTest)
  {
    digitalWrite(IR_LED, HIGH);
    if (blinkMenu)
      digitalWriteFast(LED_ALARM, HIGH);
    else
      digitalWriteFast(LED_ALARM, LOW);
  }
  else
    digitalWrite(IR_LED, LOW);
}

float getI2Ctemperature() // from ADT75
{
  unsigned int data[2] = {0};

  // temperature from ADT75 (i2c address 0x48)
  // // check if chip is responding
  Wire.beginTransmission(0x48);
  int error = Wire.endTransmission();
  if (!error) // ADT75 is responding
  {

    // Start I2C transmission
    Wire.beginTransmission(0x48);
    // Select data register
    Wire.write(0x00);
    // Stop I2C transmission
    Wire.endTransmission();

    // Request 2 byte of data (for precise 12 bit temperature)
    Wire.requestFrom(0x48, 2);

    // Read 2 bytes of data
    // temp msb, temp lsb
    if (Wire.available() == 2)
    {
      data[0] = Wire.read();
      data[1] = Wire.read();
    }

    // Convert the data to 12 bits
    int temp = ((data[0] * 256) + data[1]) / 16;
    if (temp > 2047)
    {
      temp -= 4096;
    }

    // need only MSB 8 bit value = temperature in degC
    //   Wire.requestFrom(0x48, 1);

    //   if (Wire.available() == 1)
    //   {
    //     data[0] = Wire.read();
    //   }

    // int cTemp = data[0];

    float cTemp = temp * 0.0625;

    return cTemp; // no conversion needed
  }
  else
    return 1; // error
}

void checkALARM()
{
  // check internal temperature Teensy 3.1
  // temp = adc->adc1->readSingle();
  // celsius = (181964504 - 69971 * temp12) >> 12 ; //Vref 1.2
  // celsius = 25.0 + 0.46977 * (892.43 - temp); // Vref 3.3
  if (!refreshMenuTimeout)
  {

    //celsius = (uint8_t)tempmonGetTemp();  // internal CPU temp Teensy 4.0
    celsius = (uint8_t)getI2Ctemperature();  // temperature from ADT75

    if (celsius > max_temperature)
    {
      max_temperature = celsius & 0xFF;
      eeprom_writeInt(EE_ADDR_max_temperature, (uint16_t)max_temperature);
    }
  }

  // check runtime
  if (!hourTimeout)
  { // every hour
    hourTimeout = 3600000;
    total_runtime++;
    if ((total_runtime % 4) == 1)
    {                                         // every 4 hours
      total_runtime = total_runtime & 0xFFFF; // prevent overload
      eeprom_writeInt(EE_ADDR_total_runtime, (uint16_t)total_runtime);
    }
  }

  // check alarms
  if ((motorTimeDiff > (6000 + 50)) || (motorTimeDiff < (6000 - 50)))
  { // motor alarm if not 6000us per rot.
    digitalWriteFast(LED_ALARM, HIGH);
    digitalWriteFast(OUT_ALARM_NEG, LOW); // negative output 0V=ALARM
    if (!alarmChecked)
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 0;
    }
  }                                                                                                       // set ALARM
  else if ((celsius > 70) || ((celsius > 65) && (currentMenu == MENU_ALARM) && (currentMenuOption == 1))) // 5 deg hysteresis internal temperature alarm
  {                                                                                                       // temp alarm
    digitalWriteFast(LED_ALARM, HIGH);
    digitalWriteFast(OUT_ALARM_NEG, LOW); // negative output 0V=ALARM
    if (!alarmChecked)
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 1;
    }
  }
  else if (extTest)
  {
    digitalWriteFast(OUT_ALARM_NEG, HIGH); // NO ALARM => negative output: 24V=OK , but keep LED_ALARM blinking
    if (!alarmChecked)
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 2;
    }
  }
  else if (intTest)
  {
    digitalWriteFast(OUT_ALARM_NEG, HIGH); // NO ALARM => negative output: 24V=OK, but keep LED_ALARM blinking
    if ((!alarmChecked) && !((currentMenu == MENU_SETUP) && (currentMenuOption == 5)))
    {
      currentMenu = MENU_ALARM;
      currentMenuOption = 3;
    }
  }
  else
  {                                        // no alarm, no warnings
    digitalWriteFast(LED_ALARM, LOW);      // no ALARM
    digitalWriteFast(OUT_ALARM_NEG, HIGH); // negative output: 24V=OK
    alarmChecked = false;
    if (currentMenu == MENU_ALARM)
    {
      currentMenu = MENU_MAIN;
      currentMenuOption = 0;
    }
  }
}

//****************************************************************

// SPI send 2 x 16 bit value
void updateSPI(int valueAN1, int valueAN2)
{
  // gain control of the SPI port
  // and configure settings
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); // max 3.3MBPS, CPOL=0, CPHA=0
  digitalWrite(SPI_CS, LOW);                                       // take LATCH pin LOW
  SPI.transfer16(valueAN2);
  SPI.transfer16(valueAN1);
  digitalWrite(SPI_CS, HIGH); // update DAC registers on AD420
  // release control of the SPI port
  SPI.endTransaction();
}

// INTERRUPT ROUTINES

// button interrupts
//*****************************************************************
void checkButtonA()
{
  if (digitalReadFast(PIN_BTN_A))
  {
    if (BtnPressedATimeout || resultButtonA == STATE_LONG)
      BtnReleasedA = true;
  }
  else
  {
    BtnReleasedA = false;
    BtnPressedATimeout = BTN_HOLD_TIME;
  }
}

//*****************************************************************
void checkButtonB()
{
  if (digitalReadFast(PIN_BTN_B))
  {
    if (BtnPressedBTimeout || resultButtonB == STATE_LONG)
      BtnReleasedB = true;
  }
  else
  {
    BtnReleasedB = false;
    BtnPressedBTimeout = BTN_HOLD_TIME;
  }
}

//*****************************************************************
void checkButtonC()
{
  if (digitalReadFast(PIN_BTN_C))
  {
    if (BtnPressedCTimeout || resultButtonC == STATE_LONG)
      BtnReleasedC = true;
  }
  else
  {
    BtnReleasedC = false;
    BtnPressedCTimeout = BTN_HOLD_TIME;
  }
}

//*****************************************************************
void checkButtonD()
{
  if (digitalReadFast(PIN_BTN_D))
  {
    if (BtnPressedDTimeout || resultButtonD == STATE_LONG)
      BtnReleasedD = true;
  }
  else
  {
    BtnReleasedD = false;
    BtnPressedDTimeout = BTN_HOLD_TIME;
  }
}

//*****************************************************************
// Timer interrupts
void timer500us_isr(void)
{ // every 500us
  // motor pulse
  digitalToggleFast(MOTOR_CLK);
  // digitalWriteFast(MOTOR_CLK, !digitalReadFast(MOTOR_CLK));

  // update timeouts

  if (refreshMenuTimeout)
  {
    refreshMenuTimeout--;
  }

  if (laserTimeout)
  {
    laserTimeout--;
  }
  else
  {
    digitalWriteFast(LASER, LOW);
  }

  if (testTimeout)
  {
    testTimeout--;
  }
  else
  {
    intTest = false;
  }

  if (menuTimeout)
  {
    menuTimeout--;
    if (!menuTimeout)
    {
      currentMenu = MENU_MAIN;
      currentMenuOption = 0;
      alarmChecked = false;
      loggedIn = false;
    }
  }

  // buttons timeouts
  if (BtnPressedATimeout)
  {
    BtnPressedATimeout--;
    if (!BtnReleasedA && (BtnPressedATimeout < BTN_DEBOUNCE_TIME))
    {
      resultButtonA = STATE_LONG;
      BtnPressedATimeout = 0;
    }
    if (BtnReleasedA && (BtnPressedATimeout < (BTN_HOLD_TIME - BTN_DEBOUNCE_TIME)))
    {
      resultButtonA = STATE_SHORT;
      BtnPressedATimeout = 0;
    }
  }

  if (BtnPressedBTimeout)
  {
    BtnPressedBTimeout--;
    if (!BtnReleasedB && (BtnPressedBTimeout < BTN_DEBOUNCE_TIME))
    {
      resultButtonB = STATE_LONG;
      BtnPressedBTimeout = 0;
    }
    if (BtnReleasedB && (BtnPressedBTimeout < (BTN_HOLD_TIME - BTN_DEBOUNCE_TIME)))
    {
      resultButtonB = STATE_SHORT;
      BtnPressedBTimeout = 0;
    }
  }

  if (BtnPressedCTimeout)
  {
    BtnPressedCTimeout--;
    if (!BtnReleasedC && (BtnPressedCTimeout < BTN_DEBOUNCE_TIME))
    {
      resultButtonC = STATE_LONG;
      BtnPressedCTimeout = 0;
    }
    if (BtnReleasedC && (BtnPressedCTimeout < (BTN_HOLD_TIME - BTN_DEBOUNCE_TIME)))
    {
      resultButtonC = STATE_SHORT;
      BtnPressedCTimeout = 0;
    }
  }

  if (BtnPressedDTimeout)
  {
    BtnPressedDTimeout--;
    if (!BtnReleasedD && (BtnPressedDTimeout < BTN_DEBOUNCE_TIME))
    {
      resultButtonD = STATE_LONG;
      BtnPressedDTimeout = 0;
    }
    if (BtnReleasedD && (BtnPressedDTimeout < (BTN_HOLD_TIME - BTN_DEBOUNCE_TIME)))
    {
      resultButtonD = STATE_SHORT;
      BtnPressedDTimeout = 0;
    }
  }

  if (digitalReadFast(MOTOR_CLK))
  {
    hourTimeout--;        // every 1ms
    pulsetime = micros(); // for position compensation
  }
#if defined(__IMXRT1062__) // Teensy 4.0
  asm("DSB");
#endif
}

// motor (from HALL sensor) interrupt
void motor_isr(void)
{
  motorPulseIndex++;

  if (motorPulseIndex > 5)
  { // one time per turn
    motorTimeOld = motorTimeNow;
    motorTimeNow = micros();
    motorTimeDiff = motorTimeNow - motorTimeOld; // time of one rotation = 6000us
    motorPulseIndex = 0;
  } // one time per turn

  if ((motorTimeDiff < (6000 + 50)) && (motorTimeDiff > (6000 - 50))) // motor is at full speed 6000us per rot, no motor alarm.
  {
    holdingRegs[EXEC_TIME_TRIGGER] = micros() - pulsetime;
    delayOffset = (positionOffset % 1000 - holdingRegs[EXEC_TIME_TRIGGER]) % 1000; // compensation for HALL magnets position
    if (delayOffset < 0)                                                           // rotate delayOffset between 0 - 1000
      delayOffset = 1000 + delayOffset;

    timerDelay.begin(callback_delay_isr, delayOffset);
  }
#if defined(__IMXRT1062__) // Teensy 4.0
  asm("DSB");
#endif
}

void callback_delay_isr()
{
  timerDelay.end();
  if (!adc0_busy) // previous ADC conversion ended
  {
    exectime = micros();

    // warning! SPI makes noise, do not send data through SPI when ADC is running
    // updating SPI values from previous ADC sequence now, before new ADC sequence starts

    switch (analogOutMode)
    { // an1/an2: "1Int 2Pos" = 0x0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
    case 0x0501:
      updateSPI(peakValueOut, positionValue); // range is 2x 16bit
      break;
    case 0x0105:
      updateSPI(positionValue, peakValueOut);
      break;
    case 0x0505:
      updateSPI(peakValueOut, peakValueOut);
      break;
    case 0x0101:
      updateSPI(positionValue, positionValue);
      break;
    default:
      updateSPI(peakValueOut, positionValue); // range is 2x 16bit
      break;
    }

    // Teensy 4.0 not working without clearing cache
    if ((uint32_t)adc0_buf >= 0x20200000u)
      arm_dcache_flush_delete((void *)adc0_buf, sizeof(adc0_buf));

    adc0_dma.enable();
    adc0_busy = true;
    updateResults(); // update outputs from adc_data[] during next ADC conversion
    holdingRegs[EXEC_TIME] = micros() - exectime;
  }
  else
  {
    holdingRegs[EXEC_TIME] = motorPulseIndex; // just for debugging
    // adc0_dma_isr();
  }
#if defined(__IMXRT1062__) // Teensy 4.0
  asm("DSB");
#endif
}

void adc0_dma_isr(void)
{
  adc0_dma.clearInterrupt();

  for (int i = 0; i < ANALOG_BUFFER_SIZE; i++) // copy DMA buffer
  {
    adc_data[i] = (4095 - adc0_buf[i]) >> 4;
    
    // if ((adc0_buf[i] < 2048) && (adc0_buf[i] >= 0))     
    //    adc_data[i] = (2047 - adc0_buf[i]) >> 3;
    // else adc_data[i] = 0;

    // if (adc0_buf[i] < 2048)
    //   adc_data[i] = 0;
    // else
    //   adc_data[i] = (adc0_buf[i] - 2048) >> 3; // Teensy 4.0 has no analog PGA , 12bit to 8bit positive wave only
    //  
    // adc_data[i] = adc0_buf[i] >> 4;  // full wave Teensy 4.0

    // #if defined(SERIAL_DEBUG)
    //     Serial.printf("%3u: %5u %5u\n", i, adc0_buf[i], adc_data[i]);
    // #endif
  }

  // updateResults();
  // adc0_dma.clearInterrupt();
  adc0_busy = false;
  holdingRegs[EXEC_TIME_ADC] = micros() - exectime; // exectime of adc conversions

#if defined(__IMXRT1062__) // Teensy 4.0
  asm("DSB");
#endif
}

void updateResults()
{
  int hmdThreshold = 0;
  int winBegin = 0;
  int winEnd = 0;
  // int peakValue = 0
  int peak[ANALOG_BUFFER_SIZE] = {0};
  long risingEdgeTime = 0;
  long fallingEdgeTime = 0;
  long peakValueTime = 0;
  // int peakValue = 0;
  int positionValueAvg = 0;

  peakValue = 0;
  peakValueOut = 0;
  positionValue = 0;

  // Serial.println("update results");

  for (int i = 0; i < ANALOG_BUFFER_SIZE; i++)
  {
    // calculate thresholds (with hysteresis) first for speed up calculation
    if (!digitalReadFast(FILTER_PIN))
    {
      hmdThreshold = thre256 + hmdThresholdHyst;
      winBegin = windowBegin * 5;
      winEnd = windowEnd * 5;
    }
    else
    {
      hmdThreshold = thre256 - hmdThresholdHyst;
      winBegin = windowBegin * 5 - 10;
      winEnd = windowEnd * 5 + 10;
    }

    if (i == winBegin)
      peak[i] = adc_data[i]; // check first peak

    if ((i > winBegin) && (i < winEnd)) // if value is inside the measuring window
    {
      // check peak
      if (adc_data[i] > peakValue)
      {
        peakValue = adc_data[i];
#if defined(SERIAL_DEBUG)
        Serial.printf("%3u: %5u %5u\n", i, adc0_buf[i], adc_data[i]);
#endif
      }
      peak[i] = peakValue;

      // check threshold crossing
      if (peakValue > hmdThreshold) // check threshold crossing with hysteresis
      {
        // HMD mode
        if ((positionMode == 4) && !peakValueTime)
        {
          peakValueTime = i * 2;
          digitalWriteFast(FILTER_PIN, HIGH); // update internal pin for bounce2 filter
        }

        // RISING EDGE mode
        if ((positionMode == 1) && (!risingEdgeTime)) // only first occurence
        {
          if (peak[i - 1] <= hmdThreshold)
          {
            risingEdgeTime = i * 2;
            digitalWriteFast(FILTER_PIN, HIGH); // update internal pin for bounce2 filter
          }
        }

        // check for falling edge
        if (positionMode == 2) // only the first occurence
        {
          if ((adc_data[i] < thre256 - hmdThresholdHyst) && (!fallingEdgeTime)) // added additional hysteresis to avoid flickering
          {
            fallingEdgeTime = i * 2;
            digitalWriteFast(FILTER_PIN, HIGH); // update internal pin for bounce2 filter
          }
        }

        // check for peak (but signal can be unstable)
        if (positionMode == 3)
        {
          if (peak[i - 1] + 5 < peakValue) // check for peak
          {
            peakValueTime = i * 2;
            digitalWriteFast(FILTER_PIN, HIGH); // update internal pin for bounce2 filter
          }
        }
      }
    }
  }

  // check SIGNAL PRESENT
  if ((peakValue < thre256 - hmdThresholdHyst) || (!peakValueTime && !risingEdgeTime && !fallingEdgeTime))
  {
    digitalWriteFast(FILTER_PIN, LOW);
  }

  if (extTest || intTest)
  {
    digitalWriteFast(FILTER_PIN, HIGH); // update internal pin for bounce2 filter
  }

  // update SIGNAL PRESENT bounce2 filter
  filterOnOff.update();

  if (filterOnOff.rose())
  {
    filterOnOff.interval(filterOff); // update filter interval
    digitalWriteFast(LED_SIGNAL, HIGH);
    digitalWriteFast(OUT_SIGNAL_NEG, LOW);
  }

  if (filterOnOff.fell())
  {
    filterOnOff.interval(filterOn); // update filter interval
    digitalWriteFast(LED_SIGNAL, LOW);
    digitalWriteFast(OUT_SIGNAL_NEG, HIGH);
  }

  if (digitalReadFast(LED_SIGNAL)) // update position only when SIGNAL PRESENT
  {
    switch (positionMode)
    { // for display
    case 1:
      positionValueDisp = risingEdgeTime;
      break;
    case 2:
      positionValueDisp = fallingEdgeTime;
      break;
    case 3:
    case 4:
      positionValueDisp = peakValueTime;
      break;
    default:
      break;
    }
  }
  else
    positionValueDisp = 0;

  positionValueAvg = approxSimpleMovingAverage(positionValueDisp, filterPosition);

  // remap and send to SPI
  positionValue = constrain(positionValueAvg, windowBegin * 10, windowEnd * 10); // only within measuring window

  positionValueAvgDisp = map(positionValue, windowBegin * 10, windowEnd * 10, 0, 1000); // for display range 0 - 1000

  positionValue = map(positionValue, windowBegin * 10, windowEnd * 10, 0, 65535); // remap for DAC range

  peakValueDisp = map(peakValue, 0, 255, 0, 100); // for display 0 - 100%

  peakValueOut = map(peakValue, 0, 255, 0, 65535); // remap for DAC range

  if (extTest || intTest) // check test mode and set outputs to 50% and 12mA
  {
    positionValueDisp = 500;    // 50% of display range 0 - 1000
    positionValueAvgDisp = 500; // 50% of display range 0 - 1000
    positionValue = 0x7FFF;     // 12mA on position analog output
    peakValueDisp = 75;         // 75% of display range 0 - 100%
    peakValueOut = 0xBFFF;      // 16mA on intensity analog output
  }

  // warning! SPI makes noise, do not send data through SPI when ADC is running

  // switch (analogOutMode)
  // { // an1/an2: "1Int 2Pos" = 0x0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
  // case 0x0501:
  //   updateSPI(peakValueOut, positionValue); // range is 2x 16bit
  //   break;
  // case 0x0105:
  //   updateSPI(positionValue, peakValueOut);
  //   break;
  // case 0x0505:
  //   updateSPI(peakValueOut, peakValueOut);
  //   break;
  // case 0x0101:
  //   updateSPI(positionValue, positionValue);
  //   break;
  // default:
  //   updateSPI(peakValueOut, positionValue); // range is 2x 16bit
  //   break;
  //}

  // if (dataSent && motorPulseIndex == 0) // prepare data for visualization on PC, only first mirror
  if (dataSent && motorPulseIndex == (filterPosition % 6)) // possibility to view different mirrors by changing positionFilter
  {
    for (int i = 0; i < (TOTAL_ERRORS - AN_VALUES); i++) // TOTAL_ERRORS = AN_VALUES + 50
    {
      value_buffer[i] = (adc_data[(i * 10) + 5] % 256 << 8) | (adc_data[i * 10] % 256); // MSB = value_buffer[i*8+4] , LSB = value_buffer[i*8] ; only 50 of 200
      // value_buffer[i] = (adc_data[(i * 2) + 1] % 256 << 8) | (adc_data[i * 2] % 256);
      //  value_buffer[i] = adc_data[i*8];
    }

    dataSent = false;
  }
}

// exponential moving average
long approxSimpleMovingAverage(int new_value, int period)
{

  if (filterPosition)
  {                                   // avoid div/0
    if (!digitalReadFast(LED_SIGNAL)) // clear values
    {
      sma = 0;
    }
    else
    {
      sma *= (period - 1);
      sma += new_value;
      sma /= period;
    }
    return sma;
  }
  else
    return new_value;
}

void checkSTATUS()
{
  bitWrite(io_state, IO_LASER, digitalRead(LASER));
  bitWrite(io_state, IO_IR_LED, digitalRead(IR_LED));
  bitWrite(io_state, IO_TEST_IN, !digitalRead(TEST_IN));
  bitWrite(io_state, IO_SET_IN, !digitalRead(SET_IN));
  bitWrite(io_state, IO_ALARM_OUT, !digitalRead(OUT_ALARM_NEG));
  bitWrite(io_state, IO_SIGNAL_OUT, digitalRead(LED_SIGNAL));
  bitWrite(io_state, IO_LED_ALARM, digitalRead(LED_ALARM));
  bitWrite(io_state, IO_LED_SIGNAL, digitalRead(LED_SIGNAL));
  bitWrite(io_state, IO_LED_POWER, digitalRead(LED_POWER));
  bitWrite(io_state, IO_BTN_A, !digitalRead(PIN_BTN_A));
  bitWrite(io_state, IO_BTN_B, !digitalRead(PIN_BTN_B));
  bitWrite(io_state, IO_BTN_C, !digitalRead(PIN_BTN_C));
  bitWrite(io_state, IO_BTN_D, !digitalRead(PIN_BTN_D));
}

void checkModbus()
{

  holdingRegs[MODBUS_ID] = modbusID;
  holdingRegs[MODBUS_SPEED] = modbusSpeed / 100; // baud rate/100 to fit into word
  holdingRegs[MODBUS_FORMAT] = modbusFormat;

  holdingRegs[SET] = set;
  holdingRegs[GAIN_SET1] = pga1 * 100;
  holdingRegs[THRESHOLD_SET1] = thre1 * 100;
  holdingRegs[GAIN_SET2] = pga2 * 100;
  holdingRegs[THRESHOLD_SET2] = thre2 * 100;

  holdingRegs[WINDOW_BEGIN] = windowBegin * 100;
  holdingRegs[WINDOW_END] = windowEnd * 100;
  holdingRegs[POSITION_MODE] = positionMode;
  holdingRegs[ANALOG_OUT_MODE] = analogOutMode;
  holdingRegs[POSITION_OFFSET] = positionOffset;

  holdingRegs[FILTER_POSITION] = filterPosition;
  holdingRegs[FILTER_ON] = filterOn;
  holdingRegs[FILTER_OFF] = filterOff;

  holdingRegs[ACT_TEMPERATURE] = celsius << 8;
  holdingRegs[MAX_TEMPERATURE] = max_temperature << 8;
  holdingRegs[TOTAL_RUNTIME] = total_runtime;
  holdingRegs[IO_STATE] = io_state;

  holdingRegs[MOTOR_TIME_DIFF] = motorTimeDiff;
  holdingRegs[OFFSET_DELAY] = delayOffset;

  // updated in updateResults()
  holdingRegs[PEAK_VALUE] = peakValueDisp * 100;               // 0-10000  0-100% * 100
  holdingRegs[POSITION_VALUE] = positionValueDisp * 10;        // 0-10000
  holdingRegs[POSITION_VALUE_AVG] = positionValueAvgDisp * 10; // 0-10000

  if (!dataSent)
  {
    for (byte i = 0; i < (TOTAL_ERRORS - AN_VALUES); i++) // MOTOR_TIME_DIFF = AN_VALUES + 25
    {
      holdingRegs[i + AN_VALUES] = value_buffer[i]; // values stored properly in updateResults() to save memory
    }

    dataSent = true;
  }

  // arm_dcache_flush_delete((void *)holdingRegs, sizeof(holdingRegs));
  holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);

  // check changes made via ModBus - if values are valid, save them in EEPROM

  if ((holdingRegs[MODBUS_ID] != modbusID) || (holdingRegs[MODBUS_SPEED] * 100 != modbusSpeed) || (holdingRegs[MODBUS_FORMAT] != modbusFormat))
  {
    if ((holdingRegs[MODBUS_ID] != modbusID) && (holdingRegs[MODBUS_ID] > 0) && (holdingRegs[MODBUS_ID] < 248))
    {
      modbusID = holdingRegs[MODBUS_ID];
      eeprom_writeInt(EE_ADDR_modbus_ID, holdingRegs[MODBUS_ID]);
    }

    if ((holdingRegs[MODBUS_SPEED] * 100) != modbusSpeed)
    {
      switch (holdingRegs[MODBUS_SPEED])
      {
      case 3:
      case 6:
      case 12:
      case 24:
      case 48:
      case 96:
      case 144:
      case 192:
      case 288:
      case 384:
      case 576:
      case 1152:
        modbusSpeed = holdingRegs[MODBUS_SPEED] * 100;
        eeprom_writeInt(EE_ADDR_modbus_Speed, holdingRegs[MODBUS_SPEED]);
        break;
      default:
        break;
      }
    }

    if (holdingRegs[MODBUS_FORMAT] != modbusFormat)
    {
      switch (holdingRegs[MODBUS_FORMAT])
      {
      case SERIAL_8N1:
      case SERIAL_8E1:
      case SERIAL_8O1:
      case SERIAL_8N2:
        modbusFormat = holdingRegs[MODBUS_FORMAT];
        eeprom_writeInt(EE_ADDR_modbus_Format, holdingRegs[MODBUS_FORMAT]);
        break;
      default:
        break;
      }
    }

    // restart communication
    Serial1.flush();
    Serial1.end();
    // modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, TOTAL_REGS_SIZE, 0);
    modbus_configure(modbusSpeed, modbusFormat, modbusID, TXEN, 1080, 0); // for compatibility with old SDIS
  }

  if ((holdingRegs[SET] != set) && (holdingRegs[SET] > 0) && (holdingRegs[SET] < 4))
  { // RELAY = 3 (REL1 || REL2), MAN1 = 1, MAN2 = 2
    set = holdingRegs[SET];
    eeprom_writeInt(EE_ADDR_set, holdingRegs[SET]);
  }

  if ((holdingRegs[GAIN_SET1] != (pga1 * 100)) && (holdingRegs[GAIN_SET1] >= 100) && (holdingRegs[GAIN_SET1] <= 10000))
  {
    pga1 = holdingRegs[GAIN_SET1] / 100;
    eeprom_writeInt(EE_ADDR_gain_set1, pga1);
  }

  if ((holdingRegs[THRESHOLD_SET1] != (thre1 * 100)) && (holdingRegs[THRESHOLD_SET1] >= 2000) && (holdingRegs[THRESHOLD_SET1] <= 8000))
  {
    thre1 = holdingRegs[THRESHOLD_SET1] / 100;
    eeprom_writeInt(EE_ADDR_threshold_set1, thre1);
  }

  if ((holdingRegs[GAIN_SET2] != (pga2 * 100)) && (holdingRegs[GAIN_SET2] >= 100) && (holdingRegs[GAIN_SET2] <= 10000))
  {
    pga2 = holdingRegs[GAIN_SET2] / 100;
    eeprom_writeInt(EE_ADDR_gain_set2, pga2);
  }

  if ((holdingRegs[THRESHOLD_SET2] != (thre2 * 100)) && (holdingRegs[THRESHOLD_SET2] >= 2000) && (holdingRegs[THRESHOLD_SET2] <= 8000))
  {
    thre2 = holdingRegs[THRESHOLD_SET2] / 100;
    eeprom_writeInt(EE_ADDR_threshold_set2, thre2);
  }

  if ((holdingRegs[WINDOW_BEGIN] != (windowBegin * 100)) && (holdingRegs[WINDOW_BEGIN] >= 500) && (holdingRegs[WINDOW_BEGIN] <= 4500))
  {
    windowBegin = holdingRegs[WINDOW_BEGIN] / 100;
    eeprom_writeInt(EE_ADDR_window_begin, windowBegin);
  }

  if (holdingRegs[WINDOW_END] != (windowEnd * 100) && (holdingRegs[WINDOW_END] >= 5500) && (holdingRegs[WINDOW_END] <= 9500))
  {
    windowEnd = holdingRegs[WINDOW_END] / 100;
    eeprom_writeInt(EE_ADDR_window_end, windowEnd);
  }

  if ((holdingRegs[POSITION_MODE] != positionMode) && (holdingRegs[POSITION_MODE] > 0) && (holdingRegs[POSITION_MODE] < 5))
  {
    positionMode = holdingRegs[POSITION_MODE];
    eeprom_writeInt(EE_ADDR_position_mode, holdingRegs[POSITION_MODE]);
  }

  if (holdingRegs[ANALOG_OUT_MODE] != analogOutMode)
  {
    switch (holdingRegs[ANALOG_OUT_MODE]) // save if valid mode an1/an2: "1Int 2Pos" = 0x0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
    {
    case 0x0501:
    case 0x0105:
    case 0x0505:
    case 0x0101:
      analogOutMode = holdingRegs[ANALOG_OUT_MODE];
      eeprom_writeInt(EE_ADDR_analog_out_mode, holdingRegs[ANALOG_OUT_MODE]);
      break;
    default:
      break;
    }
  }

  if ((holdingRegs[POSITION_OFFSET] != positionOffset) && (holdingRegs[POSITION_OFFSET] >= 0) && (holdingRegs[POSITION_OFFSET] <= 2000))
  {
    positionOffset = holdingRegs[POSITION_OFFSET];
    eeprom_writeInt(EE_ADDR_position_offset, positionOffset);
    // SCB_AIRCR = 0x05FA0004;        // software reset

    if (positionOffset < 1000) // depends on motor HALL sensors & mirror position - choose the best to have no timing issues
      attachInterrupt(digitalPinToInterrupt(MOTOR_ALARM), motor_isr, FALLING);
    else
      attachInterrupt(digitalPinToInterrupt(MOTOR_ALARM), motor_isr, RISING);
  }

  if ((holdingRegs[FILTER_POSITION] != filterPosition) && (holdingRegs[FILTER_POSITION] < 10000))
  {
    filterPosition = holdingRegs[FILTER_POSITION];
    eeprom_writeInt(EE_ADDR_filter_position, filterPosition);
  }

  if ((holdingRegs[FILTER_ON] != filterOn) && (holdingRegs[FILTER_ON] < 10000))
  {
    filterOn = holdingRegs[FILTER_ON];
    eeprom_writeInt(EE_ADDR_filter_on, filterOn);
  }

  if ((holdingRegs[FILTER_OFF] != filterOff) && (holdingRegs[FILTER_OFF] < 10000))
  {
    filterOff = holdingRegs[FILTER_OFF];
    eeprom_writeInt(EE_ADDR_filter_off, filterOff);
  }

  if ((holdingRegs[IO_STATE] & (1 << IO_LASER)) != (io_state & (1 << IO_LASER)))
  {
    if (holdingRegs[IO_STATE] & (1 << IO_LASER))
    { // check if IO_LASER bit is set
      laserTimeout = TIMEOUT_LASER;
      digitalWrite(LASER, HIGH);
    }
    else
    {
      digitalWrite(LASER, LOW);
      laserTimeout = 0;
    }
  }

  if ((holdingRegs[IO_STATE] & (1 << IO_IR_LED)) != (io_state & (1 << IO_IR_LED)))
  {
    if (holdingRegs[IO_STATE] & (1 << IO_IR_LED))
    { // check if IO_IR_LED bit is set
      //digitalWrite(IR_LED, HIGH);
      testTimeout = TIMEOUT_TEST;
      intTest = true;
    }
    else
    {
      //digitalWrite(IR_LED, LOW);
      intTest = false;
    }
  }

  if ((holdingRegs[IO_STATE] & (1 << IO_SW_RESET)) != (io_state & (1 << IO_SW_RESET)))
  {
    if (holdingRegs[IO_STATE] & (1 << IO_SW_RESET))
    {
      SCB_AIRCR = 0x05FA0004; // software reset on all Cortex M processors
    }
  }

}

// check void ADC_Module::startPDB() in ADC_Module.cpp for //NVIC_ENABLE_IRQ(IRQ_PDB);

// pdb interrupt is enabled in case you need it.
// void pdb_isr(void) {
//         PDB0_SC &=~PDB_SC_PDBIF; // clear interrupt
//        // NVIC_DISABLE_IRQ(IRQ_PDB); // we don't want or need the PDB interrupt
//         //digitalWriteFast(LED_BUILTIN,!digitalReadFast(LED_BUILTIN));
//         Serial.println("PDB interrupt");
// }

// void InitVariables()
// {
//   modbusID = 1;

//   actualSpeed = 3;     // array index
//   modbusSpeed = 19200; // default 19200

//   actualFormat = 1;
//   modbusFormat = SERIAL_8N1;

//   max_temperature = 0;

//   dataSent = false;
//   sendNextLn = 0;
//   io_state = 0;
//   exectime = 0;
//   pulsetime = 0;

//   total_runtime = 0;
// }