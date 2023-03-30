//Variables and definitions used in the project
#ifndef __Variables_H
#define __Variables_H

#define SERIAL_DEBUG

//defaults EEPROM
#define MODEL_TYPE 50
#define MODEL_SERIAL_NUMBER 23001
#define FW_VERSION 234           

#define DEFAULT_MODBUS_ID MODEL_SERIAL_NUMBER % 1000 % 247 // MODBUS ID slave (range 1..247)
#define DEFAULT_MODBUS_SPEED 19200
#define DEFAULT_MODBUS_FORMAT SERIAL_8N1

#define DEFAULT_SET 3             // RELAY = 3 (REL1 || REL2), MAN1 = 1, MAN2 = 2
#define DEFAULT_GAIN_SET1 16      // valid values 1,2,4,8,16,32,64
#define DEFAULT_THRESHOLD_SET1 50 // min 20, max 80
#define DEFAULT_GAIN_SET2 32
#define DEFAULT_THRESHOLD_SET2 50

#if MODEL_TYPE == 10
#define DEFAULT_WINDOW_BEGIN 45 // min 5, max 45
#define DEFAULT_WINDOW_END 55   // min 55 max 95
#elif MODEL_TYPE == 30
#define DEFAULT_WINDOW_BEGIN 35 // min 5, max 45
#define DEFAULT_WINDOW_END 65   // min 55 max 95
#else // MODEL_TYPE == 50
#define DEFAULT_WINDOW_BEGIN 20 // min 5, max 45
#define DEFAULT_WINDOW_END 80   // min 55 max 95
#endif

#define DEFAULT_POSITION_MODE 4     // rising = 1, falling = 2, peak = 3, hmd = 4
#define DEFAULT_ANALOG_OUT_MODE 0x0501   // an1/an2: "1Int 2Pos" = 0x0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
#define DEFAULT_POSITION_OFFSET 250 // min 5, max 95 to avoid coincidence with pulse interrupts

#define DEFAULT_FILTER_POSITION 6 // range 0 - 9999 ms (or nr of mirrors) for moving average
#define DEFAULT_FILTER_ON 0       // range 0 - 9999 ms
#define DEFAULT_FILTER_OFF 0      // range 0 - 9999 ms

// EEPROM Addresses (all values are WORD for easy Modbus transfers)

// EEPROM Addresses for signature code and version of firmware
#define EE_ADDR_MODEL_TYPE 0          // WORD 
#define EE_ADDR_MODEL_SERIAL_NUMBER 2 // WORD
#define EE_ADDR_FW_VERSION 4          // WORD

// EEPROM Addresses for config
#define EE_ADDR_modbus_ID 6     // WORD
#define EE_ADDR_modbus_Speed 8  // WORD  // baudrate/100 to fit 115200 to WORD
#define EE_ADDR_modbus_Format 10 // WORD

#define EE_ADDR_set 12            // WORD  // RELAY = 3 (REL1 || REL2), MAN1 = 1, MAN2 = 2
#define EE_ADDR_gain_set1 14      // WORD
#define EE_ADDR_threshold_set1 16 // WORD
#define EE_ADDR_gain_set2 18      // WORD
#define EE_ADDR_threshold_set2 20 // WORD

#define EE_ADDR_window_begin 22    // WORD
#define EE_ADDR_window_end 24      // WORD
#define EE_ADDR_position_mode 26   // WORD  // positionMode: RISE = 1, FALL = 2, PEAK = 3, HMD = 4
#define EE_ADDR_analog_out_mode 28 // WORD  // an1/an2: "1Int 2Pos" = 0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
#define EE_ADDR_position_offset 30 // WORD  // offset for position

#define EE_ADDR_filter_position 32 // WORD  // range 0 - 9999 ms
#define EE_ADDR_filter_on 34       // WORD  // range 0 - 9999 ms
#define EE_ADDR_filter_off 36      // WORD  // range 0 - 9999 ms

// EEPROM Addresses for diagnosis
#define EE_ADDR_max_temperature 38 // WORD
#define EE_ADDR_total_runtime 40   // WORD


// TEENSY4.0 pin assignment
#define FILTER_PIN 33 // not connected, for internal use of Bounce2 library filter - SIGNAL PRESENT filter ON/OFF

// assign the Arduino pin that must be connected to RE-DE RS485 transceiver
#define TXEN 2 // Serial1: RX1=0 TX1=1 TXEN=2

// SPI
#define SPI_CS 10 // LATCH on AD420

// Define pins for the LED display.
// You can change these, just re-wire your board:
#define LED_DATA_PIN 3         // 3 connects to the display's data in
#define LED_RS_PIN 4           // 4 the display's register select pin
#define LED_CLK_PIN 5          // 5 the display's clock pin
#define LED_EN_PIN  6          // 6 the display's chip enable pin
#define LED_RST_PIN 7          // 7 the display's reset pin

#define DISPLAY_LENGHT 8 // number of characters in the display

// LEDs and I/O

#define LED_POWER 17
#define LED_SIGNAL 18 // now same as OUT_SIGNAL
#define LED_ALARM 19

#define ANALOG_INPUT A10

#define PIN_BTN_A 20 //(digital pin)
#define PIN_BTN_B 21 //(digital pin)
#define PIN_BTN_C 22 //(digital pin)
#define PIN_BTN_D 23 //(digital pin)

#define TEST_IN 28
#define SET_IN 27

#define LASER 29
#define IR_LED 25
#define OUT_SIGNAL_NEG 8
#define OUT_ALARM_NEG 9 //negative output: 24V=>OK, 0V=>ALARM

#define MOTOR_ALARM 14  //pulses from Hall probe
#define MOTOR_ENABLE 15 //enable motor rotation
#define MOTOR_CLK 16    //motor speed clock

// Timeout definitions
#define TIMEOUT_LASER 1200000 // 10 min
#define TIMEOUT_TEST 600000 // 5 min

extern volatile boolean alarmChecked;
extern volatile boolean extTest;
extern volatile boolean intTest;
extern volatile int currentMenu;
extern volatile int currentMenuOption;


extern volatile int hourTimeout;
extern volatile int refreshMenuTimeout;
extern volatile int laserTimeout;
extern volatile int testTimeout;
extern volatile int menuTimeout;

// button interrupt
#define STATE_NORMAL 0
#define STATE_SHORT 1
#define STATE_LONG 2

extern volatile int resultButtonA; // global value set by checkButton()
extern volatile int resultButtonB; // global value set by checkButton()
extern volatile int resultButtonC; // global value set by checkButton()
extern volatile int resultButtonD; // global value set by checkButton()
extern volatile boolean BtnReleasedA;
extern volatile boolean BtnReleasedB;
extern volatile boolean BtnReleasedC;
extern volatile boolean BtnReleasedD;


//////////////// registers of your slave ///////////////////
enum
{
  // just add or remove registers and your good to go...
  // The first register starts at address 0
  ENUM_SIZE,
  MB_MODEL_TYPE,
  MB_MODEL_SERIAL_NUMBER,
  MB_FW_VERSION,

  MODBUS_ID,     // address 1..247
  MODBUS_SPEED,  // baud rate/100 to fit into word
  MODBUS_FORMAT, // SERIAL_8N1 = 0, SERIAL_8E1 = 6, SERIAL_8O1 = 7 , SERIAL_8N2 = 4

  SET,            // MAN1 = 1, MAN2 = 2, RELAY = 3 (REL1 || REL2),
  GAIN_SET1,      // valid values 1,2,4,8,16,32,64 * 100
  THRESHOLD_SET1, // min 20, max 80 * 100
  GAIN_SET2,      // valid values 1,2,4,8,16,32,64 * 100
  THRESHOLD_SET2, // min 20, max 80 * 100

  WINDOW_BEGIN,    // min 5, max 50 * 100
  WINDOW_END,      // min 50 max 95 * 100
  POSITION_MODE,   // rising = 1, falling = 2, peak = 3 , hmd = 4
  ANALOG_OUT_MODE, // an1/an2: "1Int 2Pos" = 0x0501, "1Pos 2Int" = 0x0105, "1Int 2Int" = 0x0505, "1Pos 2Pos" = 0x0101
  POSITION_OFFSET, // min 0, max 2000 

  FILTER_POSITION, // range 0 - 9999 ms (or nr of mirrors) for moving average
  FILTER_ON,       // range 0 - 9999 ms
  FILTER_OFF,      // range 0 - 9999 ms

  ACT_TEMPERATURE,  // act_temp * 256
  MAX_TEMPERATURE,  // max_temp * 256
  TOTAL_RUNTIME,
  IO_STATE,

  PEAK_VALUE,       // 0-100% * 100
  POSITION_VALUE,   // 0-100% * 100
  POSITION_VALUE_AVG,  // 0-100% * 100

  AN_VALUES, // 25 registers
  MOTOR_TIME_DIFF = AN_VALUES + 25,
  EXEC_TIME_ADC,     // exectime of adc conversions
  EXEC_TIME,         // exectime of adc conversions + results calculation
  EXEC_TIME_TRIGGER, // exectime of each triggering
  OFFSET_DELAY,      // calculated trigger delay
  TOTAL_ERRORS,
  // leave this one
  TOTAL_REGS_SIZE
  // total number of registers for function 3 and 16 share the same register array
};

// I/O Status bits for Modbus
enum
{
  IO_LASER,
  IO_IR_LED,
  IO_TEST_IN,
  IO_SET_IN,
  IO_ALARM_OUT,
  IO_SIGNAL_OUT,
  IO_LED_ALARM,
  IO_LED_SIGNAL,
  IO_LED_POWER,
  IO_BTN_A,
  IO_BTN_B,
  IO_BTN_C,
  IO_BTN_D,
  IO_SW_RESET
};

// MODBUS

extern volatile uint16_t modbusID;

const uint16_t modbusSpeedArray[] = {12, 48, 96, 192, 384, 576, 1152}; // baudrate/100
extern volatile uint16_t actualSpeed;                                                   // array index
extern volatile uint32_t modbusSpeed;            // default 19200

const uint16_t modbusFormatArray[] = {SERIAL_8N1, SERIAL_8E1, SERIAL_8O1, SERIAL_8N2};
extern volatile uint16_t actualFormat;
extern volatile uint16_t modbusFormat;


extern volatile uint16_t io_state;

extern uint16_t holdingRegs[TOTAL_REGS_SIZE]; // function 3 and 16 register array

extern volatile uint16_t windowBegin, windowEnd, positionOffset, positionMode, analogOutMode;
extern volatile uint16_t filterPosition, filterOn, filterOff;

extern volatile uint16_t thre256, thre, thre1, thre2;
extern volatile uint16_t set, pga, pga1, pga2;

// diagnosis
extern volatile uint16_t celsius; // internal temp in deg of Celsius
extern volatile uint16_t temp;    // internal ADC Temp channel value
extern volatile uint16_t max_temperature;
extern volatile uint16_t total_runtime;

extern volatile boolean blinkMenu;
extern int setDispIndex;
extern boolean loggedIn;

extern volatile long peakValueTimeDisp;
extern volatile int peakValueDisp;
extern volatile int positionValueDisp;
extern volatile int positionValueAvgDisp;
extern volatile int peakValue;
extern volatile int positionValue;

extern void checkSET();

#endif /* __Variables_H */