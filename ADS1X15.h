
#pragma once    // For prevent multi decleration file

// ###############################################
// Information:

// FILE: ADS1X15.h
// AUTHOR: Mohammad Nikanjam. ----> Forked from Rob Tillaart. URL: Forked from https://github.com/RobTillaart/ADS1X15
// RobTillaart/ADS1X15 is library for arduino boards.
// This new library changed for raspberry pi adaption.

// ###############################################

//$$$ this section added to orginal source
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <cstdint>
#include <thread>
#include <chrono>
#include <iostream>
#include <bcm2835.h>
#include <pigpio.h> 
#include <map>

using namespace std;

// ADS1115 define macros:
#define ADS1115_GAIN_6V144        0
#define ADS1115_GAIN_4V096        1
#define ADS1115_GAIN_2V048        2
#define ADS1115_GAIN_1V024        4
#define ADS1115_GAIN_0V512        8
#define ADS1115_GAIN_0V256        16

#define ADS1115_RATE_8SPS         0
#define ADS1115_RATE_16SPS        1
#define ADS1115_RATE_32SPS        2
#define ADS1115_RATE_64SPS        3
#define ADS1115_RATE_128SPS       4
#define ADS1115_RATE_250SPS       5
#define ADS1115_RATE_475SPS       6
#define ADS1115_RATE_860SPS       7

#define ADS1115_SINGLE_MODE       1
#define ADS1115_CONTINUOUS_MODE   0  

#define ADS1X15_LIB_VERSION               "0.4.0"

//  allow compile time default address
//  address in { 0x48, 0x49, 0x4A, 0x4B }, no test...
#ifndef ADS1015_ADDRESS
#define ADS1015_ADDRESS                   0x48
#endif

#ifndef ADS1115_ADDRESS
#define ADS1115_ADDRESS_DEF               0x48
#define ADS1115_RAW_RANGE                 32767                   // 16 bit resolution. int16 value from get ADS channels
#endif


#define ADS1X15_OK                        0
#define ADS1X15_INVALID_VOLTAGE           -100
#define ADS1X15_INVALID_GAIN              0xFF
#define ADS1X15_INVALID_MODE              0xFE

// ADS1X15 object
class ADS1X15
{
public:

  std::string errorMessage;     // Store last error message that accured for sensor.
  float refVolt;                // Voltage reference for ADC of ADS1X15.[volt]

  // Reset paramters of sensor to default values.
  void reset();

  // Open i2c slave handler. Start i2c communication by slave.
  // @return true if i2c slave correctly connect and detected.
  bool beginI2c();

  // Check device recognized.
  // @return true if connected.
  bool isConnected();

  //           GAIN
  //  0  =  +- 6.144V  default
  //  1  =  +- 4.096V
  //  2  =  +- 2.048V
  //  4  =  +- 1.024V
  //  8  =  +- 0.512V
  //  16 =  +- 0.256V
  void     setGain(uint8_t gain = 0);    //  invalid values are mapped to 0 (default).
  uint8_t  getGain();                    //  0xFF == invalid gain error.


  //  both may return ADS1X15_INVALID_VOLTAGE if the gain is invalid.
  float    toVoltage(int16_t value = 1); //   converts raw to voltage
  float    getMaxVoltage();              //   -100 == invalid voltage error


  //  0  =  CONTINUOUS
  //  1  =  SINGLE      default
  void     setMode(uint8_t mode = 1);    //  invalid values are mapped to 1 (default)
  uint8_t  getMode();                    //  0xFE == invalid mode error.


  //  0  =  slowest
  //  7  =  fastest
  //  4  =  default
  void     setDataRate(uint8_t dataRate = 4); // invalid values are mapped on 4 (default)
  uint8_t  getDataRate();                     // actual speed depends on device


  int16_t  readADC(uint8_t pin = 0);
  int16_t  readADC_Differential_0_1();

  //  used by continuous mode and async mode.
  [[deprecated("Use getValue() instead")]]
  int16_t  getLastValue() { return getValue(); };  // will be obsolete in the future 0.4.0
  int16_t  getValue();


  //  ASYNC INTERFACE
  //  requestADC(pin) -> isBusy() or isReady() -> getValue();
  //  see examples
  void     requestADC(uint8_t pin = 0);
  void     requestADC_Differential_0_1();
  bool     isBusy();
  bool     isReady();

  //  returns a pin 0x0[0..3] or 
  //          a differential "mode" 0x[pin second][pin first] or
  //          0xFF (no request / invalid request)
  uint8_t   lastRequest();

  //  COMPARATOR
  //  0    = TRADITIONAL   > high          => on      < low   => off
  //  else = WINDOW        > high or < low => on      between => off
  void     setComparatorMode(uint8_t mode);
  uint8_t  getComparatorMode();

  //  0    = LOW (default)
  //  else = HIGH
  void     setComparatorPolarity(uint8_t pol);
  uint8_t  getComparatorPolarity();

  //  0    = NON LATCH
  //  else = LATCH
  void     setComparatorLatch(uint8_t latch);
  uint8_t  getComparatorLatch();

  //  0   = trigger alert after 1 conversion
  //  1   = trigger alert after 2 conversions
  //  2   = trigger alert after 4 conversions
  //  3   = Disable comparator =  default, also for all other values.
  void     setComparatorQueConvert(uint8_t mode);
  uint8_t  getComparatorQueConvert();

  void     setComparatorThresholdLow(int16_t lo);
  int16_t  getComparatorThresholdLow();
  void     setComparatorThresholdHigh(int16_t hi);
  int16_t  getComparatorThresholdHigh();

  int8_t   getError();

  // $$$->
  // this section added to orginal source

  // Set I2C file path for sensor.
  void     setI2cPath(const char* i2c_device_path);     

  // Clean and close all resources.
  void     clean(void);

  // <-$$$

protected:

  // Constructor. reset parameters of sensors to default values.
  ADS1X15();

  //  CONFIGURATION
  //  BIT   DESCRIPTION
  //  0     # channels        0 == 1    1 == 4;
  //  1     0
  //  2     # resolution      0 == 12   1 == 16
  //  3     0
  //  4     has gain          0 = NO    1 = YES
  //  5     has comparator    0 = NO    1 = YES
  //  6     0
  //  7     0
  uint8_t  _config;
  uint8_t  _maxPorts;
  uint8_t  _address;
  uint8_t  _conversionDelay;
  uint8_t  _bitShift;
  uint16_t _gain;
  uint16_t _mode;
  uint16_t _datarate;

  // $$$-> This section added to orginal source

  const char* _i2cPath;       // I2C device path
  int _ADS_handler;           // I2c ADS device handler.

  // <-$$$

  //  COMPARATOR variables
  //  TODO merge these into one COMPARATOR MASK?  (low priority)
  //       would speed up code in _requestADC() and save 3 bytes RAM.
  //  TODO boolean flags for first three, or make it mask value that
  //       can be or-ed.   (low priority)
  uint8_t  _compMode;
  uint8_t  _compPol;
  uint8_t  _compLatch;
  uint8_t  _compQueConvert;

  //  variable to track the last pin requested, 
  //  to allow for round robin query of
  //  pins based on this state == if no last request then == 0xFFFF.
  uint16_t  _lastRequest;

  int16_t  _readADC(uint16_t readmode);
  void     _requestADC(uint16_t readmode);
  
  // $$$

  bool     _writeRegister(uint8_t reg, uint16_t value);
  uint16_t _readRegister(uint8_t reg);
  
  // $$$
  
  int8_t   _err = ADS1X15_OK;

};


///////////////////////////////////////////////////////////////////////////
//
//  DERIVED CLASSES from ADS1X15

class ADS1013 : public ADS1X15
{
public:
  ADS1013(uint8_t Address = ADS1015_ADDRESS, const char* i2c_device_path = "/dev/i2c-1");
};


class ADS1014 : public ADS1X15
{
public:
  ADS1014(uint8_t Address = ADS1015_ADDRESS, const char* i2c_device_path = "/dev/i2c-1");
};


class ADS1015 : public ADS1X15
{
public:
  ADS1015(uint8_t Address = ADS1015_ADDRESS, const char* i2c_device_path = "/dev/i2c-1");
  int16_t  readADC_Differential_0_3();
  int16_t  readADC_Differential_1_3();
  int16_t  readADC_Differential_2_3();
  int16_t  readADC_Differential_0_2();   //  not possible in async
  int16_t  readADC_Differential_1_2();   //  not possible in async
  void     requestADC_Differential_0_3();
  void     requestADC_Differential_1_3();
  void     requestADC_Differential_2_3();
};


class ADS1113 : public ADS1X15
{
public:
  ADS1113(uint8_t address, const char* i2c_device_path = "/dev/i2c-1");
};


class ADS1114 : public ADS1X15
{
public:
  ADS1114(uint8_t address, const char* i2c_device_path = "/dev/i2c-1");
};

// ######################################################################################
// ADS1115 object.
class ADS1115 : public ADS1X15
{
public:

  // 16 bit raw value for 4 channel
  int16_t valueRaw[4];

  // Real value of channels. [volts]
  double value[4];

  // Channel number for read.
  int channel = 0;                         

  // Constructor for ADS1115 object. Set i2c device address. Set I2C port path.
  //@param address: Slave i2c device address.
  //@param i2c_device_path: I2C port path.
  ADS1115(uint8_t address = ADS1115_ADDRESS_DEF, const char* i2c_device_path = "/dev/i2c-1");
  
  // Destructor. Close and clean all sources.
  ~ADS1115();

  // Start ADS actions. setup ALART/RDY pin and hardware interrupt.
  bool begin(void);

  // Set ALART/RDY GPIO pin number for digital input and external interrupts. 
  // Hint: begin() needed after that for start pin operation.
  bool setRdyPin(uint8_t rdyPin);
   
  // Clean and close all resources.
  void clean(void);

  int16_t  readADC_Differential_0_3();
  int16_t  readADC_Differential_1_3();
  int16_t  readADC_Differential_2_3();
  int16_t  readADC_Differential_0_2();   //  not possible in async
  int16_t  readADC_Differential_1_2();   //  not possible in async
  void     requestADC_Differential_0_3();
  void     requestADC_Differential_1_3();
  void     requestADC_Differential_2_3();

  // Simple handle conversion without ready interrupt pin. Read all channel data in blocking mode.
  void handleConversion(void);

  /**
   * @brief Handle conversion. Read data if ready then request new conversion. Conversion in no blocking mode.
   * @return The channel number that read.
   *  */ 
  int8_t handleConversionNoBlocking(void);

  /*
  Config ADS sensor. begin(), setGain(), setDataRate(), set continuous mode and enable Ready/ALART pin interrup pin and function action.
  @param gain: 0:+- 6.144V  default, 1:+- 4.096V, 2:+- 2.048V, 4:+- 1.024V, 8:+- 0.512V, 16:+- 0.256V
  @param dataRate: 0:slowest, 7:fastest, 4:default
  @param rdyPin: ALART/RDY GPIO pin number. if it set to -1, disabled.
  */
  bool config(uint8_t gain, uint8_t dataRate, int rdyPin = -1);

private:

  // Flag for ADS RYD pin external interrupts. In interrupt function handle, it change to true.
  volatile bool _rdyFlag;     

  // Digital output Comparator output or conversion ready pin.
  int _rdyPin;

  bool _rdyPinFlag;

  // Static map to hold instances of the class, indexed by GPIO pin number
  static std::map<int, ADS1115*> instanceMap;

  // Static ISR trampoline function for find the instance associated with the GPIO pin and call the member function
  static void _isrTrampoline(int gpio, int level, uint32_t tick); 

  /*
    Interrupt handler function for ALART/RDY intrrupt.
  */ 
  void _InterruptHandler(int gpio, int level, uint32_t tick); 

};


//  -- END OF FILE --

