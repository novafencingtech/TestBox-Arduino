/* Used for switching between the nrf52832 and nrf52840
 *  
 *  
 */



//Need special handling for nrf52840 due to conflicts between nrf52.h and nrf52840.h
#ifdef _VARIANT_FEATHER52840_  
  #include <nrf52840.h>
#else 
  #include <nrf52.h>
#endif

//ChanArray[NUM_ADC_SCAN_CHANNELS] {0, 3, 3, 4, 1, 4, 5, 5, 2}; //AA, AB, AC, BA, BB, BC, CA, CB, CC
#ifdef _VARIANT_FEATHER52840_  
  const char MCU_ID_STRING[16]="840";
  const uint8_t AINpinA_amp=2; //AIN2
  const uint8_t AINpinB_amp=3; //AIN3
  const uint8_t AINpinC_amp=6; //AIN6
  const uint8_t AINpinA_raw=4; //AIN4
  const uint8_t AINpinB_raw=0; //AIN0
  const uint8_t AINpinC_raw=1; //AIN1
  const byte AINBattMon = 5; //AIN5

  const uint8_t MUX_LATCH = 13;
  const uint8_t MUX_CLK = 12;
  const uint8_t MUX_DATA = 11;

  const byte POWER_CONTROL = 10;
  const byte DIAG_PIN = 22;
  const byte BUTTON_PIN = 5;
  const byte LED1_PIN = 3;
  const byte LED2_PIN = 4;
  //#define LED_DATA_PIN 8 //Onboard NeoPixel is pin 8
  #define LED_DATA_PIN 22 //FeatherWing WS2812b LED is on SDA/pin 22.  

  //SPI pin definitions
  #define SCLK_PIN (26)
  #define MOSI_PIN (25)
  #define DC_PIN   (2)
  //#define DC_PIN   (13)
  #define CS_PIN  (9)
  //#define RST_PIN  (-1)
  
#else
  const char MCU_ID_STRING[16]="832";
  const uint8_t AINpinA_amp=0; //AIN0
  const uint8_t AINpinB_amp=1; //AIN1
  const uint8_t AINpinC_amp=2; //AIN2
  const uint8_t AINpinA_raw=3; //AIN3
  const uint8_t AINpinB_raw=4; //AIN4
  const uint8_t AINpinC_raw=5; //AIN5
  const byte AINBattMon = 6;  //AIN6

  const uint8_t MUX_LATCH = 16;
  const uint8_t MUX_CLK = 15;
  const uint8_t MUX_DATA = 7;

  const byte POWER_CONTROL = 11;
  const byte DIAG_PIN = 25;
  const byte BUTTON_PIN = 27;
  const byte LED1_PIN = 17;
  const byte LED2_PIN = 19;
  #define LED_DATA_PIN 25 //LEDs are on SDA/pin25

  //SPI pin definitions
  #define SCLK_PIN 12
  #define MOSI_PIN 13
  #define DC_PIN   20
  #define CS_PIN   31
  //#define RST_PIN  20
#endif
