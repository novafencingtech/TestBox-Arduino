// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128 // Change this to 96 for 1.27" OLED.

/*
// You can use any (4 or) 5 pins
#define SCLK_PIN 12
#define MOSI_PIN 13
#define DC_PIN   20
#define CS_PIN   31
//#define RST_PIN  20
*/

// Color definitions
#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define ORANGE          0xFD20
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define GREY            0x79EF
#define DARKBLUE        0x0007

#define BARCOLOR  YELLOW
#define GRAPHCOLOR  RED

const int barHeight=10;
const int ABAR=20; //Vertical location of first bar graph
const int BBAR=ABAR+16+barHeight+10;
const int CBAR=BBAR+16+barHeight+10;

//#include <Adafruit_SPITFT.h>
//#include <SPI.h>
