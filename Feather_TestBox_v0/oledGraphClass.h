/*
  oledGraphClass.h - Class for plotting 

*/
#ifndef oledGraphClass_h
#define oledGraphClass_h

#define MAX_NUM_BARS 10
#define TITLE_BUF_SIZE 7
#include <Adafruit_ILI9341.h>
//#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>

#include <avr/dtostrf.h>


// Color definitions
/*#define BLACK           0x0000
#define BLUE            0x001F
#define RED             0xF800
#define GREEN           0x07E0
#define CYAN            0x07FF
#define MAGENTA         0xF81F
#define ORANGE          0xFD20
#define YELLOW          0xFFE0
#define WHITE           0xFFFF
#define GREY            0x79EF
#define DARKBLUE        0x0007*/

class oledGraph
{
  public:
    static const int cBLACK=0x0000;
    static const int cWHITE=0xFFFF;
    static const int cBLUE=0x001F;
    static const int cGREEN=0x07E0;
    static const int cCYAN=0x07FF;
    static const int cRED=0xF800;
    static const int cMAGENTA=0xF81F;
    static const int cORANGE=0xFD20;
    static const int cYELLOW=0xFFE0;    
    
    oledGraph();
    oledGraph(Adafruit_ILI9341 *tft,int X, int Y, int height, int width); 
    oledGraph(Adafruit_ILI9341 *tft,int X, int Y, int height, int width,float minValue, float maxValue); 
    void setGraphLimits(int X, int Y, int H, int W);
    void setVerticalLimits(float minValue, float maxValue);
    void setHorizontalBarValues(int numBars, float values[], int colors[]);
    void updateGraph(float newValue);
    void resetGraph();
    void drawTextLabels();
    
  private:
    void drawColumn(float val);
    void drawHLines();
    //void drawTextLabels();

    int _locX=0;
    int _maxX=319;
    int _locY=0;
    int _maxY=239;
    int _col=0;
    int _height, _width;
    float _limitMax, _limitMin;
    float _pixelScaleFactor=1.0;
    float _offset=0.0;
    int _hBarY[MAX_NUM_BARS];
    int _hBarValues[MAX_NUM_BARS];
    int _hBarColors[MAX_NUM_BARS];
    int _numActiveBars=0;
    Adafruit_ILI9341 *_tft;
};

class oledBarGraph
{
  public:
    static const int cBLACK=0x0000;
    static const int cWHITE=0xFFFF;
    static const int cBLUE=0x001F;
    static const int cGREEN=0x07E0;
    static const int cCYAN=0x07FF;
    static const int cRED=0xF800;
    static const int cMAGENTA=0xF81F;
    static const int cORANGE=0xFD20;
    static const int cYELLOW=0xFFE0;
    static const int _lblOffset=-18; //Shift the graph label up by 16 pixels
    
    
    oledBarGraph();
    oledBarGraph(Adafruit_ILI9341 *tft,int X, int Y, int height, int width); 
    oledBarGraph(Adafruit_ILI9341 *tft,int X, int Y, int height, int width,float minValue, float maxValue); 
    void setGraphLimits(float minValue, float maxValue);
    void setHorizontalBarValues(int numBars, float values[], int colors[]);
    void updateGraph(float newValue);
    void resetGraph();
    //void drawTextLabels();
    void drawTickMarks();
    void setTitle(char *newTitle);
    void updateTextValue(float value);
    void setTitlePosition(int X, int Y);
    void setTitleSize(int txtSize);
    
  private:
    void drawColumn(float val);

    char _title[TITLE_BUF_SIZE];
    char _value[5];
    int _locX=0;
    static const int _maxX=319;
    int _locY=0;
    static const int _maxY=239;
    int _height=16;
    int _width=320;   
    float _lastValue=0;
    float _noDisplayLimit=32.0;    
    float _limitMax, _limitMin;
    float _pixelScaleFactor=1.0;
    float _offset=0.0;
    int _hBarX[MAX_NUM_BARS];
    float _hBarValues[MAX_NUM_BARS];
    int _hBarColors[MAX_NUM_BARS];
    int _numActiveBars=0;
    int _prevColor;
    int _prevEnd;
    int _titleTextPosX;
    int _titleTextPosY;
    int _valTextPosX;
    int _valTextPosY;
    int _txtSize=3;    
    Adafruit_ILI9341 *_tft;
};


#endif
