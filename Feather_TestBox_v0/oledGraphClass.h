/*
  oledGraphClass.h - Class for plotting 

*/
#ifndef oledGraphClass_h
#define oledGraphClass_h

#define MAX_NUM_BARS 10
#include <Adafruit_SSD1351.h>
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

class oledColorList {
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
    static const int cLIGHTRED=0xF807;
    static const int cLIGHTGREEN=0x1FE8;
};

class oledGraph: oledColorList
{
  public:    
    oledGraph();
    oledGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width); 
    oledGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width,float minValue, float maxValue); 
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
    int _maxX=127;
    int _locY=0;
    int _maxY=127;
    int _col=0;
    int _height, _width;
    float _limitMax, _limitMin;
    float _pixelScaleFactor=1.0;
    float _offset=0.0;
    int _hBarY[MAX_NUM_BARS];
    int _hBarValues[MAX_NUM_BARS];
    int _hBarColors[MAX_NUM_BARS];
    int _numActiveBars=0;
    Adafruit_SSD1351 *_tft;
};

class oledReverseHBarGraph: oledColorList
{
  public:
   
    oledReverseHBarGraph();
    oledReverseHBarGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width); 
    oledReverseHBarGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width,float minValue, float maxValue); 
    //void setGraphLimits(int X, int Y, int H, int W);
    void setLimit(float minValue, float maxValue);
    void setBarColors(int numBars, float values[], int colors[]);
    void updateGraph(float newValue);
    void resetGraph();
    //void drawTextLabels();
    //void drawHLines();
    //void drawTextLabels();
    
  private:
    //void drawColumn(float val);
    int getBarEnd(float val);
    int getLineColor(int col);

    int _locX=0;
    int _maxX=127;
    int _locY=0;
    int _maxY=127;
    int _col=0;
    int _height, _width;
    float _limitMax, _limitMin;
    float _pixelScaleFactor=1.0;
    float _offset=0.0;
    int _hBarX[MAX_NUM_BARS];
    int _hBarValues[MAX_NUM_BARS];
    int _hBarColors[MAX_NUM_BARS];
    int _numActiveBars=0;
    float _barValue=0.0f;
    int _barEnd=0;
    Adafruit_SSD1351 *_tft;
};

class oledGraphLabel: oledColorList {
  public:
    oledGraphLabel();
    oledGraphLabel(Adafruit_SSD1351 *tft,uint16_t x, uint16_t y, int size=2, uint16_t color=cYELLOW);
    void setFontSize(int size);
    void printLabel(const char *lab, float val, bool forceColor=false, uint16_t newColor=cYELLOW);
    void setLabelColor(uint16_t newColor);
    void setColors(int numBars, float values[], int colors[]);
    
  private:
    Adafruit_SSD1351 *_tft;
    float _colorValues[MAX_NUM_BARS]={5.0f, 10.0f, 20.0f};
    uint16_t _colorList[MAX_NUM_BARS]={cGREEN, cYELLOW, cLIGHTRED};
    uint16_t _lblColor=cYELLOW;
    int _numColors=3;
    uint8_t _fontSize=2;
    uint16_t _locX=0;
    uint16_t _locY=0;
    //bool _openTxt=false;
};



#endif
