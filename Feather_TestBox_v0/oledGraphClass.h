/*
  oledGraphClass.h - Class for plotting 

*/
#ifndef oledGraphClass_h
#define oledGraphClass_h

#define MAX_NUM_BARS 10

class oledGraph
{
  public:  
    oledGraph(Adafruit_SSD1351 *tft,int X, int Y, int height, int width); 
    setGraphLimits(int X, int Y, int H, int W);
    setVerticalLimits(int lower, int upper);
    setHorizontalBarValues(int numBars, int[] values, int[] colors);
    updateGraph(int newValue);
    resetGraph();
    drawTextLabels();
    
  private:
    drawColumn(int val);

    int _locX=0;
    int _maxX=127;
    int _locY=0;
    int _maxY=127;
    int _col=0;
    int _height, _width;
    float _limitMax, _limitMin;
    float _pixelScaleFactor=1.0;
    float _offset=0.0;
    int[MAX_NUM_BARS] _hBarY;
    int[MAX_NUM_BARS] _hBarValues;
    int[MAX_NUM_BARS] _hBarColors;
    int _numActiveBars=0;
    Adafruit_SSD1351 *_tft;
};

#endif
