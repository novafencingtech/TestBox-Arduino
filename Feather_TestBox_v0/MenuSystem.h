#ifndef MENUSYSTEM_H
#define MENUSYSTEM_H

#include <Adafruit_SSD1351.h>

#define MENU_LONG_PRESS_DURATION 1000 // Milliseconds for a long press
#define MENU_MAX_STR_LEN 24

class MenuSystem {
public:
    // Constructor
    MenuSystem(Adafruit_SSD1351& display);

    enum displayType {
      GRAPH = 0,
      NUMERIC,
      BAR,
      NONE,
    };

    // Update menu based on button interaction
    void updateMenu(bool buttonPressed, unsigned long currentTime);

    const char* displayTypeToString(displayType);

    // Query configuration dictionary
    //std::string querySetting(const std::string& key);    

    bool isActive() const { return menuActive; }
    void activateMenu() {menuActive=true; return;}
   
    const static uint16_t maxStrLen = 24;

    displayType lameDispSetting = GRAPH;
    displayType weaponDispSetting = GRAPH;

private:
    const uint16_t menuColorBLACK     = 0x0000;
    const uint16_t menuColorBLUE      = 0x001F;
    const uint16_t menuColorPaleBLUE  = 0xAEDC;
    const uint16_t menuColorRED       = 0xF800;
    const uint16_t menuColorGREEN     = 0x07E0;
    const uint16_t menuColorCYAN      = 0x07FF;
    const uint16_t menuColorMAGENTA   = 0xF81F;
    const uint16_t menuColorORANGE    = 0xFD20;
    const uint16_t menuColorYELLOW    = 0xFFE0;
    const uint16_t menuColorWHITE     = 0xFFFF;
    const uint16_t menuColorGREY      = 0xBDF7;
    const uint16_t menuColorDARKBLUE  = 0x0007;

    const static uint16_t maxMenuItems = 6;
    uint8_t menuItemsCount=0;
    Adafruit_SSD1351 &display;
    char menuItems[maxMenuItems][maxStrLen];
    char menuHeading[maxStrLen];
    char exitText[maxStrLen];
    //std::map<std::string, std::string> settings; // Store settings in an STL map
    uint8_t cursorItemIndex; // Currently selected menu item
    uint8_t currentPageIndex;  // Current page
    uint8_t buttonState;       // 0: released, 1: pressed, 2: confirmed
    uint8_t highlightItemIndex;  //Which index is currently active
    unsigned long lastButtonPressTime;
    bool menuActive=false; 
    bool menuChanged=false; 
    const uint8_t lameSettingsPage = 1;
    const uint8_t weaponSettingsPage = 2;
    

    // Initialize menu with entries
    void initializeMenu();

    // Switch to the next item
    void switchItem();

    // Confirm selection and take action
    void confirmSelection();

    // Render the menu on the screen
    void renderMenu();

    // Draws a white box around a setting
    void drawSelectionBox(int x, int y, int w, int h);

    // Placeholder function for calibration
    void calibrate();
};

#endif // MENUSYSTEM_H
