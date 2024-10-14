#ifndef MENUSYSTEM_H
#define MENUSYSTEM_H

#include <Adafruit_SSD1351.h>
#include <map>
#include <string>
#include <vector>

#define MENU_LONG_PRESS_DURATION 1000 // Milliseconds for a long press

class MenuSystem {
public:
    // Constructor
    MenuSystem(Adafruit_SSD1351& display);

    // Update menu based on button interaction
    void updateMenu(bool buttonPressed, unsigned long currentTime);

    // Query configuration dictionary
    std::string querySetting(const std::string& key);

    bool isActive() const { return menuActive; }
    void activateMenu() {menuActive=true; return;}

private:
    const uint16_t menuColorBLACK     = 0x0000;
    const uint16_t menuColorBLUE      = 0x001F;
    const uint16_t menuColorRED       = 0xF800;
    const uint16_t menuColorGREEN     = 0x07E0;
    const uint16_t menuColorCYAN      = 0x07FF;
    const uint16_t menuColorMAGENTA   = 0xF81F;
    const uint16_t menuColorORANGE    = 0xFD20;
    const uint16_t menuColorYELLOW    = 0xFFE0;
    const uint16_t menuColorWHITE     = 0xFFFF;
    const uint16_t menuColorGREY      = 0x79EF;
    const uint16_t menuColorDARKBLUE  = 0x0007;

    Adafruit_SSD1351 &display;
    std::vector<std::string> menuItems;
    std::map<std::string, std::string> settings; // Store settings in an STL map
    int selectedItemIndex; // Currently selected menu item
    int currentPageIndex;  // Current page
    int buttonState;       // 0: released, 1: pressed, 2: confirmed
    unsigned long lastButtonPressTime;
    bool menuActive=false;

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
