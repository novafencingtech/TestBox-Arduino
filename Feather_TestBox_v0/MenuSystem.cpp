#include "MenuSystem.h"

// Constructor
MenuSystem::MenuSystem(Adafruit_SSD1351& display) : display(display) {
    selectedItemIndex = 0;
    currentPageIndex = 0;
    buttonState = 0;
    lastButtonPressTime = 0;

    // Default settings in the map
    settings = {{"Lame", "Graph"}, {"Weapon", "Graph"}};
    menuActive = false;
    initializeMenu();
}

// Update menu based on button interaction
void MenuSystem::updateMenu(bool buttonPressed, unsigned long currentTime) {
    if (buttonPressed) {
        if (buttonState == 0) {
            lastButtonPressTime = currentTime;
            buttonState = 1;  // Button is pressed
        } else if (buttonState == 1 && (currentTime - lastButtonPressTime >= MENU_LONG_PRESS_DURATION)) {
            // Long press detected, confirm the selected item
            buttonState = 2;
            confirmSelection();
        }
    } else if (buttonState == 1) {
        // Short press detected, switch item
        switchItem();
        buttonState = 0;  // Reset button state
    }
    renderMenu();
}

// Query configuration dictionary
std::string MenuSystem::querySetting(const std::string& key) {
    return settings[key];
}

// Initialize menu with entries
void MenuSystem::initializeMenu() {
    if (currentPageIndex == 0) {
        menuItems = {"Calibrate", "Lame", "Weapon", "Exit"};
    } else if (currentPageIndex == 1) {
        menuItems = {"Graph", "Number", "Back"};
    } else if (currentPageIndex == 2) {
        menuItems = {"Graph", "Number", "Back"};
    }
}

// Switch to the next item
void MenuSystem::switchItem() {
    selectedItemIndex++;
    if (selectedItemIndex >= menuItems.size()) {
        selectedItemIndex = 0;
    }
}

// Confirm selection and take action
void MenuSystem::confirmSelection() {
    std::string selectedItem = menuItems[selectedItemIndex];
    if (selectedItem == "Calibrate") {
        calibrate();
    } else if (selectedItem == "Lame") {
        currentPageIndex = 1;
        selectedItemIndex = 0;
    } else if (selectedItem == "Weapon") {
        currentPageIndex = 2;
        selectedItemIndex = 0;
    } else if (selectedItem == "Exit") {
        if (currentPageIndex > 0) {
            currentPageIndex = 0;
            selectedItemIndex = 0;
        }
        if (currentPageIndex == 0) {
            menuActive = false;
        }
    }
    initializeMenu();  // Refresh menu after page change
}

// Render the menu on the screen
void MenuSystem::renderMenu() {
    display.fillScreen(menuColorBLACK);
    display.setTextColor(menuColorWHITE);
    display.setTextSize(2);  // Set text size to 2 for larger text

    // Draw the menu header
    display.setCursor(0, 0);
    if (currentPageIndex == 0) {
        display.print("Settings:");
    } else if (currentPageIndex == 1) {
        display.print("Lame");
    } else if (currentPageIndex == 2) {
        display.print("Weapon");
    }

    // Draw each menu item
    for (int i = 0; i < menuItems.size(); i++) {
        display.setCursor(0, (i + 1) * 24);  // Increase line spacing for larger text size
        if (i == selectedItemIndex) {
            display.setTextColor(menuColorBLUE);  // Active selection color
            display.print("> ");
        } else {
            display.setTextColor(menuColorWHITE);
        }
        display.print(menuItems[i].c_str());

        // If the item is a stored setting, draw a light blue-green box around it
        std::string currentSetting = querySetting(menuItems[i]);
        if (!currentSetting.empty()) {
            display.setTextColor(menuColorCYAN);  // Light blue-green color for settings
            display.setCursor(64, (i + 1) * 24);  // Print the setting next to the item
            display.print(currentSetting.c_str());

            // Draw a white box around the setting
            drawSelectionBox(64, (i + 1) * 24, 60, 24);
        }
    }

    //display.display();
}

// Draw a white box around a setting
void MenuSystem::drawSelectionBox(int x, int y, int w, int h) {
    display.drawRect(x - 2, y - 2, w + 4, h + 4, menuColorWHITE);  // Draw a box around the setting
}

// Placeholder function for calibration
void MenuSystem::calibrate() {
    // Perform calibration logic here
}
