#include "MenuSystem.h"

//Adafruit_LittleFS_Namespace::File settingsFile(Adafruit_LittleFS);

// Constructor
MenuSystem::MenuSystem(Adafruit_SSD1351& display)
  : display(display) {
  cursorItemIndex = 0;
  currentPageIndex = 0;
  buttonState = 0;
  lastButtonPressTime = 0;

  // Default settings in the map

  menuActive = false;
  initializeMenu();
}

MenuSystem::MenuSystem(Adafruit_SSD1351& display, const char* version)
  : MenuSystem(display) {
  
  strncpy(settings.version,version,16);
  //resetToDefaults();
  //loadSettings();
}

MenuSystem::Settings MenuSystem::getSettings() const {
  return settings;
}

void MenuSystem::resetToDefaults() {
  // Set default values for the settings
  //strncpy(settings.version, CURRENT_VERSION, sizeof(settings.version));
  Serial.println("Loading default settings...");
  settings.lameDisplay = GRAPH;
  settings.weaponDisplay = GRAPH;
  settings.advancedMode = false;
  display.fillScreen(menuColorBLACK);
  display.setCursor(0, 50);
  display.setTextColor(menuColorORANGE);
  display.println("Defaults\n  loaded");
  delay(1000);
}


const char* MenuSystem::displayTypeToString(displayType val) {
  switch (val) {
    case GRAPH:
      return "Graph";
    case NUMERIC:
      return "Numeric";
    case BAR:
      return "Bar";
    case NONE:
      return "None";
    default:
      return "Error";
  }
}

// Update menu based on button interaction
void MenuSystem::updateMenu(bool buttonPressed, unsigned long currentTime) {
  unsigned long debounceTime = 20;
  if (buttonPressed) {
    if (buttonState == 0) {
      lastButtonPressTime = currentTime;
      buttonState = 1;  // Button is pressed
    }
    if ((buttonState == 1) && (currentTime - lastButtonPressTime >= MENU_LONG_PRESS_DURATION)) {
      // Long press detected, confirm the selected item
      buttonState = 2;
      confirmSelection();
    }
  }
  if ((buttonState != 0) && (!buttonPressed)) {
    // Short press detected, switch item
    if (buttonState == 1) { switchItem(); }
    buttonState = 0;  // Reset button state
  }
  renderMenu();
}

// Query configuration dictionary
/*
std::string MenuSystem::querySetting(const std::string& key) {
    return settings[key];
}
*/

// Initialize menu with entries
void MenuSystem::initializeMenu() {
  if (currentPageIndex == 0) {
    strcpy(menuHeading, "Settings");
    strcpy(menuItems[0], "Calibrate");
    strcpy(menuItems[lameSettingsPage], "Lame");
    strcpy(menuItems[weaponSettingsPage], "Weapon");
    strcpy(exitText, "Exit");
    menuItemsCount = 3;
    cursorItemIndex = 0;
    highlightItemIndex = menuItemsCount + 2;
  } else if (currentPageIndex == lameSettingsPage) {
    strcpy(menuHeading, "Lame");
    strcpy(menuItems[0], "Graph");
    strcpy(menuItems[1], "Numeric");
    strcpy(exitText, "Back");
    switch (lameDispSetting) {
      case GRAPH:
        highlightItemIndex = 0;
        break;
      case NUMERIC:
        highlightItemIndex = 1;
        break;
    }
    cursorItemIndex = 0;
    menuItemsCount = 2;
  } else if (currentPageIndex == weaponSettingsPage) {
    strcpy(menuHeading, "Weapon");
    strcpy(menuItems[0], "Graph");
    strcpy(menuItems[1], "Numeric");
    strcpy(exitText, "Back");
    switch (weaponDispSetting) {
      case GRAPH:
        highlightItemIndex = 0;
        break;
      case NUMERIC:
        highlightItemIndex = 1;
        break;
    }
    cursorItemIndex = 0;
    menuItemsCount = 2;
  }
}

// Switch to the next item
void MenuSystem::switchItem() {
  cursorItemIndex++;
  if (cursorItemIndex >= menuItemsCount + 1) {
    cursorItemIndex = 0;
  }
  menuChanged = true;
}

// Confirm selection and take action
void MenuSystem::confirmSelection() {
  char* selectedItem = menuItems[cursorItemIndex];
  Serial.print("Confirming setting: ");
  Serial.println(selectedItem);
  if (currentPageIndex == 0) {
    if (cursorItemIndex == 0) {
      calibrate();
    }
    if (cursorItemIndex == lameSettingsPage) {
      currentPageIndex = lameSettingsPage;
      cursorItemIndex = 0;
    }
    if (cursorItemIndex == weaponSettingsPage) {
      currentPageIndex = weaponSettingsPage;
      cursorItemIndex = 0;
    }
    if (cursorItemIndex == menuItemsCount) {
      menuActive = false;
      if (settingsChanged) {saveSettings();}
    }
  }

  if (currentPageIndex == lameSettingsPage) {
    //Back to root menu
    if (cursorItemIndex == menuItemsCount) {
      currentPageIndex = 0;
      cursorItemIndex = 0;
    } else {
      if (strcmp(selectedItem, "Graph")) {
        settingsChanged = true;
        lameDispSetting = GRAPH;
      }
      if (strcmp(selectedItem, "Numeric")) {
        settingsChanged = true;
        lameDispSetting = NUMERIC;
      }
      highlightItemIndex = cursorItemIndex;
    }
  }
  if (currentPageIndex == weaponSettingsPage) {
    //Back to root menu
    if (cursorItemIndex == menuItemsCount) {
      currentPageIndex = 0;
      cursorItemIndex = 0;
    }
  }

  menuChanged = true;
  initializeMenu();  // Refresh menu after page change
}

// Render the menu on the screen
void MenuSystem::renderMenu() {
  //If nothing changed do no re-draw
  if (!menuChanged) return;

  display.fillScreen(menuColorBLACK);
  if (!menuActive) return;
  display.setTextSize(2);  // Set text size to 2 for larger text

  // Draw the menu header
  display.setCursor(0, 0);
  display.setTextColor(menuColorWHITE);
  if (currentPageIndex == 0) {
    display.print("Settings:");
  } else if (currentPageIndex == 1) {
    display.print("Lame");
  } else if (currentPageIndex == 2) {
    display.print("Weapon");
  }

  // Draw each menu item
  for (int i = 0; i < menuItemsCount; i++) {
    if (i == cursorItemIndex) {
      display.setTextColor(menuColorPaleBLUE);  // Active selection color
      display.setCursor(0, (i + 1) * 24);       // Increase line spacing for larger text size
      display.print("> ");
    } else {
      display.setTextColor(menuColorWHITE);
    }
    display.setTextColor(menuColorGREY);
    display.setCursor(15, (i + 1) * 24);  // Increase line spacing for larger text size
    display.print(menuItems[i]);

    // If the item is a stored setting, draw a light blue-green box around it
    //std::string currentSetting = querySetting(menuItems[i]);
    if (i == highlightItemIndex) {
      display.setTextColor(menuColorCYAN);  // Light blue-green color for settings
      display.setCursor(15, (i + 1) * 24);  // Print the setting next to the item
      display.print(menuItems[highlightItemIndex]);

      // Draw a white box around the setting
      display.drawRect(15 - 2, (i + 1) * 24 - 2, 12 * strlen(menuItems[highlightItemIndex]) + 4, 16 + 4, menuColorCYAN);
    }
  }

  //Print the exit button at the bottom
  if (cursorItemIndex == menuItemsCount) {
    display.setCursor(60 - 14, 112);
    display.setTextColor(menuColorPaleBLUE);
    display.print(">");
  }
  display.setCursor(60, 112);
  display.setTextColor(menuColorRED);
  display.print(exitText);

  menuChanged = false;
  //display.display();
}

// Draw a white box around a setting
void MenuSystem::drawSelectionBox(int x, int y, int w, int h) {
  // Draw a box around the setting
}

// Placeholder function for calibration
void MenuSystem::calibrate() {
  // Perform calibration logic here
}

void MenuSystem::loadSettings() {

  Adafruit_LittleFS_Namespace::File settingsFile(InternalFS);

  InternalFS.begin();

  if (settingsFile.open(settingsFileName, Adafruit_LittleFS_Namespace::FILE_O_READ)) {
    Settings readSettings;
    settingsFile.read(reinterpret_cast<uint8_t*>(&readSettings), sizeof(Settings));
    settingsFile.close();

    // Check the version before applying the settings
    if (strncmp(readSettings.version, settings.version, sizeof(settings.version)) == 0) {
      settings = readSettings;  // Only apply settings if the version matches
      display.fillScreen(menuColorBLACK);
      display.setTextColor(menuColorCYAN);
      display.setCursor(0, 50);
      display.println("Settings\n  loaded");
      delay(1000);
    } else {
      
      resetToDefaults();
    }
  } else {
    // If the file doesn't exist or fails to read, use default settings
    resetToDefaults();
  }
}

void MenuSystem::saveSettings() {

  Adafruit_LittleFS_Namespace::File settingsFile(InternalFS);
  //InternalFS.begin();

  if (settingsFile.open(settingsFileName, Adafruit_LittleFS_Namespace::FILE_O_WRITE)) {
    // Ensure the version is set correctly before saving
    //strncpy(settings.version, CURRENT_VERSION, sizeof(settings.version));
    settingsFile.write(reinterpret_cast<const uint8_t*>(&settings), sizeof(Settings));
    settingsFile.close();
    display.fillScreen(menuColorBLACK);
    display.setTextColor(menuColorCYAN);
    display.setCursor(0, 50);
    display.println("Settings\n  saved");
    delay(1000);
  }
  settingsChanged = false;
}
