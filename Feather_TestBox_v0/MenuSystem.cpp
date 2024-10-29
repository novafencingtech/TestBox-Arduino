#include "MenuSystem.h"

// Uncomment and adjust the following line if using LittleFS
// Adafruit_LittleFS_Namespace::File settingsFile(Adafruit_LittleFS);

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

  // Issue 1 Fix: Ensure strncpy null-terminates the string
  strncpy(settings.version, version, sizeof(settings.version) - 1);
  settings.version[sizeof(settings.version) - 1] = '\0';

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
  // Issue 2 Fix: Removed unused debounceTime
  // unsigned long debounceTime = 20;

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
  } else {
    switch (buttonState) {
      case 0:  // Button not pressed
        break;
      case 1:  // Button was pressed and is now released
        switchItem();
        break;  // Issue 2 Fix: Added missing break statement
      case 2:   // Confirmed has released
        break;
      default:
        break;
    }
    buttonState = 0;
  }
  renderMenu();
}

// Initialize menu with entries
void MenuSystem::initializeMenu() {
  //printSettings();
  //Serial.print("Lame Display: ");
  //Serial.println(displayTypeToString(settings.lameDisplay));
  if (currentPageIndex == 0) {
    strcpy(menuHeading, "Settings");
    strcpy(menuItems[calibratePage - 1], "Calibrate");
    strcpy(menuItems[lameSettingsPage - 1], "Lame");
    strcpy(menuItems[weaponSettingsPage - 1], "Weapon");
    strcpy(menuItems[advancedModePage - 1], "User Mode");
    strcpy(exitText, "Exit");
    menuItemsCount = 4;
    cursorItemIndex = 0;
    highlightItemIndex = menuItemsCount + 2;
  } else if (currentPageIndex == lameSettingsPage) {
    strcpy(menuHeading, "Lame");
    strcpy(menuItems[0], "Graph");
    strcpy(menuItems[1], "Numeric");
    strcpy(exitText, "Back");
    switch (settings.lameDisplay) {
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
    switch (settings.weaponDisplay) {
      case GRAPH:
        highlightItemIndex = 0;
        break;
      case NUMERIC:
        highlightItemIndex = 1;
        break;
    }
    cursorItemIndex = 0;
    menuItemsCount = 2;
  } else if (currentPageIndex == advancedModePage) {
    strcpy(menuHeading, "Box Mode");
    strcpy(menuItems[0], "Basic");
    strcpy(menuItems[1], "Advanced");
    strcpy(exitText, "Back");
    if (settings.advancedMode) {
      highlightItemIndex = 1;
    } else {
      highlightItemIndex = 0;
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
    if (cursorItemIndex == calibratePage-1) {
      calibrate();
    }
    if (cursorItemIndex == lameSettingsPage-1) {
      currentPageIndex = lameSettingsPage;
      cursorItemIndex = 0;
    }
    if (cursorItemIndex == advancedModePage-1) {
      currentPageIndex = advancedModePage;
      cursorItemIndex = 0;
    }
    if (cursorItemIndex == weaponSettingsPage-1) {
      currentPageIndex = weaponSettingsPage;
      cursorItemIndex = 0;
    }
    if (cursorItemIndex == menuItemsCount) {
      menuActive = false;
      if (settingsChanged) { saveSettings(); }
    }
  }

  if (currentPageIndex == lameSettingsPage) {
    // Back to root menu
    if (cursorItemIndex == menuItemsCount) {
      currentPageIndex = 0;
      cursorItemIndex = 0;
    } else {
      // Issue 3 Fix: Corrected strcmp conditions to check for equality
      if (strcmp(selectedItem, "Graph") == 0) {
        settingsChanged = true;
        settings.lameDisplay = GRAPH;
      }
      if (strcmp(selectedItem, "Numeric") == 0) {
        settingsChanged = true;
        settings.lameDisplay = NUMERIC;
      }
      //highlightItemIndex = cursorItemIndex;
    }
  }
  if (currentPageIndex == weaponSettingsPage) {
    // Back to root menu
    if (cursorItemIndex == menuItemsCount) {
      currentPageIndex = 0;
      cursorItemIndex = 0;
    } else {
      // Issue 3 Fix: Corrected strcmp conditions to check for equality
      if (strcmp(selectedItem, "Graph") == 0) {
        settingsChanged = true;
        settings.weaponDisplay = GRAPH;
      }
      if (strcmp(selectedItem, "Numeric") == 0) {
        settingsChanged = true;
        settings.weaponDisplay = NUMERIC;
      }
      //highlightItemIndex = cursorItemIndex;
    }
  }
  if (currentPageIndex == advancedModePage) {
    // Back to root menu
    if (cursorItemIndex == menuItemsCount) {
      currentPageIndex = 0;
      cursorItemIndex = 0;
    } else {
      // Issue 3 Fix: Corrected strcmp conditions to check for equality
      if (strcmp(selectedItem, "Basic") == 0) {
        settingsChanged = true;
        settings.advancedMode = false;
      }
      if (strcmp(selectedItem, "Advanced") == 0) {
        settingsChanged = true;
        settings.advancedMode = true;
      }
      //highlightItemIndex = cursorItemIndex;
    }
  }

  menuChanged = true;
  initializeMenu();  // Refresh menu after page change
}

// Render the menu on the screen
void MenuSystem::renderMenu() {
  // If nothing changed, do not re-draw
  if (!menuChanged) return;

  Serial.println("Rendering menu");
  Serial.print("Page: ");
  Serial.print(currentPageIndex);
  Serial.print("\tCursor: ");
  Serial.print(cursorItemIndex);
  Serial.print("\tHighlight: ");
  Serial.print(highlightItemIndex);

  display.fillScreen(menuColorBLACK);

  if (!menuActive) return;

  display.setTextSize(2);  // Set text size to 2 for larger text

  // Draw the menu header
  display.setCursor(0, 0);
  display.setTextColor(menuColorWHITE);
  if (currentPageIndex == 0) {
    display.print("Settings:");
  } else if (currentPageIndex == lameSettingsPage) {
    display.print("Lame");
  } else if (currentPageIndex == weaponSettingsPage) {
    display.print("Weapon");
  } else if (currentPageIndex == advancedModePage) {
    display.print("Box Mode");
  }

  // Draw each menu item
  for (int i = 0; i < menuItemsCount; i++) {
    if (i == cursorItemIndex) {
      display.setTextColor(menuColorPaleBLUE);  // Active selection color
      display.setCursor(0, (i + 1) * 22);       // Increase line spacing for larger text size
      display.print("> ");
    } else {
      display.setTextColor(menuColorGREY);
    }
    display.setCursor(15, (i + 1) * 22);  // Position the text
    display.print(menuItems[i]);

    // If the item is a stored setting, draw a light blue-green box around it
    //std::string currentSetting = querySetting(menuItems[i]);
    if (i == highlightItemIndex) {
      display.setTextColor(menuColorCYAN);  // Light blue-green color for settings
      display.setCursor(15, (i + 1) * 22);  // Print the setting next to the item
      display.print(menuItems[i]);

      // Draw a white box around the setting
      display.drawRect(15 - 2, (i + 1) * 22 - 2, 12 * strlen(menuItems[i]) + 4, 16 + 4, menuColorCYAN);
    }
  }

  // Print the exit button at the bottom
  if (cursorItemIndex == menuItemsCount) {
    display.setCursor(70 - 14, 5 * 22);
    display.setTextColor(menuColorPaleBLUE);
    display.print(">");
  }
  display.setCursor(70, 5 * 22);
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
  // Issue 2 Fix: Initialize InternalFS before opening files
  if (!InternalFS.begin()) {
    Serial.println("Failed to initialize filesystem");
    resetToDefaults();
    return;
  }

  Adafruit_LittleFS_Namespace::File settingsFile = InternalFS.open(settingsFileName, Adafruit_LittleFS_Namespace::FILE_O_READ);

  if (settingsFile) {  // Equivalent to checking if the file was successfully opened
    Settings readSettings;
    if (settingsFile.read(reinterpret_cast<uint8_t*>(&readSettings), sizeof(Settings)) == sizeof(Settings)) {
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
      Serial.println("Failed to read settings from file");
      resetToDefaults();
    }
    settingsFile.close();
  } else {
    Serial.println("Settings file not found, loading defaults");
    // If the file doesn't exist or fails to read, use default settings
    resetToDefaults();
  }
}

void MenuSystem::saveSettings() {
  // Issue 2 Fix: Initialize InternalFS before opening files
  if (!InternalFS.begin()) {
    Serial.println("Failed to initialize filesystem");
    return;
  }

  Adafruit_LittleFS_Namespace::File settingsFile = InternalFS.open(settingsFileName, Adafruit_LittleFS_Namespace::FILE_O_WRITE);

  if (settingsFile) {  // Equivalent to checking if the file was successfully opened
    // Ensure the version is set correctly before saving
    //strncpy(settings.version, CURRENT_VERSION, sizeof(settings.version));
    if (settingsFile.write(reinterpret_cast<const uint8_t*>(&settings), sizeof(Settings)) == sizeof(Settings)) {
      display.fillScreen(menuColorBLACK);
      display.setTextColor(menuColorCYAN);
      display.setCursor(0, 50);
      display.println("Settings\n  saved");
      delay(1000);
    } else {
      Serial.println("Failed to write settings to file");
    }
    settingsFile.close();
    settingsChanged = false;
  } else {
    Serial.println("Failed to open settings file for writing");
  }
}

/*
void MenuSystem::printSettings() {
    Serial.println("Settings Status:");
    Serial.print("Version: ");
    Serial.println(settings.version);
    
    Serial.print("Lame Display: ");
    Serial.println(displayTypeToString(settings.lameDisplay));
    
    Serial.print("Weapon Display: ");
    Serial.println(displayTypeToString(settings.weaponDisplay));
    
    Serial.print("Advanced Mode: ");
    Serial.println(settings.advancedMode ? "Enabled" : "Disabled");
    
    Serial.println(); // For spacing between different outputs
}
*/
