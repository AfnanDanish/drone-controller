#include <U8g2lib.h>
#include <Wire.h>
#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <EEPROM.h>
#include <mbedtls/aes.h>

// LoRa Module Pins
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// Joystick Pins
#define LEFT_JOYSTICK_X 34  // Yaw
#define LEFT_JOYSTICK_Y 35  // Throttle
#define RIGHT_JOYSTICK_X 32 // Roll
#define RIGHT_JOYSTICK_Y 33 // Pitch
#define LEFT_JOYSTICK_SW 0   // Left joystick press button - using GPIO 0
#define RIGHT_JOYSTICK_SW 5  // Right joystick press button - using GPIO 5

// Button/Switch Pins
#define DEBUG_SWITCH_PIN 13
#define KILL_SWITCH_PIN 12
#define ROTARY_ENABLE_PIN 27
#define ROTARY_CLK_PIN 25
#define ROTARY_DT_PIN 15
#define ROTARY_SW_PIN 4

// LED Pins
#define STATUS_LED_GREEN 2
#define STATUS_LED_RED 16
#define BATTERY_LED_1 17
#define BATTERY_LED_2 21
#define BATTERY_LED_3 22

// Vibration Motor
#define VIBRATION_PIN 23

// Battery Monitoring
#define BATTERY_PIN 36
#define VBAT_MIN 3.3
#define VBAT_MAX 4.2

// OLED Display
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0);

// AES Encryption
mbedtls_aes_context aes;
unsigned char aes_key[16] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 
                            0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

// Controller States
enum ControllerState {
    PLATFORM_SELECT,
    TUNING_PAGE,
    SETTINGS_PAGE,
    CALIBRATION_PAGE
};

enum PlatformType {
    ORNITHOPTER,
    PLANE,
    HELICOPTER,
    QUADCOPTER
};

// Global Variables
ControllerState currentState = PLATFORM_SELECT;
PlatformType currentPlatform = QUADCOPTER;
int menuPosition = 0;
int menuOffset = 0;
bool isConnected = false;
bool isArmed = false;
bool isGpsLocked = false;
bool debugMode = false;  // Added debug mode flag - when true, joysticks control UI instead of aircraft
int batteryLevel = 100;
int throttleValue = 0;
bool rotaryEnabled = false;
int lastRotaryValue = 0;

// Joystick navigation variables
unsigned long lastJoystickMoveTime = 0;
const int joystickMoveDelay = 300; // ms between menu movements

// Encoder Variables
int encoderValue = 0;
int lastEncoderState = 0;
unsigned long lastEncoderTime = 0;

// Joystick Variables
int leftX = 0, leftY = 0, rightX = 0, rightY = 0;
int leftXOffset = 0, leftYOffset = 0, rightXOffset = 0, rightYOffset = 0;
int joystickDeadzone = 100;

// Function Prototypes
void setupDisplay();
void setupLoRa();
void setupJoysticks();
void setupButtons();
void setupEncoder();
void setupLEDs();
void updateDisplay();
void drawPlatformSelect();
void drawTuningPage();
void drawSettingsPage();
void drawCalibrationPage();
void handleJoysticks();
void handleButtons();
void handleEncoder();
void checkBattery();
void updateLEDs();
void triggerVibration(int duration);
void sendControlData();
void receiveTelemData();
void changeDroneType(int direction);
int mapJoystick(int value, int deadzone);
int readJoystick(int pin);
void encryptData(uint8_t* input, uint8_t* output, size_t length);
void decryptData(uint8_t* input, uint8_t* output, size_t length);

// Icon Bitmaps (8x8 pixels)
const unsigned char ornithopterIcon[] U8X8_PROGMEM = {
  0x00, 0x0C, 0x12, 0x24, 0x24, 0x12, 0x0C, 0x00
};

const unsigned char planeIcon[] U8X8_PROGMEM = {
  0x00, 0x00, 0x18, 0x3C, 0xFF, 0x18, 0x18, 0x00
};

const unsigned char helicopterIcon[] U8X8_PROGMEM = {
  0x00, 0x18, 0x18, 0xFF, 0x18, 0x18, 0x24, 0x00
};

const unsigned char quadcopterIcon[] U8X8_PROGMEM = {
  0x42, 0x24, 0x18, 0x18, 0x18, 0x18, 0x24, 0x42
};

void setup(void) {
  Serial.begin(115200);
  
  setupDisplay();
  setupLoRa();
  setupJoysticks();
  setupButtons();
  setupEncoder();
  setupLEDs();
  
  // Initialize AES encryption
  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, aes_key, 128);
  
  // Initialize EEPROM
  EEPROM.begin(512);
  
  // Show startup screen
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.drawStr(10, 20, "Drone Controller");
  u8g2.drawStr(10, 40, "Initializing...");
  u8g2.sendBuffer();
  
  // Wait for LoRa initialization and auto-binding
  delay(2000);
  
  // Attempt initial connection
  for (int i = 0; i < 3; i++) {
    u8g2.clearBuffer();
    u8g2.drawStr(10, 30, "Connecting...");
    u8g2.sendBuffer();
    
    // Try to connect (send heartbeat)
    // In a real implementation, we'd wait for a response
    delay(1000);
  }
  
  triggerVibration(200); // Short vibration to indicate system ready
}

void loop(void) {
  // Read inputs
  handleJoysticks();
  handleButtons();
  handleEncoder();
  
  // Check battery status
  checkBattery();
  
  // Update display
  updateDisplay();
  
  // Send control data to drone
  sendControlData();
  
  // Receive telemetry data from drone
  receiveTelemData();
  
  // Update status LEDs
  updateLEDs();
  
  delay(50); // 20Hz update rate
}

// Setup Functions
void setupDisplay() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setContrast(128);
}

void setupLoRa() {
  // Comment out actual LoRa initialization during testing
  Serial.println("LoRa module bypass enabled for testing");
  
  /* Original LoRa initialization code
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  if (!LoRa.begin(915E6)) { // 915MHz band
    Serial.println("LoRa initialization failed");
    while (1);
  }
  
  // Configure LoRa parameters
  LoRa.setSpreadingFactor(7);    // 7 is default
  LoRa.setSignalBandwidth(125E3); // 125kHz
  LoRa.setCodingRate4(5);        // 4/5
  LoRa.setPreambleLength(8);     // Default is 8
  LoRa.enableCrc();
  */
  
  // For testing - set connection as initially successful
  isConnected = true;
}

void setupJoysticks() {
  // Set pins as inputs
  pinMode(LEFT_JOYSTICK_X, INPUT);
  pinMode(LEFT_JOYSTICK_Y, INPUT);
  pinMode(RIGHT_JOYSTICK_X, INPUT);
  pinMode(RIGHT_JOYSTICK_Y, INPUT);
  
  // Set up joystick button pins with pullups
  pinMode(LEFT_JOYSTICK_SW, INPUT_PULLUP);
  pinMode(RIGHT_JOYSTICK_SW, INPUT_PULLUP);
  
  // Calibrate joystick centers
  leftXOffset = analogRead(LEFT_JOYSTICK_X);
  leftYOffset = analogRead(LEFT_JOYSTICK_Y);
  rightXOffset = analogRead(RIGHT_JOYSTICK_X);
  rightYOffset = analogRead(RIGHT_JOYSTICK_Y);
}

void setupButtons() {
  // For 3-pin slide switches:
  // - Connect middle pin (common) to GPIO pin
  // - Connect one outer pin to GND 
  // - Connect other outer pin to 3.3V with a 10K resistor (optional)
  pinMode(DEBUG_SWITCH_PIN, INPUT_PULLUP);
  pinMode(KILL_SWITCH_PIN, INPUT_PULLUP);
  pinMode(ROTARY_ENABLE_PIN, INPUT_PULLUP);
}

void setupEncoder() {
  // For rotary encoder:
  // - CLK and DT pins connect directly to GPIO pins (no pull-up needed)
  // - For 4-pin pushbutton (SW):
  //   - Connect pins 1.l or 1.r to the GPIO pin
  //   - Connect pins 2.l or 2.r to GND
  pinMode(ROTARY_CLK_PIN, INPUT);
  pinMode(ROTARY_DT_PIN, INPUT);
  pinMode(ROTARY_SW_PIN, INPUT_PULLUP);
  lastEncoderState = digitalRead(ROTARY_CLK_PIN);
}

void setupLEDs() {
  // For LEDs:
  // - Connect LED Anode (+) through a 220-330 ohm resistor to the GPIO pin
  // - Connect LED Cathode (-) directly to GND
  pinMode(STATUS_LED_GREEN, OUTPUT);
  pinMode(STATUS_LED_RED, OUTPUT);
  pinMode(BATTERY_LED_1, OUTPUT);
  pinMode(BATTERY_LED_2, OUTPUT);
  pinMode(BATTERY_LED_3, OUTPUT);
  
  // For vibration motor:
  // - Connect GPIO pin through a 1K resistor to the base of an NPN transistor
  // - Connect the collector of the transistor to the negative lead of the motor
  // - Connect the emitter of the transistor to GND
  // - Connect the positive lead of the motor to VCC (3.3V or 5V)
  // - Add a protection diode across the motor terminals
  pinMode(VIBRATION_PIN, OUTPUT);
  
  // Initial LED states
  digitalWrite(STATUS_LED_GREEN, HIGH);
  digitalWrite(STATUS_LED_RED, LOW);
}

// Display Functions
void updateDisplay() {
  u8g2.clearBuffer();
  
  switch (currentState) {
    case PLATFORM_SELECT:
      drawPlatformSelect();
      break;
    case TUNING_PAGE:
      drawTuningPage();
      break;
    case SETTINGS_PAGE:
      drawSettingsPage();
      break;
    case CALIBRATION_PAGE:
      drawCalibrationPage();
      break;
  }
  
  // Draw status bar at bottom
  u8g2.drawHLine(0, 54, 128);
  
  // Connection status
  u8g2.drawStr(2, 63, isConnected ? "LINK:OK" : "LINK:--");
  
  // Show debug mode status
  if (debugMode) {
    u8g2.drawStr(50, 63, "DBG");
  }
  
  // Battery status
  char batteryStr[8];
  sprintf(batteryStr, "BAT:%d%%", batteryLevel);
  u8g2.drawStr(90, 63, batteryStr);
  
  u8g2.sendBuffer();
}

void drawPlatformSelect() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(5, 10, "Select Platform:");
  
  // Calculate which items to show (3 at a time)
  for (int i = 0; i < 3; i++) {
    int itemIdx = menuOffset + i;
    if (itemIdx > 3) break; // We only have 4 platform options
    
    int yPos = 20 + (i * 11);
    bool isSelected = (itemIdx == menuPosition);
    
    if (isSelected) {
      u8g2.drawBox(0, yPos - 8, 128, 10);
      u8g2.setDrawColor(0); // Inverse color for selected item
    } else {
      u8g2.setDrawColor(1);
    }
    
    // Draw icon and text
    const unsigned char* icon;
    const char* text;
    
    switch (itemIdx) {
      case ORNITHOPTER:
        icon = ornithopterIcon;
        text = "Ornithopter";
        break;
      case PLANE:
        icon = planeIcon;
        text = "Plane";
        break;
      case HELICOPTER:
        icon = helicopterIcon;
        text = "Helicopter";
        break;
      case QUADCOPTER:
        icon = quadcopterIcon;
        text = "Quadcopter";
        break;
    }
    
    u8g2.drawXBM(10, yPos - 7, 8, 8, icon);
    u8g2.drawStr(22, yPos, text);
    
    u8g2.setDrawColor(1); // Reset draw color
  }
  
  // Draw scrollbar if needed
  if (menuOffset > 0) {
    u8g2.drawTriangle(124, 15, 128, 15, 126, 12);
  }
  if (menuOffset < 1) { // 1 = max items (4) - visible items (3)
    u8g2.drawTriangle(124, 50, 128, 50, 126, 53);
  }
}

void drawTuningPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(5, 10, "Tuning:");
  
  // Draw throttle value bar
  u8g2.drawStr(5, 25, "Throttle:");
  u8g2.drawFrame(60, 18, 60, 8);
  u8g2.drawBox(60, 18, map(throttleValue, 0, 1023, 0, 60), 8);
  
  // Draw rotary status
  u8g2.drawStr(5, 40, "Tune:");
  if (rotaryEnabled) {
    u8g2.drawStr(40, 40, "ENABLED");
    char valStr[5];
    sprintf(valStr, "%d", encoderValue);
    u8g2.drawStr(90, 40, valStr);
  } else {
    u8g2.drawStr(40, 40, "LOCKED");
  }
}

void drawSettingsPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(5, 10, "Settings:");
  
  // Draw connection status with icon
  u8g2.drawStr(5, 25, "Status:");
  u8g2.drawStr(60, 25, isConnected ? "Connected" : "Disconnected");
  
  // Draw armed status
  u8g2.drawStr(5, 35, "Motors:");
  u8g2.drawStr(60, 35, isArmed ? "ARMED" : "DISARMED");
  
  // Draw GPS status
  u8g2.drawStr(5, 45, "GPS:");
  u8g2.drawStr(60, 45, isGpsLocked ? "Locked" : "No Fix");
}

void drawCalibrationPage() {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.drawStr(5, 10, "Calibration:");
  
  // Draw joystick values
  char lxStr[10], lyStr[10], rxStr[10], ryStr[10];
  sprintf(lxStr, "LX: %4d", leftX);
  sprintf(lyStr, "LY: %4d", leftY);
  sprintf(rxStr, "RX: %4d", rightX);
  sprintf(ryStr, "RY: %4d", rightY);
  
  u8g2.drawStr(5, 25, lxStr);
  u8g2.drawStr(65, 25, lyStr);
  u8g2.drawStr(5, 40, rxStr);
  u8g2.drawStr(65, 40, ryStr);
}

// Input Handling Functions
void handleJoysticks() {
  // Read raw values
  leftX = readJoystick(LEFT_JOYSTICK_X);
  leftY = readJoystick(LEFT_JOYSTICK_Y);
  rightX = readJoystick(RIGHT_JOYSTICK_X);
  rightY = readJoystick(RIGHT_JOYSTICK_Y);
  
  // Apply deadzones and mapping
  leftX = mapJoystick(leftX - leftXOffset, joystickDeadzone);
  leftY = mapJoystick(leftY - leftYOffset, joystickDeadzone);
  rightX = mapJoystick(rightX - rightXOffset, joystickDeadzone);
  rightY = mapJoystick(rightY - rightYOffset, joystickDeadzone);
  
  // Handle joystick button presses for menu selection
  static bool lastRightJoystickPress = false;
  bool rightJoystickPress = !digitalRead(RIGHT_JOYSTICK_SW); // Active low with pull-up
  
  if (rightJoystickPress != lastRightJoystickPress) {
    // Debounce press
    static unsigned long lastPressTime = 0;
    if (millis() - lastPressTime > 200) { // 200ms debounce
      lastPressTime = millis();
      
      if (rightJoystickPress) { // Only on press, not release
        if (debugMode) {
          // In debug mode, handle press for menu navigation
          if (currentState == PLATFORM_SELECT) {
            currentPlatform = (PlatformType)menuPosition;
            currentState = TUNING_PAGE;
            Serial.println("Right Joystick Press - Selected platform: " + String(menuPosition));
          } else {
            currentState = (ControllerState)((currentState + 1) % 4);
            Serial.println("Right Joystick Press - Next page");
          }
          triggerVibration(100); // Longer feedback for selection
        }
      }
      lastRightJoystickPress = rightJoystickPress;
    }
  }
  
  if (debugMode) {
    // In debug mode, use right joystick for menu navigation
    // Handle joystick menu navigation with timing to avoid too rapid changes
    if (millis() - lastJoystickMoveTime > joystickMoveDelay) {
      // Right joystick Y axis controls menu up/down
      if (rightY > 250) { // Significant up movement
        if (currentState == PLATFORM_SELECT) {
          changeDroneType(-1); // Move up in menu
          Serial.println("Joystick - Menu Up");
        } else {
          // In other pages, increase value
          encoderValue++;
          if (encoderValue > 100) encoderValue = 100;
          Serial.println("Joystick - Value Up: " + String(encoderValue));
        }
        lastJoystickMoveTime = millis();
        triggerVibration(30); // Short feedback
      }
      else if (rightY < -250) { // Significant down movement
        if (currentState == PLATFORM_SELECT) {
          changeDroneType(1); // Move down in menu
          Serial.println("Joystick - Menu Down");
        } else {
          // In other pages, decrease value
          encoderValue--;
          if (encoderValue < 0) encoderValue = 0; 
          Serial.println("Joystick - Value Down: " + String(encoderValue));
        }
        lastJoystickMoveTime = millis();
        triggerVibration(30); // Short feedback
      }
      
      // Removed right joystick X-axis selection code since we now use the press button
    }
    
    // In debug mode, left joystick is disabled
    // Clear left joystick values to prevent aircraft control
    leftX = 0;
    leftY = 0;
  }
  else {
    // Normal mode - use left Y as throttle
    throttleValue = map(leftY, -512, 512, 0, 1023);
    if (throttleValue < 0) throttleValue = 0;
    if (throttleValue > 1023) throttleValue = 1023;
  }
}

void handleButtons() {
  // Debug switch
  static bool lastDebugState = false;
  bool debugState = !digitalRead(DEBUG_SWITCH_PIN); // Inverted because of INPUT_PULLUP
  
  if (debugState != lastDebugState) {
    lastDebugState = debugState;
    // Toggle debug mode
    debugMode = debugState;
    
    if (debugMode) {
      Serial.println("Debug mode ON - Using right joystick for menu navigation");
    } else {
      Serial.println("Debug mode OFF - Normal flight control");
    }
    
    triggerVibration(50); // Short vibration for feedback
  }
  
  // Kill switch
  static bool lastKillState = false;
  bool killState = !digitalRead(KILL_SWITCH_PIN); // Inverted because of INPUT_PULLUP
  
  if (killState != lastKillState) {
    lastKillState = killState;
    isArmed = !killState;
    
    if (killState) {
      // Motors were killed
      triggerVibration(300); // Longer vibration for important action
    }
  }
  
  // Rotary enable button - only used when not in debug mode
  if (!debugMode) {
    rotaryEnabled = !digitalRead(ROTARY_ENABLE_PIN); // Inverted because of INPUT_PULLUP
  }
}

void handleEncoder() {
  // First, let's handle rotary encoder's button press regardless of rotary status
  static bool lastRotaryPress = false;
  bool rotaryPress = !digitalRead(ROTARY_SW_PIN); // Inverted because of INPUT_PULLUP
  
  // Add debounce for button
  static unsigned long lastButtonTime = 0;
  if (rotaryPress != lastRotaryPress && millis() - lastButtonTime > 50) { // 50ms debounce
    lastButtonTime = millis();
    
    if (rotaryPress) { // Only on button press, not release
      // In platform select, confirm selection
      if (currentState == PLATFORM_SELECT) {
        currentPlatform = (PlatformType)menuPosition;
        currentState = TUNING_PAGE; // Move to tuning page
        triggerVibration(100);
      }
      // Otherwise cycle through pages
      else {
        currentState = (ControllerState)((currentState + 1) % 4);
        triggerVibration(50);
      }
    }
    lastRotaryPress = rotaryPress;
  }
  
  // Now handle the rotary part
  // For debugging - always print encoder pin states
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 500) { // Print every 500ms
    lastDebugTime = millis();
    Serial.print("CLK: ");
    Serial.print(digitalRead(ROTARY_CLK_PIN));
    Serial.print(" DT: ");
    Serial.print(digitalRead(ROTARY_DT_PIN));
    Serial.print(" ROT_EN: ");
    Serial.println(rotaryEnabled);
  }
  
  // If in platform select menu, always allow encoder without enable button
  bool allowRotary = rotaryEnabled;
  if (currentState == PLATFORM_SELECT) {
    allowRotary = true;
  }
  
  if (!allowRotary) return; // Only proceed if enabled or in platform select
  
  // Improved encoder reading with static variables for better state tracking
  static uint8_t prevState = 0;
  static unsigned long lastRotaryTime = 0;
  
  // Read both pins as a 2-bit value
  uint8_t currState = (digitalRead(ROTARY_CLK_PIN) << 1) | digitalRead(ROTARY_DT_PIN);
  
  // Only process if state changed and enough time passed (debounce)
  if (currState != prevState && millis() - lastRotaryTime > 5) { // 5ms debounce
    lastRotaryTime = millis();
    
    // Determine direction based on state transition
    // This is a more reliable method than checking individual pins
    if (currentState == PLATFORM_SELECT) {
      // In platform select, we change menu position
      if ((prevState == 0b00 && currState == 0b01) ||
          (prevState == 0b01 && currState == 0b11) ||
          (prevState == 0b11 && currState == 0b10) ||
          (prevState == 0b10 && currState == 0b00)) {
        // Clockwise - move down in menu
        changeDroneType(1);
        Serial.println("Rotary CW - Menu Down");
      } else if ((prevState == 0b00 && currState == 0b10) ||
                (prevState == 0b10 && currState == 0b11) ||
                (prevState == 0b11 && currState == 0b01) ||
                (prevState == 0b01 && currState == 0b00)) {
        // Counter-clockwise - move up in menu
        changeDroneType(-1);
        Serial.println("Rotary CCW - Menu Up");
      }
    } else {
      // In other pages, adjust encoder value if enabled
      if ((prevState == 0b00 && currState == 0b01) ||
          (prevState == 0b01 && currState == 0b11) ||
          (prevState == 0b11 && currState == 0b10) ||
          (prevState == 0b10 && currState == 0b00)) {
        // Clockwise
        encoderValue++;
        if (encoderValue > 100) encoderValue = 100;
        Serial.println("Rotary CW - Value Up: " + String(encoderValue));
      } else if ((prevState == 0b00 && currState == 0b10) ||
                (prevState == 0b10 && currState == 0b11) ||
                (prevState == 0b11 && currState == 0b01) ||
                (prevState == 0b01 && currState == 0b00)) {
        // Counter-clockwise
        encoderValue--;
        if (encoderValue < 0) encoderValue = 0;
        Serial.println("Rotary CCW - Value Down: " + String(encoderValue));
      }
      
      // Only vibrate if value actually changed
      if (encoderValue != lastRotaryValue) {
        lastRotaryValue = encoderValue;
        triggerVibration(20); // Short vibration for feedback
      }
    }
    
    prevState = currState; // Update state for next time
  }
}

void checkBattery() {
  // Battery Monitoring Connection:
  // - Create voltage divider with two resistors (e.g., 47K and 33K)
  // - Connect Battery+ to 47K resistor, other end to GPIO 36 (BATTERY_PIN)
  // - Connect junction between resistors to 33K resistor, other end to GND
  // - This scales 4.2V battery voltage to ~2.45V for ESP32's ADC input
  
  // Read battery voltage through voltage divider
  int rawValue = analogRead(BATTERY_PIN);
  float voltage = rawValue * (3.3 / 4095.0) * 2; // Adjust multiplier based on your voltage divider
  
  // Map voltage to percentage
  batteryLevel = map(voltage * 100, VBAT_MIN * 100, VBAT_MAX * 100, 0, 100);
  
  if (batteryLevel < 0) batteryLevel = 0;
  if (batteryLevel > 100) batteryLevel = 100;
  
  // Warning for low battery
  static unsigned long lastBatteryWarnTime = 0;
  if (batteryLevel < 15 && millis() - lastBatteryWarnTime > 30000) { // Every 30 seconds
    triggerVibration(500); // Long vibration for low battery
    lastBatteryWarnTime = millis();
  }
}

void updateLEDs() {
  // Status LEDs
  digitalWrite(STATUS_LED_GREEN, isConnected);
  digitalWrite(STATUS_LED_RED, !isArmed); // Red when motors are killed
  
  // Battery LEDs
  digitalWrite(BATTERY_LED_1, batteryLevel > 30);
  digitalWrite(BATTERY_LED_2, batteryLevel > 60);
  digitalWrite(BATTERY_LED_3, batteryLevel > 90);
}

void triggerVibration(int duration) {
  digitalWrite(VIBRATION_PIN, HIGH);
  delay(duration);
  digitalWrite(VIBRATION_PIN, LOW);
}

void changeDroneType(int direction) {
  menuPosition += direction;
  
  // Wrap around
  if (menuPosition < 0) menuPosition = 3;
  if (menuPosition > 3) menuPosition = 0;
  
  // Adjust menu offset to ensure selected item is visible
  if (menuPosition < menuOffset) menuOffset = menuPosition;
  if (menuPosition >= menuOffset + 3) menuOffset = menuPosition - 2;
  
  triggerVibration(30); // Provide feedback
}

int mapJoystick(int value, int deadzone) {
  // Apply deadzone
  if (abs(value) < deadzone) {
    return 0;
  }
  
  // Map to -512 to 512 range with deadzone applied
  if (value > 0) {
    return map(value, deadzone, 2048, 0, 512);
  } else {
    return map(value, -2048, -deadzone, -512, 0);
  }
}

int readJoystick(int pin) {
  return analogRead(pin);
}

// Communication Functions
void sendControlData() {
  // Create control packet
  uint8_t controlData[16] = {0}; // 16 bytes for AES block size
  
  // Pack data
  controlData[0] = 0xA5; // Header
  controlData[1] = (uint8_t)currentPlatform;
  
  // Pack joystick values (2 bytes each)
  controlData[2] = (leftX >> 8) & 0xFF;
  controlData[3] = leftX & 0xFF;
  controlData[4] = (leftY >> 8) & 0xFF;
  controlData[5] = leftY & 0xFF;
  controlData[6] = (rightX >> 8) & 0xFF;
  controlData[7] = rightX & 0xFF;
  controlData[8] = (rightY >> 8) & 0xFF;
  controlData[9] = rightY & 0xFF;
  
  // Pack other controls
  controlData[10] = encoderValue;
  controlData[11] = isArmed ? 0x01 : 0x00;
  
  // Checksum
  uint8_t checksum = 0;
  for (int i = 0; i < 15; i++) {
    checksum ^= controlData[i]; // Simple XOR checksum
  }
  controlData[15] = checksum;
  
  // For debugging - print joystick values to Serial
  Serial.print("Sending - LX: ");
  Serial.print(leftX);
  Serial.print(" LY: ");
  Serial.print(leftY);
  Serial.print(" RX: ");
  Serial.print(rightX);
  Serial.print(" RY: ");
  Serial.print(rightY);
  Serial.print(" Armed: ");
  Serial.println(isArmed ? "YES" : "NO");
  
  /* Commented out during LoRa bypass testing
  // Encrypt data
  uint8_t encryptedData[16];
  encryptData(controlData, encryptedData, 16);
  
  // Send via LoRa
  LoRa.beginPacket();
  LoRa.write(encryptedData, 16);
  LoRa.endPacket();
  */
}

void receiveTelemData() {
  // Simulate receiving telemetry data for testing
  static unsigned long lastSimTime = 0;
  static bool simulatedGps = false;
  static int simulatedBattery = 85;
  
  // Simulate received packet every 1 second
  if (millis() - lastSimTime > 1000) {
    lastSimTime = millis();
    
    // Update connection status (always connected in test mode)
    isConnected = true;
    
    // Simulate GPS lock toggle every 10 seconds
    if ((millis() / 10000) % 2 == 0) {
      simulatedGps = !simulatedGps;
    }
    isGpsLocked = simulatedGps;
    
    // Simulate battery slowly decreasing
    if (random(100) < 5) { // 5% chance to decrease battery each update
      simulatedBattery--;
      if (simulatedBattery < 10) {
        simulatedBattery = 85; // Reset for testing
      }
    }
    
    // For debugging
    Serial.print("Simulated telemetry - GPS Lock: ");
    Serial.print(isGpsLocked ? "YES" : "NO");
    Serial.print(" Battery: ");
    Serial.println(simulatedBattery);
  }
  
  /* Original LoRa code commented out for testing
  // Check if there's incoming data
  int packetSize = LoRa.parsePacket();
  if (packetSize == 16) { // Expected size for telemetry
    uint8_t encryptedData[16] = {0};
    uint8_t decryptedData[16] = {0};
    
    // Read the data
    for (int i = 0; i < 16; i++) {
      if (LoRa.available()) {
        encryptedData[i] = LoRa.read();
      }
    }
    
    // Decrypt the data
    decryptData(encryptedData, decryptedData, 16);
    
    // Verify header
    if (decryptedData[0] == 0x5A) {
      // Update connection status
      isConnected = true;
      
      // Extract telemetry data
      isGpsLocked = (decryptedData[1] & 0x01) > 0;
      
      // Update battery level from drone data
      int droneBattery = decryptedData[2];
      
      // More telemetry data can be extracted here...
    }
    
    // Connection timestamp
    static unsigned long lastConnTime = 0;
    lastConnTime = millis();
  }
  
  // Check for connection timeout
  static unsigned long lastCheckTime = 0;
  if (millis() - lastCheckTime > 1000) { // Check every second
    lastCheckTime = millis();
    
    // If no data received for 3 seconds, consider disconnected
    static unsigned long lastConnTime = 0;
    if (isConnected && millis() - lastConnTime > 3000) {
      isConnected = false;
      triggerVibration(200); // Alert user to connection loss
    }
  }
  */
}

// Encryption/Decryption Functions
void encryptData(uint8_t* input, uint8_t* output, size_t length) {
  // For AES-128, length must be a multiple of 16 bytes
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, input, output);
}

void decryptData(uint8_t* input, uint8_t* output, size_t length) {
  // AES context for decryption
  mbedtls_aes_context aes_dec;
  mbedtls_aes_init(&aes_dec);
  mbedtls_aes_setkey_dec(&aes_dec, aes_key, 128);
  
  // Decrypt
  mbedtls_aes_crypt_ecb(&aes_dec, MBEDTLS_AES_DECRYPT, input, output);
  
  // Clean up
  mbedtls_aes_free(&aes_dec);
}