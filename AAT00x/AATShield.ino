//-------------------------------------------------------
//| NVE Arduino Shield Angle/Rotation sensor demo
//| AAT-10 TMR SPI angle sensor
//| Author - NVE Corporation, Sam Weber
//| 9/9/2024
//|
//| Compatable boards: AAT001-10E-EVB01, AAT003-10E-EVB01
//|                    AAT006-10E-EVB01, AAT009-10E-EVB01
//| 
//| Insert an AAT00x-10E-EVB01 breakout board
//| into the Shield edge connector.
//| Attach the magnet fixture.
//| Turn the magnet to see
//| the sensor’s functionality and precision.
//| 
//| Sensor connections: 
//| SIN A3; COS A1;
//-------------------------------------------------------

#include <Adafruit_IS31FL3741.h>

Adafruit_IS31FL3741_QT ledmatrix;
TwoWire *i2c = &Wire; // I2C interface for LED matrix

// Constants and Definitions
const int numLeds = 60;  // Total number of LEDs

// Struct to define LED coordinates as (x, y)
struct LEDCoordinate {
  int x;
  int y;
};

// State and Debounce Variables
volatile int toggleStateAAT = 0;         // Toggle between functions
volatile unsigned long lastDebounceTime = 0; 
const unsigned long debounceDelay = 50;   // Debounce time for button press
const int buttonPin = 2;                  // Pin for the interrupt (Digital Pin 2)

// Timing Variables
unsigned long lastUpdateTime = 0;
unsigned long updateInterval = 0;
unsigned long previousMillis = 0;
const unsigned long interval = 1000;      // 1 second interval for stability check

// LED Control Variables
int currentLEDIndex = 0;
int previousLEDs[5] = {-1, -1, -1, -1, -1}; // Store the last 5 LEDs for comet effect
float previousAngle = 0;
float currentAngle = 0;
bool firstIterationToggle = true;

// LED Matrix Coordinates
LEDCoordinate ledCoordinates[numLeds] = {
    {0, 8}, {0, 7}, {0, 6}, {0, 5}, {0, 4}, {0, 3}, {0, 2}, {0, 1}, {0, 0},
    {1, 8}, {1, 7}, {1, 6}, {1, 5}, {1, 4}, {1, 3}, {1, 2}, {1, 1}, {1, 0},
    {2, 8}, {2, 7}, {2, 6}, {2, 5}, {2, 4}, {2, 3}, {2, 2}, {2, 1}, {2, 0},
    {3, 8}, {3, 7}, {3, 6}, {3, 5}, {3, 4}, {3, 3}, {3, 2}, {3, 1}, {3, 0},
    {4, 8}, {4, 7}, {4, 6}, {4, 5}, {4, 4}, {4, 3}, {4, 2}, {4, 1}, {4, 0},
    {5, 8}, {5, 7}, {5, 6}, {5, 5}, {5, 4}, {5, 3}, {5, 2}, {5, 1}, {5, 0},
    {6, 8}, {6, 7}, {6, 6}, {6, 5}, {6, 4}, {6, 3}
};

// -------------------------------------------------------
// Setup: Initialize serial communication, button interrupt, and LED matrix
// -------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit QT RGB Matrix LED Walker");

  // Setup interrupt for button press
  attachInterrupt(digitalPinToInterrupt(buttonPin), toggleStateISR, FALLING);

  // Initialize LED matrix
  if (!ledmatrix.begin(IS3741_ADDR_DEFAULT, i2c)) {
    Serial.println("IS41 not found");
    while (1);  // Halt if matrix initialization fails
  }
  Serial.println("IS41 found!");
  i2c->setClock(800000);    // Set I2C clock speed

  // Configure LED settings
  ledmatrix.setLEDscaling(0xFF);
  ledmatrix.setGlobalCurrent(0xFF);
  ledmatrix.enable(true);
  Serial.print("Global current set to: ");
  Serial.println(ledmatrix.getGlobalCurrent());

  // Initialize analog pins for sensor inputs
  initAnalogPins();
}

// -------------------------------------------------------
// Main Loop: Executes different functions based on toggleStateAAT
// -------------------------------------------------------
void loop() {  
  if (firstIterationToggle) {
    turnOffAllQuadrants();      
    firstIterationToggle = false;  // Reset after the first iteration
  }
  
  switch (toggleStateAAT) {
    case 0:
      function1Flashing();        // Function to display the current angle on the LED panel
      break;
    case 1:
      function2CountCycles();     // Count full revolutions and change color on each cycle
      break;
    case 2:
      function3();                // Function with precision position and color changes based on speed
      break;
    case 3:
      function4();                // Speedometer display with LED color transitions
      break;
  }
}

// -------------------------------------------------------
// Interrupt Service Routine: Handles button presses to toggle between functions
// -------------------------------------------------------
void toggleStateISR() {
  unsigned long currentTime = millis(); 
  if ((currentTime - lastDebounceTime) > debounceDelay) {  // Debounce check
    lastDebounceTime = currentTime;
    firstIterationToggle = true;  // Reset on state change
    
    // Cycle through 4 states (0-3)
    toggleStateAAT = (toggleStateAAT + 1) % 4;

    // Debugging output
    Serial.print("toggleState updated to: ");
    Serial.println(toggleStateAAT);
  }
}

// -------------------------------------------------------
// Function 1: Displays current angle with flashing effect based on stability
// -------------------------------------------------------
void function1Flashing() {
  static uint16_t hue_offset = 0;
  static int previousIndex = -1;
  static float previousAngle = 0;

  float currentAngle = getSmoothedAngle(50); // Smoothed sensor angle
  int ledIndex = degreeToLED(currentAngle);  // Convert angle to LED index

  // Update hue based on movement direction
  hue_offset = updateHueOffset(currentAngle, previousAngle, hue_offset);

  // Stability check to flash LEDs if unchanged for a certain interval
  handleStabilityAndFlashing(ledIndex, previousIndex);

  // Update LED display with new colors
  updateLEDDisplay(ledIndex, hue_offset, previousIndex);

  // Store previous values for the next iteration
  previousIndex = ledIndex;
  previousAngle = currentAngle;

  delay(15);  // Adjust delay for flashing speed
}

// -------------------------------------------------------
// Function 2: Counts full revolutions and changes LED colors accordingly
// -------------------------------------------------------
void function2CountCycles() {
  static int previousIndex = -1;
  static float previousAngle = 0;
  static uint16_t backgroundHue = 0;  // Hue value for background color

  float currentAngle = getSmoothedAngle(50);  // Smoothed sensor angle
  int ledIndex = degreeToLED(currentAngle);   // Get corresponding LED index

  // Check for a full clockwise revolution (angle crosses from 360 to 0)
  if (previousAngle > 335 && currentAngle < 25) { 
    backgroundHue += 4368;  // Increment hue value for clockwise revolution
    if (backgroundHue >= 65536) {
      backgroundHue = 0;    // Wrap hue back to 0 if it exceeds maximum
    }
    Serial.println("Full revolution clockwise! Hue incremented.");
  }
  // Check for a full counterclockwise revolution (angle crosses from 0 to 360)
  else if (previousAngle < 25 && currentAngle > 335) {
    backgroundHue = (backgroundHue < 4368) ? 65536 - (4368 - backgroundHue) : backgroundHue - 4368;
    Serial.println("Full revolution counterclockwise! Hue decremented.");
  }

  // Set background color based on hue
  uint16_t backgroundColor = ledmatrix.color565(ledmatrix.ColorHSV(backgroundHue));

  // Turn on all LEDs with background color, except the one corresponding to the current angle
  for (int i = 0; i < numLeds; i++) {
    if (i == ledIndex) {
      ledmatrix.drawPixel(ledCoordinates[i].x, ledCoordinates[i].y, ledmatrix.color565(255, 255, 255));  // Current angle LED: white
    } else {
      ledmatrix.drawPixel(ledCoordinates[i].x, ledCoordinates[i].y, backgroundColor);  // Background color for other LEDs
    }
  }

  // Update the display
  ledmatrix.show();

  // Store previous values for the next iteration
  previousIndex = ledIndex;
  previousAngle = currentAngle;

  delay(15);  // Adjust delay for revolution speed
}

// -------------------------------------------------------
// Function 3: Precision position display with color changes based on speed
// -------------------------------------------------------
void function3() {
  float currentAngle = getSmoothedAngle(50);  // Get the current angle
  int ledIndex = degreeToLED(currentAngle);   // Convert angle to LED index

  if (ledIndex != currentLEDIndex) {  // Only update if the LED index has changed
    unsigned long currentTime = millis();
    updateInterval = currentTime - lastUpdateTime;  // Time since last update
    lastUpdateTime = currentTime;

    // Update the comet trail effect with speed-based color
    updateCometTrail(ledIndex);

    // Update current LED index for the next loop iteration
    currentLEDIndex = ledIndex;
  }
}

// -------------------------------------------------------
// Function 4: Speedometer effect using LED transitions from green to red
// -------------------------------------------------------
void function4() {
  currentAngle = getSmoothedAngle(50);   // Get the current angle
  float angleDifference = abs(currentAngle - previousAngle);  // Calculate angle difference (speed)

  int ledCount = determineLEDCount(angleDifference);  // Determine how many LEDs to light based on speed

  // Light up the speedometer with transitioning colors
  lightUpSpeedometer(ledCount);

  previousAngle = currentAngle;  // Update the previous angle for the next iteration

  delay(15);  // Adjust delay for speedometer refresh rate
}

// -------------------------------------------------------
// Utility Functions: These functions handle supporting tasks such as calculating angles, updating displays, etc.
// -------------------------------------------------------

// Initialize analog pins for sensor input
void initAnalogPins() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}

// Get the smoothed angle from the analog pins (averaging over multiple samples)
float getSmoothedAngle(int numSamples) {
  float sum = 0;
  float lastAngle = getAngle();  // Initialize with the first reading
  float currentAngle;

  for (int i = 0; i < numSamples; i++) {
    currentAngle = getAngle();

    // Handle the 360 to 0 wraparound
    if (lastAngle > 300 && currentAngle < 60) {
      // Adjust for wraparound: treat the current angle as if it were above 360
      currentAngle += 360;
    } else if (lastAngle < 60 && currentAngle > 300) {
      // Adjust for wraparound: treat the current angle as if it were below 0
      currentAngle -= 360;
    }

    sum += currentAngle;
    lastAngle = currentAngle;  // Update lastAngle for the next comparison
  }

  float averageAngle = sum / numSamples;

  // Normalize the averaged angle back to the 0-360 range
  if (averageAngle < 0) {
    averageAngle += 360;
  } else if (averageAngle >= 360) {
    averageAngle -= 360;
  }

  return averageAngle;
}


// Get the current angle from the analog pins (based on sensor data)
float getAngle() {
  // Read the analog values from the sensor
  double cosine = (analogRead(A1) - 512.0) / 512.0;  // Normalize to range -1 to 1
  double sine = (analogRead(A3) - 512.0) / 512.0;    // Normalize to range -1 to 1
  
  // Calculate the angle using atan2 (result is in radians, converted to degrees)
  return (atan2(sine, cosine) * 180.0 / 3.14159) + 180.0;  // Shift angle to 0-360 degrees
}

// Convert an angle (degrees) to the corresponding LED index
int degreeToLED(float degree) {
  degree = fmod(degree, 360);  // Normalize angle to 0-359 degrees
  if (degree < 0) degree += 360;
  return degree / 6;  // 360 degrees / 60 LEDs = 6 degrees per LED
}

// Update hue offset based on rotation direction
uint16_t updateHueOffset(float currentAngle, float previousAngle, uint16_t hue_offset) {
  if (currentAngle > previousAngle) {
    hue_offset += 1092;  // Clockwise rotation
  } else if (currentAngle < previousAngle) {
    hue_offset -= 1092;  // Counterclockwise rotation
  }
  return hue_offset;
}

// Handle stability check: if LED position unchanged, trigger flashing effect
void handleStabilityAndFlashing(int ledIndex, int previousIndex) {
  if (ledIndex == previousIndex) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      Serial.println("LED position unchanged for 1 second! Flashing LEDs...");
      flashLEDs(ledIndex);  // Trigger flashing if position is stable
      previousMillis = currentMillis;  // Reset timer
    }
  } else {
    previousMillis = millis();  // Reset timer if position changed
  }
}

// Update the LED matrix display with new colors
void updateLEDDisplay(int ledIndex, uint16_t hue_offset, int previousIndex) {
  uint32_t color888 = ledmatrix.ColorHSV(hue_offset);   // Get HSV color based on hue offset
  uint16_t color565 = ledmatrix.color565(color888);     // Convert 888 color to 565 format

  ledmatrix.drawPixel(ledCoordinates[ledIndex].x, ledCoordinates[ledIndex].y, color565);  // Turn on current LED

  // Turn off previous LED if necessary
  if (previousIndex >= 0 && previousIndex != ledIndex) {
    turnOffPreviousLED(previousIndex);
  }
}

// Turn off the previous LED
void turnOffPreviousLED(int prevIndex) {
  ledmatrix.drawPixel(ledCoordinates[prevIndex].x, ledCoordinates[prevIndex].y, 0);  // Turn off previous LED
}

// Flash LEDs for stability effect (triggered if position is unchanged)
void flashLEDs(int ledIndex) {
  int flashDelay = 15;  // Delay between flashes

  for (int j = 0; j < 10; j++) {  // Flash sequence for 10 LEDs
    int currentLedIndex = degreeToLED(getSmoothedAngle(50));  // Check if position changed during flash
    if (currentLedIndex != ledIndex) {
      Serial.println("Position changed during flash, stopping flash...");
      return;  // Stop flash if position changes
    }

    int forwardIndex = (ledIndex + 10 - j + numLeds) % numLeds;
    int backwardIndex = (ledIndex - 10 + j + numLeds) % numLeds;

    // Turn on forward and backward LEDs with green color
    ledmatrix.drawPixel(ledCoordinates[forwardIndex].x, ledCoordinates[forwardIndex].y, ledmatrix.color565(255, 0, 0));
    ledmatrix.drawPixel(ledCoordinates[backwardIndex].x, ledCoordinates[backwardIndex].y, ledmatrix.color565(255, 0, 0));
    ledmatrix.show();
    delay(flashDelay);

    // Turn off forward and backward LEDs
    ledmatrix.drawPixel(ledCoordinates[forwardIndex].x, ledCoordinates[forwardIndex].y, 0);
    ledmatrix.drawPixel(ledCoordinates[backwardIndex].x, ledCoordinates[backwardIndex].y, 0);
    ledmatrix.show();
    delay(flashDelay);
  }
}

// Turn off all LEDs
void turnOffAllQuadrants() {
  for (int i = 0; i < numLeds; i++) {
    int x = ledCoordinates[i].x;
    int y = ledCoordinates[i].y;
    ledmatrix.drawPixel(x, y, ledmatrix.color565(0, 0, 0));  // Turn off LED
  }
}

// -------------------------------------------------------
// Speedometer and Comet Effect Functions
// -------------------------------------------------------

// Update comet trail with speed-based color
void updateCometTrail(int ledIndex) {
  for (int i = 4; i > 0; i--) {
    previousLEDs[i] = previousLEDs[i - 1];  // Shift previous LED indices
  }
  previousLEDs[0] = ledIndex;  // Add current LED to the trail

  if (previousLEDs[4] != -1) {
    int x = ledCoordinates[previousLEDs[4]].x;
    int y = ledCoordinates[previousLEDs[4]].y;
    ledmatrix.drawPixel(x, y, ledmatrix.color565(0, 0, 0));  // Turn off LED at end of tail
  }

  // Determine LED color based on rotation speed
  uint16_t color = getColorBasedOnSpeed(updateInterval);

  // Draw the current LED with the calculated color
  int x = ledCoordinates[ledIndex].x;
  int y = ledCoordinates[ledIndex].y;
  ledmatrix.drawPixel(x, y, color);

  ledmatrix.show();  // Update the display
}

// Get color based on rotation speed
uint16_t getColorBasedOnSpeed(unsigned long interval) {
  const unsigned long slowThreshold = 35;
  const unsigned long mediumThreshold = 24;
  const unsigned long fastThreshold = 19;

  if (interval > slowThreshold) {
    return ledmatrix.color565(255, 0, 0);  // Green for slow speed
  } else if (interval > mediumThreshold) {
    return ledmatrix.color565(255, 0, 255);  // Yellow for medium speed
  } else {
    return ledmatrix.color565(0, 0, 255);  // Red for fast speed
  }
}

// Determine the number of LEDs to light up based on speed
int determineLEDCount(float angleDifference) {
  if (angleDifference < 12.0) {
    return map(angleDifference, 0, 12, 0, 19);  // Bottom 20 LEDs (Green to Yellow)
  } else if (angleDifference < 24.0) {
    return map(angleDifference, 12, 24, 20, 39);  // Middle 20 LEDs (Yellow to Orange)
  } else {
    return map(angleDifference, 24, 36, 40, 59);  // Top 20 LEDs (Orange to Red)
  }
}

// Light up the speedometer with transitioning colors based on speed
void lightUpSpeedometer(int ledCount) {
  turnOffAllQuadrants();  // Turn off all LEDs before lighting up new section

  for (int i = 0; i < ledCount; i++) {
    int x = ledCoordinates[i].x;
    int y = ledCoordinates[i].y;

    uint16_t color;
    if (i < 20) {
      int r = map(i, 0, 19, 0, 255);  // Transition from green to yellow
      int g = 255;
      color = ledmatrix.color565(g, 0, r);
    } else if (i < 40) {
      int r = 255;
      int g = map(i, 20, 39, 255, 165);  // Transition from yellow to orange
      color = ledmatrix.color565(g, 0, r);
    } else {
      int r = 255;
      int g = map(i, 40, 59, 165, 0);  // Transition from orange to red
      color = ledmatrix.color565(g, 0, r);
    }

    ledmatrix.drawPixel(x, y, color);  // Draw pixel with the calculated color
  }

  ledmatrix.show();  // Update the display
} 
