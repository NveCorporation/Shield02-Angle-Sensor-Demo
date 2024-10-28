//-------------------------------------------------------
//| NVE Arduino Shield Angle/Rotation sensor demo
//| ALT521-10 TMR analog rotation sensor
//| Author - NVE Corporation, Sam Weber
//| 9/9/2024
//|  
//| Compatible breakout board: ALT521-10E-EVB01
//|
//| Insert an ALT521-10E-EVB01 breakout board
//| into the Shield edge connector.
//| Attach the magnet fixture.
//| Turn the magnet to see
//| the sensor’s functionality and precision.
//| Additionally the ALT521 has a magnetic field
//| strength function represented by LED brightness
//| 
//| Sensor connections: 
//| SIN A3; COS A1
//-------------------------------------------------------

#include <Adafruit_IS31FL3741.h>

Adafruit_IS31FL3741_QT ledmatrix; // Initialize the LED matrix object
TwoWire *i2c = &Wire; // I2C interface

const int numLeds = 60; // Number of LEDs in the array

// Define an array of LED coordinates as (x, y) pairs
struct LEDCoordinate {
  int x;
  int y;
};

// Variables for button press debounce and state toggling
volatile int toggleState = 0; // Variable to toggle between functions
volatile unsigned long lastDebounceTime = 0; // To store the last debounce time
const unsigned long debounceDelay = 50; // Debounce time in milliseconds

const int buttonPin = 2; // Pin for the button interrupt (Digital Pin 2)

// Variables for controlling the state of LED effects and angles
bool firstIterationToggle = true;
unsigned long lastUpdateTime = 0;
unsigned long updateInterval = 0;
int currentLEDIndex = 0;
int previousLEDs[5] = {-1, -1, -1, -1, -1}; // Store the last 5 LEDs for comet effect
float previousAngle = 0;
float currentAngle = 0;

// Define an array of LED coordinates
LEDCoordinate ledCoordinates[numLeds] = {
    {0, 8}, {0, 7}, {0, 6}, {0, 5}, {0, 4}, {0, 3}, {0, 2}, {0, 1}, {0, 0},
    {1, 8}, {1, 7}, {1, 6}, {1, 5}, {1, 4}, {1, 3}, {1, 2}, {1, 1}, {1, 0},
    {2, 8}, {2, 7}, {2, 6}, {2, 5}, {2, 4}, {2, 3}, {2, 2}, {2, 1}, {2, 0},
    {3, 8}, {3, 7}, {3, 6}, {3, 5}, {3, 4}, {3, 3}, {3, 2}, {3, 1}, {3, 0},
    {4, 8}, {4, 7}, {4, 6}, {4, 5}, {4, 4}, {4, 3}, {4, 2}, {4, 1}, {4, 0},
    {5, 8}, {5, 7}, {5, 6}, {5, 5}, {5, 4}, {5, 3}, {5, 2}, {5, 1}, {5, 0},
    {6, 8}, {6, 7}, {6, 6}, {6, 5}, {6, 4}, {6, 3}
};

// Time tracking variables for flashing effect
unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second interval for LED stability checks

// Setup function: Initializes the LED matrix, button interrupt, and analog pins


// Initialize analog pins for sensor input
void initAnalogPins() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit QT RGB Matrix LED Walker");
  
  // Attach an interrupt to the button pin to toggle between states
  attachInterrupt(digitalPinToInterrupt(buttonPin), toggleStateISR, FALLING);
  
  // Initialize the LED matrix
  if (!ledmatrix.begin(IS3741_ADDR_DEFAULT, i2c)) {
    Serial.println("IS41 not found");
    while (1); // Stop execution if matrix initialization fails
  }
  Serial.println("IS41 found!");
  
  // Set I2C clock speed
  i2c->setClock(800000);

  // Set up LED scaling and global current for brightness control
  ledmatrix.setLEDscaling(0xFF); // Maximum LED scaling
  ledmatrix.setGlobalCurrent(0xFF); // Maximum global current (brightness)
  Serial.print("Global current set to: ");
  Serial.println(ledmatrix.getGlobalCurrent());
  
  ledmatrix.enable(true); // Enable the LED matrix
  
  // Initialize analog pins as inputs for the magnetic field sensor
  initAnalogPins();
}

// Main loop: Executes the corresponding function based on the toggle state
void loop() {  
  if (firstIterationToggle) {
    turnOffAllQuadrants(); // Turn off all LEDs after the first iteration
    firstIterationToggle = false;
  }
  
  // Check the toggle state and call the corresponding function
  switch (toggleState) {
    case 0:
      function1Flashing(); // Flashing LED based on the current angle
      break;
    case 1:
      function2CountCycles(); // Full revolution tracking and color change
      break;
    case 2:
      Serial.println("Case 2");
      function3(); // Comet trail effect based on rotation speed
      break;
    case 3:
      Serial.println("Case 3");
      function4(); // Speedometer-style display based on speed
  }

  // Adjust brightness based on the magnetic field strength
  int8_t magnetifFieldRationalized = brightnessSetter();
  ledmatrix.setLEDscaling(magnetifFieldRationalized); // Scale brightness
  ledmatrix.setGlobalCurrent(magnetifFieldRationalized * 2); // Set global brightness
}
//=======================================================================================
// Button ISR: Toggles between states when the button is pressed
void toggleStateISR() {
  unsigned long currentTime = millis(); 
  // Check if debounce delay has passed
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = currentTime; // Update debounce time
    firstIterationToggle = true; // Reset toggle flag
    
    // Cycle through the states (0-3)
    toggleState = (toggleState < 3) ? toggleState + 1 : 0;

    // Debugging output
    Serial.print("toggleState updated to: ");
    Serial.println(toggleState);
  }
}
//=======================================================================================
//Function 1 code, for the accurate angle display and flashing LED

// Function 1: Displays a flashing effect based on the current angle
void function1Flashing() {
  static uint16_t hue_offset = 0;
  static int previousIndex = -1;
  static float previousAngle = 0;

  // Get the current smoothed angle
  float currentAngle = getSmoothedAngle(25); 

  // Get the corresponding LED index based on the angle
  int ledIndex = degreeToLED(currentAngle);

  // Update the hue offset based on the direction of movement
  hue_offset = updateHueOffset(currentAngle, previousAngle, hue_offset);

  // Check for LED stability and trigger flashing if necessary
  handleStabilityAndFlashing(ledIndex, previousIndex);

  // Update the LED colors and display
  updateLEDDisplay(ledIndex, hue_offset, previousIndex);

  // Update previous values for the next loop iteration
  previousIndex = ledIndex;
  previousAngle = currentAngle;

  delay(15); // Control speed of LED flashing
}


//=======================================================================================
//Function 2, Color change every revolution

// Function 2: Changes the background color of the entire LED matrix on each full revolution
void function2CountCycles() {
  static int previousIndex = -1;
  static float previousAngle = 0;
  static uint16_t backgroundHue = 0;  // Hue value for background color

  // Get the current smoothed angle
  float currentAngle = getSmoothedAngle(50); 

  // Get the corresponding LED index
  int ledIndex = degreeToLED(currentAngle);

  // Check for a full revolution (clockwise)
  if (previousAngle > 335 && currentAngle < 25) { 
    backgroundHue += 4368; // Increment hue value
    if (backgroundHue >= 65536) {
      backgroundHue = 0; // Wrap around hue value
    }
    Serial.println("Full revolution clockwise! Hue incremented.");
  }
  // Check for a full revolution (counterclockwise)
  else if (previousAngle < 25 && currentAngle > 335) {
    backgroundHue = (backgroundHue < 4368) ? 65536 - (4368 - backgroundHue) : backgroundHue - 4368;
    Serial.println("Full revolution counterclockwise! Hue decremented.");
  }

  // Set the background color for the LED matrix
  uint16_t backgroundColor = ledmatrix.color565(ledmatrix.ColorHSV(backgroundHue));

  // Set each LED to the background color except the one corresponding to the current angle
  for (int i = 0; i < numLeds; i++) {
    if (i == ledIndex) {
      ledmatrix.drawPixel(ledCoordinates[i].x, ledCoordinates[i].y, ledmatrix.color565(255, 255, 255)); // Set current angle LED to white
    } else {
      ledmatrix.drawPixel(ledCoordinates[i].x, ledCoordinates[i].y, backgroundColor); // Set background color for other LEDs
    }
  }

  ledmatrix.show(); // Update the LED matrix display

  // Update previous values for the next loop iteration
  previousIndex = ledIndex;
  previousAngle = currentAngle;

  delay(15); // Control speed of LED color change
}

// Update the hue offset based on the movement direction
uint16_t updateHueOffset(float currentAngle, float previousAngle, uint16_t hue_offset) {
  if (currentAngle > previousAngle) {
    hue_offset += 1092;  // Increase hue for clockwise movement
  } else if (currentAngle < previousAngle) {
    hue_offset -= 1092;  // Decrease hue for counterclockwise movement
  }
  return hue_offset;
}

// Check if the LED index is stable, and flash the LEDs if it hasn't changed for a while
void handleStabilityAndFlashing(int ledIndex, int previousIndex) {
  if (ledIndex == previousIndex) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      Serial.println("LED position unchanged for 3 seconds! Flashing LEDs...");
      flashLEDs(ledIndex); // Flash LEDs if no movement for 3 seconds
      previousMillis = currentMillis; // Reset the timer
    }
  } else {
    previousMillis = millis(); // Reset timer if position changed
  }
}

// Flash the LEDs for the stability effect (triggered if position is unchanged)
void flashLEDs(int ledIndex) {
  int flashDelay = 15; // Delay between flashes

  for (int j = 0; j < 10; j++) {  // Flash sequence for 10 LEDs
    int currentLedIndex = degreeToLED(getSmoothedAngle(50));  // Check if position has changed during flash
    if (currentLedIndex != ledIndex) {
      Serial.println("Position changed during flash, stopping flash...");
      return;  // Stop flashing if the position has changed
    }

    int forwardIndex = (ledIndex + 10 - j + numLeds) % numLeds; // Forward LED index
    int backwardIndex = (ledIndex - 10 + j + numLeds) % numLeds; // Backward LED index

    // Turn on the forward and backward LEDs with red color
    ledmatrix.drawPixel(ledCoordinates[forwardIndex].x, ledCoordinates[forwardIndex].y, ledmatrix.color565(255, 0, 0));
    ledmatrix.drawPixel(ledCoordinates[backwardIndex].x, ledCoordinates[backwardIndex].y, ledmatrix.color565(255, 0, 0));
    ledmatrix.show(); // Update the LED matrix
    delay(flashDelay);

    // Turn off the forward and backward LEDs
    ledmatrix.drawPixel(ledCoordinates[forwardIndex].x, ledCoordinates[forwardIndex].y, 0);
    ledmatrix.drawPixel(ledCoordinates[backwardIndex].x, ledCoordinates[backwardIndex].y, 0);
    ledmatrix.show(); // Update the LED matrix
    delay(flashDelay);
  }
}

//=======================================================================================
//Function 3 methods for the comet trail, changes colors based on speed

// Function 3: Comet trail effect based on rotation speed
void function3() {
  // Get the current angle
  float currentAngle = getSmoothedAngle(50); 

  // Calculate the LED index based on the angle
  int ledIndex = degreeToLED(currentAngle);

  // Only update if the LED index has changed
  if (ledIndex != currentLEDIndex) {
    // Calculate the time since the last update
    unsigned long currentTime = millis();
    updateInterval = currentTime - lastUpdateTime;
    lastUpdateTime = currentTime;

    // Update the comet trail with speed-based color
    updateCometTrail(ledIndex);

    // Update the current LED index for the next loop
    currentLEDIndex = ledIndex;
  }
}

// Update the comet trail with speed-based color
void updateCometTrail(int ledIndex) {
  // Shift the previous LED indices to make room for the new one
  for (int i = 4; i > 0; i--) {
    previousLEDs[i] = previousLEDs[i - 1];
  }

  // Add the current LED index to the trail
  previousLEDs[0] = ledIndex;

  // Turn off the LED at the end of the trail
  if (previousLEDs[4] != -1) {
    ledmatrix.drawPixel(ledCoordinates[previousLEDs[4]].x, ledCoordinates[previousLEDs[4]].y, 0); // Turn off the LED
  }

  // Get the color based on the speed of rotation
  uint16_t color = getColorBasedOnSpeed(updateInterval);

  // Draw the new LED with the calculated color
  ledmatrix.drawPixel(ledCoordinates[ledIndex].x, ledCoordinates[ledIndex].y, color);
  ledmatrix.show(); // Update the display
}


// Function 4: Speedometer effect using color transitions
void function4() {
  currentAngle = getSmoothedAngle(50);   // Get the current angle
  float angleDifference = abs(currentAngle - previousAngle);  // Calculate the angle difference (speed)

  int ledCount = determineLEDCount(angleDifference);  // Determine how many LEDs to light up based on speed

  // Light up the LEDs with transitioning colors
  lightUpSpeedometer(ledCount);

  previousAngle = currentAngle;  // Update the previous angle for the next loop

  delay(15); // Adjust delay for speedometer refresh rate
}

//=======================================================================================
//Speedometer code
// Light up the speedometer LEDs with transitioning colors
void lightUpSpeedometer(int ledCount) {
  turnOffAllQuadrants(); // Turn off all LEDs before lighting up the speedometer

  for (int i = 0; i < ledCount; i++) {
    int x = ledCoordinates[i].x;
    int y = ledCoordinates[i].y;

    // Transition colors based on the position in the array
    uint16_t color;
    if (i < 20) {
      int r = map(i, 0, 19, 0, 255); // Transition from green to yellow
      int g = 255;
      color = ledmatrix.color565(g, 0, r);
    } else if (i < 40) {
      int r = 255;
      int g = map(i, 20, 39, 255, 165); // Transition from yellow to orange
      color = ledmatrix.color565(g, 0, r);
    } else {
      int r = 255;
      int g = map(i, 40, 59, 165, 0); // Transition from orange to red
      color = ledmatrix.color565(g, 0, r);
    }

    ledmatrix.drawPixel(x, y, color); // Draw the pixel with the calculated color
  }

  ledmatrix.show(); // Update the LED matrix display
}

// Get the color based on the speed of rotation
uint16_t getColorBasedOnSpeed(unsigned long interval) {
  // Define speed thresholds (in milliseconds)
  const unsigned long slowThreshold = 35;
  const unsigned long mediumThreshold = 24;
  const unsigned long fastThreshold = 19;

  // Return color based on speed thresholds
  if (interval > slowThreshold) {
    return ledmatrix.color565(255, 0, 0);  // Green for slow speed
  } else if (interval > mediumThreshold) {
    return ledmatrix.color565(255, 0, 255);  // Yellow for medium speed
  } else {
    return ledmatrix.color565(0, 0, 255);  // Red for fast speed
  }
}
//=======================================================================================
//Angle calculation section

// Calculate the hypotenuse (magnitude) of the magnetic field vector
float calculateHypotenuse(float sinValue, float cosValue) {
  return sqrt(sinValue * sinValue + cosValue * cosValue); // Use Pythagorean theorem to calculate hypotenuse
}

// Set the brightness based on the strength of the magnetic field
int brightnessSetter() {
  // Calculate the magnetic field strength as the hypotenuse of the sine and cosine readings
  float c = calculateHypotenuse(analogRead(1), analogRead(3));

  // Map the calculated value to a brightness level
  int brightness = (c - 500) / 4;
  
  // Ensure the brightness does not go below a minimum threshold
  if (brightness <= 20) {
    brightness = 20;
  }

  Serial.println(brightness); // Print the brightness value for debugging
  return brightness;
}

// Get the current angle from the analog pins (based on sensor data)
float getAngle() {
  // Read the analog values from the sensor
  double cosine = (analogRead(A1) - 512.0) / 512.0;  // Normalize to range -1 to 1
  double sine = (analogRead(A3) - 512.0) / 512.0;    // Normalize to range -1 to 1
  
  // Calculate the angle using atan2 (result is in radians, converted to degrees)
  return (atan2(sine, cosine) * 180.0 / 3.14159) + 180.0;  // Shift angle to 0-360 degrees
}

// Convert an angle in degrees to the corresponding LED index
int degreeToLED(float degree) {
  degree = fmod(degree, 360);  // Normalize degree to 0-359
  if (degree < 0) degree += 360;
  return degree / 6;  // Map 360 degrees to 60 LEDs (6 degrees per LED)
}

// Determine how many LEDs to light up based on the speed (angle difference)
int determineLEDCount(float angleDifference) {
  if (angleDifference < 12.0) {
    return map(angleDifference, 0, 12, 0, 19); // Bottom 20 LEDs (Green to Yellow)
  } else if (angleDifference < 24.0) {
    return map(angleDifference, 12, 24, 20, 39); // Middle 20 LEDs (Yellow to Orange)
  } else {
    return map(angleDifference, 24, 36, 40, 59); // Top 20 LEDs (Orange to Red)
  }
}

// Get the smoothed angle based on the sensor readings, averaging over multiple samples
float getSmoothedAngle(int numSamples) {
  float sum = 0;
  float lastAngle = getAngle();  // Initialize with the first reading
  float currentAngle;

  for (int i = 0; i < numSamples; i++) {
    currentAngle = getAngle();

    // Handle the 360 to 0 wraparound for angle transition
    if (lastAngle > 300 && currentAngle < 60) {
      currentAngle += 360; // Adjust wraparound to handle transition smoothly
    } else if (lastAngle < 60 && currentAngle > 300) {
      currentAngle -= 360; // Adjust wraparound for transition back to 0
    }

    sum += currentAngle;
    lastAngle = currentAngle; // Update for the next comparison
  }

  float averageAngle = sum / numSamples; // Average angle over all samples

  // Normalize the averaged angle back to the 0-360 range
  if (averageAngle < 0) {
    averageAngle += 360;
  } else if (averageAngle >= 360) {
    averageAngle -= 360;
  }

  return averageAngle; // Return the smoothed angle
}
//=======================================================================================
//Section dedicated to handling the LED's 

// Turn off all LEDs in the matrix
void turnOffAllQuadrants() {
  for (int i = 0; i < numLeds; i++) {
    ledmatrix.drawPixel(ledCoordinates[i].x, ledCoordinates[i].y, ledmatrix.color565(0, 0, 0)); // Turn off LED
  }
}

// Update the LED display with the new color based on the angle and hue
void updateLEDDisplay(int ledIndex, uint16_t hue_offset, int previousIndex) {
  uint32_t color888 = ledmatrix.ColorHSV(hue_offset);   // Get color based on hue offset
  uint16_t color565 = ledmatrix.color565(color888);     // Convert 888 color to 565 format

  ledmatrix.drawPixel(ledCoordinates[ledIndex].x, ledCoordinates[ledIndex].y, color565);  // Turn on the current LED

  // Turn off the previous LED if it was different
  if (previousIndex >= 0 && previousIndex != ledIndex) {
    turnOffPreviousLED(previousIndex);
  }
}

// Turn off the previous LED
void turnOffPreviousLED(int prevIndex) {
  ledmatrix.drawPixel(ledCoordinates[prevIndex].x, ledCoordinates[prevIndex].y, 0);  // Turn off previous LED
}
//=======================================================================================
