#include <Adafruit_IS31FL3741.h>

Adafruit_IS31FL3741_QT ledmatrix;
TwoWire *i2c = &Wire; // I2C interface

const int numLeds = 60; // Number of LEDs in the array

// Define your array of LED coordinates as ordered pairs (x, y)
struct LEDCoordinate {
  int x;
  int y;
};

volatile int toggleState = 0; // Variable to toggle between true and false
volatile unsigned long lastDebounceTime = 0; // To store the last debounce time
const unsigned long debounceDelay = 50; // Debounce time in milliseconds

const int buttonPin = 2; // Pin for the interrupt (Digital Pin 2)

bool firstIterationToggle = true;
unsigned long lastUpdateTime = 0;
unsigned long updateInterval = 0;
int currentLEDIndex = 0;
int previousLEDs[5] = {-1, -1, -1, -1, -1}; // Store the last 5 LEDs
float previousAngle = 0;
float currentAngle = 0;


LEDCoordinate ledCoordinates[numLeds] = {
    {0, 8}, {0, 7}, {0, 6}, {0, 5}, {0, 4}, {0, 3}, {0, 2}, {0, 1}, {0, 0},
    {1, 8}, {1, 7}, {1, 6}, {1, 5}, {1, 4}, {1, 3}, {1, 2}, {1, 1}, {1, 0},
    {2, 8}, {2, 7}, {2, 6}, {2, 5}, {2, 4}, {2, 3}, {2, 2}, {2, 1}, {2, 0},
    {3, 8}, {3, 7}, {3, 6}, {3, 5}, {3, 4}, {3, 3}, {3, 2}, {3, 1}, {3, 0},
    {4, 8}, {4, 7}, {4, 6}, {4, 5}, {4, 4}, {4, 3}, {4, 2}, {4, 1}, {4, 0},
    {5, 8}, {5, 7}, {5, 6}, {5, 5}, {5, 4}, {5, 3}, {5, 2}, {5, 1}, {5, 0},
    {6, 8}, {6, 7}, {6, 6}, {6, 5}, {6, 4}, {6, 3}
};

// Time tracking variables
unsigned long previousMillis = 0;
const unsigned long interval = 1000;

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit QT RGB Matrix LED Walker");
  attachInterrupt(digitalPinToInterrupt(buttonPin), toggleStateISR, FALLING);
  if (!ledmatrix.begin(IS3741_ADDR_DEFAULT, i2c)) {
    Serial.println("IS41 not found");
    while (1);
  }
  Serial.println("IS41 found!");
  i2c->setClock(800000);

  ledmatrix.setLEDscaling(0xFF);
  ledmatrix.setGlobalCurrent(0xFF);
  Serial.print("Global current set to: ");
  Serial.println(ledmatrix.getGlobalCurrent());
  ledmatrix.enable(true);
  
  // Initialize analog pins as inputs
  initAnalogPins();
}

void loop() {  
  if (firstIterationToggle) {
    turnOffAllQuadrants();      
    firstIterationToggle = false; // Set to false after the first iteration
  }
  
  switch (toggleState) {
    case 0:
      function1Flashing();
      break;
    case 1:
      function2CountCycles();
      break;
    case 2:
    Serial.println("Case 2");
      function3(); // Placeholder for the third function
      break;
    case 3:
    Serial.println("Case 3");
    function4();
  }
  int8_t magnetifFieldRationalized = brightnessSetter();
  ledmatrix.setLEDscaling(magnetifFieldRationalized);
  ledmatrix.setGlobalCurrent(magnetifFieldRationalized * 2);
}

void toggleStateISR() {
  unsigned long currentTime = millis(); 
  // Check if the debounce time has passed
  if ((currentTime - lastDebounceTime) > debounceDelay) {
    lastDebounceTime = currentTime; // Update the last debounce time
    firstIterationToggle = true; // Reset on state change
    
    // Cycle through the states manually
    if(toggleState <3){
      toggleState = toggleState + 1;
    }else{
      toggleState = 0;
    }

    // Debugging output
    Serial.print("toggleState updated to: ");
    Serial.println(toggleState);
  }
}



void function1Flashing(){
    static uint16_t hue_offset = 0;
  static int previousIndex = -1;
  static float previousAngle = 0;

  // Get the current smoothed angle
  float currentAngle = getSmoothedAngle(50); 

  // Get the corresponding LED index
  int ledIndex = degreeToLED(currentAngle);

  // Update the hue offset based on movement direction
  hue_offset = updateHueOffset(currentAngle, previousAngle, hue_offset);

  // Check and handle LED stability for flashing effect
  handleStabilityAndFlashing(ledIndex, previousIndex);

  // Update LED colors and display
  updateLEDDisplay(ledIndex, hue_offset, previousIndex);

  // Update previous values for the next loop
  previousIndex = ledIndex;
  previousAngle = currentAngle;

  delay(15); // Adjust delay for desired speed
}
void function2CountCycles() {
  static int previousIndex = -1;
  static float previousAngle = 0;
  static uint16_t backgroundHue = 0;  // Hue value for background color

  // Get the current smoothed angle
  float currentAngle = getSmoothedAngle(50); 

  // Get the corresponding LED index
  int ledIndex = degreeToLED(currentAngle);

  // Check for a full revolution clockwise
  if (previousAngle > 335 && currentAngle < 25) { 
    backgroundHue += 4368; // Increment background color hue
    if (backgroundHue >= 65536) {
      backgroundHue = 0; // Wrap around the hue value if it exceeds the maximum
    }
    Serial.println("Full revolution clockwise! Hue incremented.");
  }
  // Check for a full revolution counterclockwise
  else if (previousAngle < 25 && currentAngle > 335) {
    if (backgroundHue < 4368) {
      backgroundHue = 65536 - (4368 - backgroundHue); // Wrap around to the maximum hue value
    } else {
      backgroundHue -= 4368; // Decrement background color hue
    }
    Serial.println("Full revolution counterclockwise! Hue decremented.");
  }

  // Calculate the background color based on the current hue
  uint16_t backgroundColor = ledmatrix.color565(ledmatrix.ColorHSV(backgroundHue));

  // Turn on all LEDs except the one corresponding to the current angle
  for (int i = 0; i < numLeds; i++) {
    if (i == ledIndex) {
      // Set the LED corresponding to the current angle to white
      ledmatrix.drawPixel(ledCoordinates[i].x, ledCoordinates[i].y, ledmatrix.color565(255, 255, 255));
    } else {
      // Set all other LEDs to the background color
      ledmatrix.drawPixel(ledCoordinates[i].x, ledCoordinates[i].y, backgroundColor);
    }
  }

  // Update the LED matrix display
  ledmatrix.show();

  // Update previous values for the next loop
  previousIndex = ledIndex;
  previousAngle = currentAngle;

  delay(15); // Adjust delay for desired speed
}



// Function to initialize analog pins
void initAnalogPins() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}

// Get the angle from analog pins and apply smoothing
float getSmoothedAngle(int numSamples) {
  float sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += getAngle();
  }
  return sum / numSamples;
}

// Calculate the angle based on analog inputs
float getAngle() {
  double cosine = analogRead(A1) * 1.0 - 512;
  double sine = analogRead(A3) * 1.0 - 512;
  //Serial.println(analogRead(A1));
  //Serial.println(analogRead(A3));
  return (atan2(cosine, sine) * 180.0 / 3.14159) + 180.0;
}

// Convert an angle to the corresponding LED index
int degreeToLED(float degree) {
  degree = fmod(degree, 360);  // Normalize degree to 0-359
  if (degree < 0) degree += 360;
  return degree / 6;  // 360 degrees / 60 LEDs = 6 degrees per LED
}

// Update the hue offset based on the direction of movement
uint16_t updateHueOffset(float currentAngle, float previousAngle, uint16_t hue_offset) {
  if (currentAngle > previousAngle) {
    hue_offset += 1092;  // Clockwise
  } else if (currentAngle < previousAngle) {
    hue_offset -= 1092;  // Counterclockwise
  }
  return hue_offset;
}

// Handle LED stability check and trigger flashing if necessary
void handleStabilityAndFlashing(int ledIndex, int previousIndex) {
  if (ledIndex == previousIndex) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
      Serial.println("LED position unchanged for 3 seconds! Flashing LEDs...");
      flashLEDs(ledIndex);
      previousMillis = currentMillis; // Reset the timer
    }
  } else {
    previousMillis = millis(); // Reset timer if position changed
  }
}

// Update the LED display with new colors
void updateLEDDisplay(int ledIndex, uint16_t hue_offset, int previousIndex) {
  // Calculate color for the current LED
  uint32_t color888 = ledmatrix.ColorHSV(hue_offset);
  uint16_t color565 = ledmatrix.color565(color888);

  // Turn on the current LED
  ledmatrix.drawPixel(ledCoordinates[ledIndex].x, ledCoordinates[ledIndex].y, color565);

  // Turn off the previous LED
  if (previousIndex >= 0 && previousIndex != ledIndex) {
    turnOffPreviousLED(previousIndex);
  }
}

// Turn off the previous LED
void turnOffPreviousLED(int prevIndex) {
  ledmatrix.drawPixel(ledCoordinates[prevIndex].x, ledCoordinates[prevIndex].y, 0);
}

// Flash the LEDs for the stability effect
void flashLEDs(int ledIndex) {
  int flashDelay = 15; // Delay for the flashing effect

  for (int j = 0; j < 10; j++) { // Flash sequence for the 10 LEDs
    // Check if the position has changed
    int currentAngle = getSmoothedAngle(50);
    int currentLedIndex = degreeToLED(currentAngle);
    if (currentLedIndex != ledIndex) {
      Serial.println("Position changed during flash, stopping flash...");
      return; // Exit the flashing sequence if the position has changed
    }

    int forwardIndex = (ledIndex + 10 - j + numLeds) % numLeds;
    int backwardIndex = (ledIndex - 10 + j + numLeds) % numLeds;

    // Turn on forward and backward LEDs
    ledmatrix.drawPixel(ledCoordinates[forwardIndex].x, ledCoordinates[forwardIndex].y, ledmatrix.color565(255, 0, 0)); // Green color
    ledmatrix.drawPixel(ledCoordinates[backwardIndex].x, ledCoordinates[backwardIndex].y, ledmatrix.color565(255, 0, 0)); // Green color
    ledmatrix.show(); // Update the matrix
    delay(flashDelay);

    // Turn off forward and backward LEDs
    ledmatrix.drawPixel(ledCoordinates[forwardIndex].x, ledCoordinates[forwardIndex].y, 0);
    ledmatrix.drawPixel(ledCoordinates[backwardIndex].x, ledCoordinates[backwardIndex].y, 0);
    ledmatrix.show(); // Update the matrix
    delay(flashDelay);
  }
}

void turnOffAllQuadrants() {
  for (int i = 0; i < numLeds; i++) {
    int x = ledCoordinates[i].x;
    int y = ledCoordinates[i].y;
    ledmatrix.drawPixel(x, y, ledmatrix.color565(0, 0, 0)); // Turn off LED
  }
}

void function3(){
    // Get the current angle
  float currentAngle = getSmoothedAngle(50); 

  // Calculate the LED index based on angle
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
// Update the comet trail and handle overlap
void updateCometTrail(int ledIndex) {
  // Shift the previous LED indices to make room for the new one
  for (int i = 4; i > 0; i--) {
    previousLEDs[i] = previousLEDs[i - 1];
  }

  // Add the current LED index to the trail
  previousLEDs[0] = ledIndex;

  // Turn off the LED that was at the end of the tail
  if (previousLEDs[4] != -1) {
    int x = ledCoordinates[previousLEDs[4]].x;
    int y = ledCoordinates[previousLEDs[4]].y;
    ledmatrix.drawPixel(x, y, ledmatrix.color565(0, 0, 0)); // Turn off the LED
  }

  // Determine the color based on speed for the new LED
  uint16_t color = getColorBasedOnSpeed(updateInterval);

  // Draw the new LED with the determined color
  int x = ledCoordinates[ledIndex].x;
  int y = ledCoordinates[ledIndex].y;
  ledmatrix.drawPixel(x, y, color);

  ledmatrix.show();
}

// Get the color based on the speed of the rotation
uint16_t getColorBasedOnSpeed(unsigned long interval) {
  // Define speed thresholds (in milliseconds)
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
void function4(){
    // Get the current angle
  currentAngle = getSmoothedAngle(50); 

  // Calculate the difference between the current and previous angle
  float angleDifference = abs(currentAngle - previousAngle);

  // Determine how many LEDs to light up based on speed
  int ledCount = determineLEDCount(angleDifference);

  // Light up the LEDs with transitioning colors
  lightUpSpeedometer(ledCount);

  // Store the current angle as the previous angle for the next loop
  previousAngle = currentAngle;

  delay(15); // Adjust delay for desired speed
}

// Function to determine how many LEDs to light up based on speed
int determineLEDCount(float angleDifference) {
  if (angleDifference < 12.0) {
    return map(angleDifference, 0, 12, 0, 19); // Bottom 20 LEDs (Green to Yellow)
  } else if (angleDifference < 24.0) {
    return map(angleDifference, 12, 24, 20, 39); // Middle 20 LEDs (Yellow to Orange)
  } else {
    return map(angleDifference, 24, 36, 40, 59); // Top 20 LEDs (Orange to Red)
  }
}

// Function to light up the speedometer with transitioning colors
void lightUpSpeedometer(int ledCount) {
  turnOffAllQuadrants(); // Turn off all LEDs before lighting up the section

  for (int i = 0; i < ledCount; i++) {
    int x = ledCoordinates[i].x;
    int y = ledCoordinates[i].y;

    // Transition colors based on position in the array
    uint16_t color;
    if (i < 20) {
      // Transition from green to yellow
      int r = map(i, 0, 19, 0, 255); // Increase red component
      int g = 255;
      color = ledmatrix.color565(g, 0, r);
    } else if (i < 40) {
      // Transition from yellow to orange
      int r = 255;
      int g = map(i, 20, 39, 255, 165); // Decrease green component
      color = ledmatrix.color565(g, 0, r);
    } else {
      // Transition from orange to red
      int r = 255;
      int g = map(i, 40, 59, 165, 0); // Decrease green component
      color = ledmatrix.color565(g, 0, r);
    }

    ledmatrix.drawPixel(x, y, color);
  }

  ledmatrix.show();
}






float calculateHypotenuse(float sinValue, float cosValue) {
  // Use the Pythagorean theorem to calculate the hypotenuse
  float hypotenuse = sqrt(sinValue * sinValue + cosValue * cosValue);
  return hypotenuse;
}

int brightnessSetter(){
  float c = calculateHypotenuse(analogRead(1), analogRead(3));
  int brightness = (c - 500) / 4;
  if(brightness <= 20){
    brightness = 20;
  }
  Serial.println(brightness);
  return brightness;
}



