//-------------------------------------------------------
//| NVE's Arduino Shield Demo
//| ASR022 
//| Author - NVE Coorporation, Sam Weber
//| 9/9/2024
//| 
//| Insert the the ASR022 into the Demo, 
//| then spin the magnet in their slots
//| to see the demo functions and sensor
//| precision.
//| 
//| Sensor connections: 
//| A 10; B 12;
//-------------------------------------------------------
#include <Adafruit_IS31FL3741.h>

Adafruit_IS31FL3741_QT ledmatrix;
TwoWire *i2c = &Wire; // I2C interface

// Define the structure for LED coordinates
struct LEDCoordinate {
  int x;
  int y;
};

// Number of LEDs
const int numLeds = 60;  // Number of LEDs in your array

// Custom LED coordinate array
LEDCoordinate ledCoordinates[numLeds] = {
    {0, 8}, {0, 7}, {0, 6}, {0, 5}, {0, 4}, {0, 3}, {0, 2}, {0, 1}, {0, 0},
    {1, 8}, {1, 7}, {1, 6}, {1, 5}, {1, 4}, {1, 3}, {1, 2}, {1, 1}, {1, 0},
    {2, 8}, {2, 7}, {2, 6}, {2, 5}, {2, 4}, {2, 3}, {2, 2}, {2, 1}, {2, 0},
    {3, 8}, {3, 7}, {3, 6}, {3, 5}, {3, 4}, {3, 3}, {3, 2}, {3, 1}, {3, 0},
    {4, 8}, {4, 7}, {4, 6}, {4, 5}, {4, 4}, {4, 3}, {4, 2}, {4, 1}, {4, 0},
    {5, 8}, {5, 7}, {5, 6}, {5, 5}, {5, 4}, {5, 3}, {5, 2}, {5, 1}, {5, 0},
    {6, 8}, {6, 7}, {6, 6}, {6, 5}, {6, 4}, {6, 3}
};

int counter = 0;  // Tracks the position of the encoder
int A = 0;
int B = 0;
int lastA = 0;
int lastB = 0;
int previousIndex = -1;  // Store the last LED position to turn it off

void setup() {
  Serial.begin(115200);  // Initialize serial communication at 115200 baud
  pinMode(10, INPUT);    // A signal on pin 10
  pinMode(12, INPUT);    // B signal on pin 12
  lastA = digitalRead(10);
  lastB = digitalRead(12);

  // Initialize the LED matrix
  if (!ledmatrix.begin(IS3741_ADDR_DEFAULT, i2c)) {
    Serial.println("IS31FL3741 not found");
    while (1);
  }
  ledmatrix.setLEDscaling(0xFF);
  ledmatrix.setGlobalCurrent(0xFF);
  ledmatrix.enable(true);

  ledmatrix.show();
}

void loop() {
ASR022Master();
}

//ASR022 master method, controls the LED panel as well as signal processing with 
//ABZ signals for encoding
void ASR022Master(){
  A = digitalRead(10);  // A signal from pin 10
  B = digitalRead(12);  // B signal from pin 12

  if (A != lastA) {
    if (A == B) {
      counter++;  // Clockwise
    } else {
      counter--;  // Counterclockwise
    }
    lastA = A;

    // Wrap the counter to stay within the 0-255 range
    counter = wrapCounter(counter, 256);

    // Calculate the angle based on the counter
    int angle = map(counter, 0, 255, 0, 360);

    // Convert angle to LED position (each LED represents 6 degrees)
    // Convert angle to LED position (each LED represents 6 degrees)
    int ledIndex = (angle / 6) % numLeds;  // Ensures ledIndex stays within 0 to 59

    // Update LED display based on the calculated LED position
    updateLEDDisplay(ledIndex);

    // Print the counter and angle for debugging
    Serial.print("Counter: ");
    Serial.print(counter);
    Serial.print(" | Angle: ");
    Serial.print(angle);
    Serial.print(" | LED Index: ");
    Serial.print(ledIndex);
    Serial.print(" | Values: ");
    
  }
}

void updateLEDDisplay(int position) {
  // Clear the previous LED if necessary
  if (previousIndex >= 0 && previousIndex != position) {
    turnOffPreviousLED(previousIndex);
  }

  // Get the LED coordinates from the custom array
  int x = ledCoordinates[position].x;
  int y = ledCoordinates[position].y;
  Serial.println(String(ledCoordinates[position].x) + " " + String(ledCoordinates[position].y));
  
  // Turn on the LED at the calculated position
  setLED(x, y, true);

  // Update the display
  ledmatrix.show();

  // Update the previous index
  previousIndex = position;
}

void turnOffPreviousLED(int prevIndex) {
  // Get the coordinates of the previous LED
  int x = ledCoordinates[prevIndex].x;
  int y = ledCoordinates[prevIndex].y;

  // Turn off the LED at the previous position
  setLED(x, y, false);

  // Update the display
  ledmatrix.show();
}

void setLED(int x, int y, bool state) {
  if (state) {
    // Turn on the LED at (x, y)
    ledmatrix.drawPixel(x, y, ledmatrix.color565(255, 255, 255));  // White color
  } else {
    // Turn off the LED at (x, y)
    ledmatrix.drawPixel(x, y, 0);
  }
}

// Wrap the counter within the specified range
int wrapCounter(int value, int maxValue) {
  if (value < 0) {
    return maxValue - 1;
  } else if (value >= maxValue) {
    return 0;
  } else {
    return value;
  }
}


