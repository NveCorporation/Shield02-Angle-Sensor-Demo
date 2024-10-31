//-------------------------------------------------------
//| NVE Arduino Shield Angle/Rotation sensor demo
//| ASR022-10 TMR ABZ angle sensor
//| Author - NVE Corporation, Sam Weber
//| 9/9/2024
//|
//| Compatible board: ASR022-10E-EVB01
//|
//| Insert an ASR022-10E-EVB01 breakout board
//| into the Shield edge connector.
//| Attach the magnet fixture.
//| Turn the magnet to see
//| the sensor’s functionality and precision,
//| through quadrature encoding.
//|
//| Sensor connections: 
//| A -> 10; B -> 12; Z -> 13
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
int lastZpinState = LOW;  // Track the last state of the Z pin

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

// ASR022 master method, controls the LED panel as well as signal processing with 
// ABZ signals for encoding
void ASR022Master(){
  // Read the state of the Z pin
  int Zpin = digitalRead(13);

  // Check if Z pin is HIGH
  if (Zpin == HIGH) {
    lightUpAllLEDs();  // Turn on all LEDs
  } else if (lastZpinState == HIGH && Zpin == LOW) {
    // Turn off all LEDs when Z pin goes from HIGH to LOW
    turnOffAllLEDs();
  } else {
    // Normal behavior when Z pin is LOW
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
      int ledIndex = (angle / 6) % numLeds;  // Ensures ledIndex stays within 0 to 59

      // Update LED display based on the calculated LED position
      updateLEDDisplay(ledIndex);
    }
  }

  // Update last state of the Z pin
  lastZpinState = Zpin;
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

//Turns off the previous LED to make sure only 1 LED is lit at a time
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

//Lights up all the LEDs
void lightUpAllLEDs() {
  // Turn on all LEDs in the LED matrix
  for (int i = 0; i < numLeds; i++) {
    int x = ledCoordinates[i].x;
    int y = ledCoordinates[i].y;
    setLED(x, y, true);  // Turn on the LED at each coordinate
  }
  // Update the display
  ledmatrix.show();
}


//Turns off all of the LEDs
void turnOffAllLEDs() {
  // Turn off all LEDs in the LED matrix
  for (int i = 0; i < numLeds; i++) {
    int x = ledCoordinates[i].x;
    int y = ledCoordinates[i].y;
    setLED(x, y, false);  // Turn off the LED at each coordinate
  }
  // Update the display
  ledmatrix.show();
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
