//-------------------------------------------------------
//| NVE Arduino Shield Angle/Rotation sensor demo
//| ASR012-10 TMR I2C angle sensor
//| Author - NVE Corporation, Sam Weber
//| 9/9/2024
//|
//| Compatible board: ASR012-10E-EVB01
//|
//| Insert an ASR012-10E-EVB01 breakout board
//| into the Shield edge connector.
//| Attach the magnet fixture.
//| Turn the magnet to see
//| the sensor’s functionality and precision.
//| 
//| Sensor connections: 
//|   SCL: SCL, SDA: SDA;
//-------------------------------------------------------

#include <Wire.h>                 // Include the Wire library for I²C communication
#include <Adafruit_IS31FL3741.h>   // Include the Adafruit library for the LED panel

const uint8_t ASR012_I2C_ADDRESS = 0x24; // I²C address of the ASR012 sensor
Adafruit_IS31FL3741_QT ledmatrix;        // Create an instance of the LED matrix

const int numLeds = 60; // Number of LEDs in the array

// Define LED coordinates
struct LEDCoordinate {
  int x;
  int y;
};

LEDCoordinate ledCoordinates[numLeds] = {
    {0, 8}, {0, 7}, {0, 6}, {0, 5}, {0, 4}, {0, 3}, {0, 2}, {0, 1}, {0, 0},
    {1, 8}, {1, 7}, {1, 6}, {1, 5}, {1, 4}, {1, 3}, {1, 2}, {1, 1}, {1, 0},
    {2, 8}, {2, 7}, {2, 6}, {2, 5}, {2, 4}, {2, 3}, {2, 2}, {2, 1}, {2, 0},
    {3, 8}, {3, 7}, {3, 6}, {3, 5}, {3, 4}, {3, 3}, {3, 2}, {3, 1}, {3, 0},
    {4, 8}, {4, 7}, {4, 6}, {4, 5}, {4, 4}, {4, 3}, {4, 2}, {4, 1}, {4, 0},
    {5, 8}, {5, 7}, {5, 6}, {5, 5}, {5, 4}, {5, 3}, {5, 2}, {5, 1}, {5, 0},
    {6, 8}, {6, 7}, {6, 6}, {6, 5}, {6, 4}, {6, 3}
};

int previousLedIndex = -1; // Store the previous LED index

void setup() {
  Serial.begin(9600);  // Start serial communication at 9600 baud
  Wire.begin();        // Initialize I²C as master
  pinMode(9, OUTPUT);
  digitalWrite(9, HIGH);
  
  // Initialize the LED matrix
  if (!ledmatrix.begin()) {
    Serial.println("LED panel not found!");
    while (1);  // Halt if LED matrix is not found
  }
  Serial.println("LED panel initialized!");
  ledmatrix.setGlobalCurrent(0x50);    // Set global brightness
  ledmatrix.setLEDscaling(0xFF);       // Set all LEDs to full brightness
  ledmatrix.enable(true);              // Enable the LED matrix
}

void loop() {
  // Read the angle and print it
  uint16_t angle = readAngle();       // Read the angle from the sensor
  float angleDegrees = angle / 10.0;  // Convert the angle to degrees
  Serial.print("Angle: ");
  Serial.print(angleDegrees);
  Serial.println(" degrees");

  // Convert the angle to the corresponding LED index
  int ledIndex = angleToLED(angle);

  // Only update the LED if the index has changed
  if (ledIndex != previousLedIndex) {
    lightUpLED(ledIndex);            // Light up the new LED
    previousLedIndex = ledIndex;      // Update the previous index
  }


}

// Function to read angle from ASR012 sensor
uint16_t readAngle() {
  uint16_t angle = 0;
  
  Wire.beginTransmission(ASR012_I2C_ADDRESS);
  Wire.write(0x00); // Address 0 to read the angle
  Wire.endTransmission(false); // Keep the connection active

  uint8_t bytesReceived = Wire.requestFrom((int)ASR012_I2C_ADDRESS, 2);
  if (bytesReceived == 2) { // Check if 2 bytes were received
    angle = (Wire.read() << 8) | Wire.read(); // Read and combine MSB and LSB
  }
  
  return angle; // Return the 16-bit angle value in tenths of degrees
}

// Convert angle to corresponding LED index
int angleToLED(int angle) {
  angle = angle % 3600;           // Normalize angle from 0-3600
  return angle / (3600 / numLeds); // Map angle to an LED index (0-59)
}

// Light up the LED at the specified index
void lightUpLED(int ledIndex) {
  // Turn off the previously lit LED, if any
  if (previousLedIndex >= 0) {
    ledmatrix.drawPixel(ledCoordinates[previousLedIndex].x, ledCoordinates[previousLedIndex].y, ledmatrix.color565(0, 0, 0));
  }
  
  // Turn on the LED at the specified index in green
  ledmatrix.drawPixel(ledCoordinates[ledIndex].x, ledCoordinates[ledIndex].y, ledmatrix.color565(0, 255, 0));
  ledmatrix.show(); // Update the LED panel
}

