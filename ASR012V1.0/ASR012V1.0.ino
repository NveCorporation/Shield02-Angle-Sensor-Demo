//-------------------------------------------------------
//| NVE's Arduino Shield Product Demo's
//| NVE Off Axis Sensor's - ASR0012 Series
//| Author - NVE Coorporation, Sam Weber
//| 9/9/2024
//| 
//| Here is the code for NVE's arduino shield
//| product with the ASR012 sensor. Insert the ASR012
//| sensor into its slot then put the plastic 
//| shield on top. Spin the magnet and see
//| the functionality and precision
//-------------------------------------------------------
#include <Adafruit_IS31FL3741.h>
#include <SoftwareWire.h> // Include the SoftwareWire library for software I²C communication

Adafruit_IS31FL3741_QT ledmatrix;
TwoWire *i2c = &Wire; // I2C interface for the LED matrix

const int numLeds = 60; // Number of LEDs in the array

// Define your array of LED coordinates as ordered pairs (x, y)
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

int angle;
int previousLedIndex = -1;
int lastLedIndexesASR012[5] = {-1, -1, -1, -1, -1};  // To store the last 5 LED indexes
bool clockwise = true;  // To track the rotation direction

const int ASR012_I2C_ADDRESS = 0x24; // I²C address of the ASR012 sensor
SoftwareWire myWire(11, 12); // SDA on pin 11, SCL on pin 12

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  myWire.begin(); // Initialize I²C as master on pins 11 and 12
  
  if (!ledmatrix.begin(IS3741_ADDR_DEFAULT, i2c)) {
    Serial.println("IS31FL3741 not found");
    while (1);
  }
  Serial.println("IS31FL3741 found!");

  i2c->setClock(800000);  // Set I2C clock speed for the LED matrix

  ledmatrix.setLEDscaling(0xFF);  // Set all LEDs to full scale

  // Lower the global current to reduce overall brightness
  ledmatrix.setGlobalCurrent(0x50);  // Set the global current
  Serial.print("Global current set to: ");
  Serial.println(ledmatrix.getGlobalCurrent());

  ledmatrix.enable(true);  // Enable the LED matrix
}

void loop() {
  ASR012();  // Call the function to read the angle and update LEDs
}


//ASR012 master method that controls the LED's and
//I2C communication with the sensor
void ASR012() {
  readAngleASR012();
  int ledIndex = angleToLED(angle);
  if (ledIndex != previousLedIndex) {
    // Determine the rotation direction
    clockwise = (ledIndex > previousLedIndex) || (ledIndex == 0 && previousLedIndex == numLeds - 1);
    
    // Update LEDs with continuous lighting
    updateCometASR012(ledIndex);
    previousLedIndex = ledIndex; // Update the previous index
    pinMode(11, INPUT);
    pinMode(12, INPUT);
  }

  delay(10); // Adjust delay for desired update speed
}


//Uses Softwire I2C, the LED panel is using the I2C pins
//so we must rewire the device to function with a software
//driven I2C Communication protocol
uint16_t readAngleASR012() {
  myWire.beginTransmission(ASR012_I2C_ADDRESS);
  myWire.write(0x00); // Address 0 to read the angle
  myWire.endTransmission(false); // Send the data but keep the connection active
  int bytesReceived = myWire.requestFrom(ASR012_I2C_ADDRESS, 2, true); // Request 2 bytes from the sensor
  if (bytesReceived == 2) { // Check if 2 bytes were received
    angle = myWire.read() << 8; // Read the MSB
    angle |= myWire.read(); // Read the LSB and combine with MSB
  }
  return angle; // Return the 16-bit angle value in tenths of degrees
}


//Converts the angle to corresponding LED
int angleToLED(int angle) {
  // Normalize angle from 0-3600 to 0-3599
  angle = angle % 3600;
  
  // Map angle to an LED index (0-59)
  return angle / 60;
}


//Updates the comet tail of the ASR012
void updateCometASR012(int ledIndex) {
  // Turn off the oldest LED in the tail (only if the tail is fully populated)
  if (lastLedIndexesASR012[4] != -1) {
    int oldIndex = lastLedIndexesASR012[4];
    ledmatrix.drawPixel(ledCoordinates[oldIndex].x, ledCoordinates[oldIndex].y, ledmatrix.color565(0, 0, 0));
  }

  // Shift the last 5 LED indexes
  for (int i = 4; i > 0; i--) {
    lastLedIndexesASR012[i] = lastLedIndexesASR012[i - 1];
  }
  lastLedIndexesASR012[0] = ledIndex;  // Store the current LED index

  // Light up the LEDs based on the direction of movement with harsher dimming
  for (int i = 0; i < 5; i++) {
    if (lastLedIndexesASR012[i] != -1) {  // Check if the index is valid
      int index = lastLedIndexesASR012[i];
      uint16_t brightness = map(i, 0, 4, 255, 10); // Harsher dimming, lower the minimum brightness
      ledmatrix.drawPixel(ledCoordinates[index].x, ledCoordinates[index].y, ledmatrix.color565(0, brightness, brightness)); // Green, Blue, Red order
    }
  }

  // Update the LED matrix display
  ledmatrix.show();
}

void turnOffAllQuadrants() {
  for (int i = 0; i < numLeds; i++) {
    int x = ledCoordinates[i].x;
    int y = ledCoordinates[i].y;
    ledmatrix.drawPixel(x, y, ledmatrix.color565(0, 0, 0)); // Turn off LED
  }
}
