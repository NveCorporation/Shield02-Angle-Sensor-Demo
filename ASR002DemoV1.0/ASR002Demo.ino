//-------------------------------------------------------
//| NVE's Arduino Shield Demo
//| ASR002 
//| Author - NVE Coorporation, Sam Weber
//| 9/9/2024
//| 
//| Insert the the ASR002 into the Demo, 
//| then spin the magnet in their slots
//| to see the demo functions and sensor
//| precision.
//| 
//| Sensor connections: 
//| SCLK->P13; MISO->P12; MOSI->P11; SS->P10.
//-------------------------------------------------------
#include <Adafruit_IS31FL3741.h>
#include <SPI.h>

Adafruit_IS31FL3741_QT ledmatrix;
TwoWire *i2c = &Wire; // I2C interface

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
int lastLedIndexes[5] = {-1, -1, -1, -1, -1};  // To store the last 5 LED indexes
bool clockwise = true;  // To track the rotation direction

void setup() {
  Serial.begin(9600);
  pinMode(10, OUTPUT); // Pin 10 = Sensor SS 
  SPI.begin(); 
  // Set clock rate at 2 Mbits/s; MSB first; Mode 0 
  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0)); 

  digitalWrite(10, HIGH); // Disable to reset the sensor 
  digitalWrite(10, LOW);  // Re-enable sensor 

  if (!ledmatrix.begin(IS3741_ADDR_DEFAULT, i2c)) {
    Serial.println("IS31FL3741 not found");
    while (1);
  }
  Serial.println("IS31FL3741 found!");

  i2c->setClock(800000);

  ledmatrix.setLEDscaling(0xFF);

  // Lower the global current to reduce overall brightness
  ledmatrix.setGlobalCurrent(0x50);  // Reduced global current
  Serial.print("Global current set to: ");
  Serial.println(ledmatrix.getGlobalCurrent());

  ledmatrix.enable(true);
}

void loop() {
asr002Method();
}

//ASR002 master method, holds the functionality of the LED
//panel and the ASR002's SPI communication 
void asr002Method(){
    SPICommunication();
  int ledIndex = angleToLED(angle);

  if (ledIndex != previousLedIndex) {
    // Determine the rotation direction
    clockwise = (ledIndex > previousLedIndex) || (ledIndex == 0 && previousLedIndex == numLeds - 1);
    
    // Update LEDs with continuous lighting
    updateComet(ledIndex);
    previousLedIndex = ledIndex; // Update the previous index
  }

  delay(10); // Adjust delay for desired update speed
}


//Easy SPI communication to the ASR002 
void SPICommunication() {
  digitalWrite(10, LOW);  // Enable sensor communication
  
  // Read angle from sensor
  angle = (SPI.transfer(0)) << 8; // Send 0 for address angle; receive angle MSB 
  delayMicroseconds(3);   // Allow 3 us between address bytes 
  angle |= SPI.transfer(0); // 2nd address byte (0 for read); receive angle LSB 
  delayMicroseconds(15);  // Allow 10 us for next data 

  digitalWrite(10, HIGH); // Disable sensor communication

  Serial.print("Angle: ");
  Serial.println(angle);
}


//converts the current angle to what LED should be lit
int angleToLED(int angle) {
  // Normalize angle from 0-3600 to 0-3599
  angle = angle % 3600;
  
  // Map angle to an LED index (0-59)
  return angle / 60;
}

void updateComet(int ledIndex) {
  // Turn off the oldest LED in the tail (only if the tail is fully populated)
  if (lastLedIndexes[4] != -1) {
    int oldIndex = lastLedIndexes[4];
    ledmatrix.drawPixel(ledCoordinates[oldIndex].x, ledCoordinates[oldIndex].y, ledmatrix.color565(0, 0, 0));
  }

  // Shift the last 5 LED indexes
  for (int i = 4; i > 0; i--) {
    lastLedIndexes[i] = lastLedIndexes[i - 1];
  }
  lastLedIndexes[0] = ledIndex;  // Store the current LED index

  // Light up the LEDs based on the direction of movement with harsher dimming
  for (int i = 0; i < 5; i++) {
    if (lastLedIndexes[i] != -1) {  // Check if the index is valid
      int index = lastLedIndexes[i];
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

