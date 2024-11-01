//-------------------------------------------------------
//| NVE Arduino Shield Angle/Rotation sensor demo
//| ADT00x-10 TMR rotation sensors
//| Author - NVE Corporation, Sam Weber
//| 9/26/2024
//|
//| Compatible breakout boards: 
//|   ADT001-10E-EVB01
//|   ADT002-10E-EVB01
//|   ADT005-10E-EVB01
//|
//| Insert an ADT00x-10E-EVB01 breakout board
//| into the Shield edge connector.
//| Attach the magnet fixture.
//| Turn the magnet to see
//| the sensor’s operation.
//| 
//| Sensor connections: 
//|   SIN (digital)11; COS (digital)12; Excess Magnetic Field (digital)13
//-------------------------------------------------------
 

#include <Adafruit_IS31FL3741.h>

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

// Define the states
//this device treats its quadrants as states
enum StatesADT {
  STATE_0,
  STATE_1,
  STATE_3,
  STATE_2,
  STATE_COUNT // Helps to keep track of the number of states
};

// Define the struct for each state
struct QuadrantState {
  StatesADT state;
  uint16_t color;
  int startLED;
  int endLED;
};

// Create an array of QuadrantState structs, indexed by the enum States
QuadrantState quadrantStates[STATE_COUNT] = {
  {STATE_0, ledmatrix.color565(0, 255, 0), 0, 14},   // Blue for STATE_0
  {STATE_1, ledmatrix.color565(255, 0, 255), 15, 29},  // Yellow for STATE_1
  {STATE_3, ledmatrix.color565(255, 0, 0), 30, 44},  // Green for STATE_3
  {STATE_2, ledmatrix.color565(0, 0, 255), 45, 59} // Red for STATE_2
};

// Current and previous states
StatesADT currentStateADT = STATE_0;
StatesADT previousStateADT = STATE_0;  // To track the previous state
StatesADT startStateADT = STATE_0;  // Track the starting state for revolution detection

// Revolution counter
int revolutionCount = 0;

// Direction flag
bool clockwise = true;  // true for clockwise, false for counterclockwise

// Define the input pins for state detection
const int pin11 = 11;
const int pin12 = 12;

volatile bool toggleState = false; // Variable to toggle between true and false
volatile unsigned long lastDebounceTime = 0; // To store the last debounce time
const unsigned long debounceDelay = 50; // Debounce time in milliseconds

const int buttonPin = 2; // Pin for the interrupt (Digital Pin 2)

bool firstIterationToggle = true;
//=======================================================================================
//This section of the code contains the setup for the Arduino and the LED shield,
//as well as shows the program flow in the infinite loop.

void setup() {
  Serial.begin(115200);
  Serial.println("Adafruit QT RGB Matrix LED Setup");

  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with internal pull-up resistor
  
  // Attach interrupt to the button pin (pin 2), triggering on falling edge (button press)
  attachInterrupt(digitalPinToInterrupt(buttonPin), toggleStateISR, FALLING);

  if (!ledmatrix.begin(IS3741_ADDR_DEFAULT, i2c)) {
    Serial.println("IS31FL3741 not found");
    while (1);
  }
  Serial.println("IS31FL3741 found!");

  i2c->setClock(800000);

  ledmatrix.setLEDscaling(0xFF);
  ledmatrix.setGlobalCurrent(0xFF);  // Max brightness
  ledmatrix.enable(true);

  // Set the pin modes to INPUT
  pinMode(pin11, INPUT);
  pinMode(pin12, INPUT);
}

void loop() {  
  if(digitalRead(13) == 0){
    if(toggleState){
      if(firstIterationToggle){
        turnOffAllQuadrants(); 
        Serial.println("ToggleState: " + String(firstIterationToggle));
        quadrantFlasher();
        firstIterationToggle = false; // Set to false after the first iteration
      } else {
        quadrantFlasher();
      }
    } else {
      if(firstIterationToggle){
        turnOffAllQuadrants(); 
        directionalQuadrants();
        firstIterationToggle = false; // Set to false after the first iteration
      } else {
        directionalQuadrants();
      }
    }
  } else{
    Serial.println("Trigger");
    flashAllLEDs(0, 255, 0, 2);  // Adjusted to the correct RGB order (Green,
  }
}

//=======================================================================================
//This ISR handles the button. When pressed it switches the LED display mode.
void toggleStateISR() {
  unsigned long currentTime = millis(); 
  // Check if the debounce time has passed

  if ((currentTime - lastDebounceTime) > debounceDelay) {
    toggleState = !toggleState; // Toggle the state
    lastDebounceTime = currentTime; // Update the last debounce time
    firstIterationToggle = true; // Reset on state change
  }
}
//=======================================================================================
//These functions deal with the State Machine which corresponds to the states
//produced by the ADT002. This device breaks the LED panel into 4 separate quadrants
//to reflect the 2-bit state machine that is the ADT002, which is why you see the
//quadrant term appear so frequently.

//Flashes all the LEDs in a quadrant
void quadrantFlasher(){
    // Read the current states of pin11 and pin12
  int pin11State = digitalRead(pin11);
  int pin12State = digitalRead(pin12);

  // Determine the next state based on the current inputs and direction
  switch (currentStateADT) {
    case STATE_0:
      if (pin11State == HIGH && pin12State == LOW) {
        changeState(STATE_1);
        clockwise = true;
      } else if (pin11State == LOW && pin12State == HIGH) {
        changeState(STATE_2);
        clockwise = false;
      }
      break;
    case STATE_1:
      if (pin11State == HIGH && pin12State == HIGH) {
        changeState(STATE_3);
      } else if (pin11State == LOW && pin12State == LOW) {
        if (!clockwise && previousStateADT == STATE_1) {
          handleFullRevolution(false);  // Counterclockwise decrement
        }
        changeState(STATE_0);
      }
      break;
    case STATE_3:
      if (pin11State == LOW && pin12State == HIGH) {
        changeState(STATE_2);
      } else if (pin11State == HIGH && pin12State == LOW) {
        changeState(STATE_1);
      }
      break;
    case STATE_2:
      if (pin11State == LOW && pin12State == LOW) {
        if (clockwise && previousStateADT == STATE_2) {
          handleFullRevolution(true);  // Clockwise increment
        }
        changeState(STATE_0);
      } else if (pin11State == HIGH && pin12State == HIGH) {
        changeState(STATE_3);
      }
      break;
  }
  // Small delay to avoid rapid state changes (debounce)
  delay(50);
}

void directionalQuadrants() {
  // Read the current states of pin11 and pin12
  int pin11State = digitalRead(pin11);
  int pin12State = digitalRead(pin12);

  // Determine the next state based on the current inputs and direction
  switch (currentStateADT) {
    case STATE_0:
      if (pin11State == HIGH && pin12State == LOW) {
        clockwise = true;
        changeStateAlternate(STATE_1);
      } else if (pin11State == LOW && pin12State == HIGH) {
        clockwise = false;
        changeStateAlternate(STATE_2);
      }
      break;
    case STATE_1:
      if (pin11State == HIGH && pin12State == HIGH) {
        clockwise = true;
        changeStateAlternate(STATE_3);
      } else if (pin11State == LOW && pin12State == LOW) {
        clockwise = false;
        changeStateAlternate(STATE_0);
      }
      break;
    case STATE_3:
      if (pin11State == LOW && pin12State == HIGH) {
        clockwise = true;
        changeStateAlternate(STATE_2);
      } else if (pin11State == HIGH && pin12State == LOW) {
        clockwise = false;
        changeStateAlternate(STATE_1);
      }
      break;
    case STATE_2:
      if (pin11State == LOW && pin12State == LOW) {
        clockwise = true;
        changeStateAlternate(STATE_0);
      } else if (pin11State == HIGH && pin12State == HIGH) {
        clockwise = false;
        changeStateAlternate(STATE_3);
      }
      break;
  }
  // Small delay to avoid rapid state changes (debounce)
  delay(50);
}

void changeStateAlternate(StatesADT newState) {
  if (currentStateADT != newState) {
    previousStateADT = currentStateADT;
    currentStateADT = newState;

    // Turn off all quadrants
    turnOffAllQuadrants();

    // Create a temporary QuadrantState to modify the color
    QuadrantState tempState = quadrantStates[currentStateADT];

    // Set color based on the direction
    if (clockwise) {
      tempState.color = ledmatrix.color565(0, 255, 0); // Green for clockwise
    } else {
      tempState.color = ledmatrix.color565(0, 0, 255); // Blue for counterclockwise
    }

    // Activate the current quadrant based on the modified state
    setQuadrantColorWithTempState(tempState);

    // Update the LED matrix
    ledmatrix.show();
  }
}

// Function to set the color of a specific quadrant based on a temporary QuadrantState
void setQuadrantColorWithTempState(QuadrantState tempState) {
  for (int i = tempState.startLED; i <= tempState.endLED && i < numLeds; i++) {
    int x = ledCoordinates[i].x;  // Get x position from the array
    int y = ledCoordinates[i].y;  // Get y position from the array
    ledmatrix.drawPixel(x, y, tempState.color);
  }
}

// Change state and update LEDs only when the state changes
void changeState(StatesADT newState) {
  if (currentStateADT != newState) {
    previousStateADT = currentStateADT;
    currentStateADT = newState;

    // Detect a full revolution
    if (currentStateADT == STATE_0 && previousStateADT == STATE_2 && startStateADT == STATE_0 && clockwise) {
      handleFullRevolution(true);
    } else if (currentStateADT == STATE_0 && previousStateADT == STATE_1 && startStateADT == STATE_0 && !clockwise) {
      handleFullRevolution(false);
    }

    // Turn off all quadrants
    turnOffAllQuadrants();

    // Activate the current quadrant based on the state
    setQuadrantColor(newState);

    // Update the LED matrix
    ledmatrix.show();
  }
}

void handleFullRevolution(bool clockwise) {
  // Increment or decrement the revolution count based on the direction
  if (clockwise) {
    revolutionCount++;
  } else {
    revolutionCount--;
  }
  
  // Print the current revolution count
  Serial.print("Full revolution ");
  Serial.print(clockwise ? "clockwise" : "counterclockwise");
  Serial.print(". Count: ");
  Serial.println(revolutionCount);

  // Flash all LEDs green twice
  flashAllLEDs(0, 255, 0, 2);  // Adjusted to the correct RGB order (Green, Blue, Red)

  // After flashing, update the LED states based on the new state
  changeState(currentStateADT);
}

//=======================================================================================
//These functions turn LEDs on or off individually or by quadrant.

// Function to turn off all LEDs in all quadrants
void turnOffAllQuadrants() {
  for (int i = 0; i < numLeds; i++) {
    int x = ledCoordinates[i].x;
    int y = ledCoordinates[i].y;
    ledmatrix.drawPixel(x, y, ledmatrix.color565(0, 0, 0)); // Turn off LED
  }
}

// Function to flash all LEDs a specified color a certain number of times
void flashAllLEDs(uint8_t r, uint8_t g, uint8_t b, int times) {
  for (int t = 0; t < times; t++) {
    // Turn all LEDs to the specified color
    for (int i = 0; i < numLeds; i++) {
      int x = ledCoordinates[i].x;
      int y = ledCoordinates[i].y;
      ledmatrix.drawPixel(x, y, ledmatrix.color565(r, g, b));
    }
    ledmatrix.show();
    delay(250); // Hold the color for 250 milliseconds

    // Turn off all LEDs
    turnOffAllQuadrants();
    delay(250); // Pause for 250 milliseconds before the next flash
  }
}

// Function to set the color of a specific quadrant based on the state
void setQuadrantColor(StatesADT state) {
  QuadrantState qs = quadrantStates[state];  // Get the struct for the state
  for (int i = qs.startLED; i <= qs.endLED && i < numLeds; i++) {
    int x = ledCoordinates[i].x;  // Get x position from the array
    int y = ledCoordinates[i].y;  // Get y position from the array
    ledmatrix.drawPixel(x, y, qs.color);
  }
}
//=======================================================================================
