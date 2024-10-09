// #include <Arduino.h>

// // Define input pins
// const int leftPin = 13;   // GPIO 13
// const int rightPin = 14;  // GPIO 14
// const int upPin = 27;     // GPIO 27
// const int downPin = 15;   // GPIO 15

// void setup() {
//   Serial.begin(9600);

//   // Initialize input pins with pull-up resistors
//   pinMode(leftPin, INPUT_PULLUP);
//   pinMode(rightPin, INPUT_PULLUP);
//   pinMode(upPin, INPUT_PULLUP);
//   pinMode(downPin, INPUT_PULLUP);

//   // Optional: Set internal pull-up resistors if external ones are not used
//   // ESP32's INPUT_PULLUP mode enables internal pull-up resistors (~45 kΩ)
// }

// void loop() {
//   // Read and display the state of each plate
//   // Serial.print("Left: ");
//   // Serial.print(digitalRead(leftPin));
//   // Serial.print(", Right: ");
//   // Serial.print(digitalRead(rightPin));
//   // Serial.print(", Up: ");
//   // Serial.print(digitalRead(upPin));
//   // Serial.print(", Down: ");
//   // Serial.println(digitalRead(downPin));
//   // Read the state of each plate
//   int leftState = digitalRead(leftPin);
//   int rightState = digitalRead(rightPin);
//   int upState = digitalRead(upPin);
//   int downState = digitalRead(downPin);

//   // The input is LOW when touched
//   if (leftState == LOW) {
//     Serial.println("Left touched");
//     // Perform action for left touch
//   }

//   if (rightState == LOW) {
//     Serial.println("Right touched");
//     // Perform action for right touch
//   }

//   if (upState == LOW) {
//     Serial.println("Up touched");
//     // Perform action for up touch
//   }

//   if (downState == LOW) {
//     Serial.println("Down touched");
//     // Perform action for down touch
//   }

//   delay(60); // Small delay to debounce
// }


// #include <Arduino.h>

// // Define the number of input pins
// const int numPins = 4;

// // Define input pins: Left, Right, Up, Down
// const int inputPins[numPins] = {13, 14, 27, 15};

// // Define labels for serial output
// const String pinLabels[numPins] = {"Left", "Right", "Up", "Down"};

// // Debounce interval in milliseconds
// const unsigned long debounceInterval = 200;

// // Arrays to store the last detection time and previous states for each pin
// unsigned long lastTouchTime[numPins] = {0, 0, 0, 0};
// bool previousState[numPins] = {HIGH, HIGH, HIGH, HIGH};

// void setup() {
//   Serial.begin(9600);

//   // Initialize input pins with internal pull-up resistors
//   for (int i = 0; i < numPins; i++) {
//     pinMode(inputPins[i], INPUT_PULLUP);
//   }
// }

// void loop() {
//   unsigned long currentMillis = millis(); // Current time

//   // Iterate through each pin
//   for (int i = 0; i < numPins; i++) {
//     int currentState = digitalRead(inputPins[i]);

//     // Detect transition from HIGH to LOW (touch event)
//     if (currentState == LOW && previousState[i] == HIGH) {
//       // Check if debounce interval has passed since the last touch
//       if (currentMillis - lastTouchTime[i] > debounceInterval) {
//         Serial.println(pinLabels[i] + " touched");
//         lastTouchTime[i] = currentMillis;
//         // Add your action here (e.g., send a signal, control a device, etc.)
//       }
//     }

//     // Update previous state for the next loop iteration
//     previousState[i] = currentState;
//   }

//   // No delay here; loop runs continuously
// }


// #include <Arduino.h>

// // Define input pins
// const int leftPin = 13;   // GPIO 13
// const int rightPin = 14;  // GPIO 14
// const int upPin = 32;     // GPIO 27
// const int downPin = 15;   // GPIO 15

// // Debounce interval in milliseconds
// const unsigned long debounceInterval = 200;

// // Arrays to store the last detection time and previous states for each pin
// unsigned long lastTouchTime[] = {0, 0, 0, 0};
// bool previousState[] = {HIGH, HIGH, HIGH, HIGH};

// const int numPins = 4;
// const String pinLabels[] = {"Left", "Right", "Up", "Down"};

// void setup() {
//   Serial.begin(9600);

//   // Initialize input pins with internal pull-up resistors
//   for (int i = 0; i < numPins; i++) {
//     pinMode(leftPin + i, INPUT_PULLUP); // Assumes pins are sequential
//   }
// }

// void loop() {
//   unsigned long currentMillis = millis(); // Current time

//   // Iterate through each pin
//   for (int i = 0; i < numPins; i++) {
//     int currentState = digitalRead(leftPin + i); // Read each pin

//     // Detect transition from HIGH to LOW (touch event)
//     if (currentState == LOW && previousState[i] == HIGH) {
//       // Check if debounce interval has passed since the last touch
//       if (currentMillis - lastTouchTime[i] > debounceInterval) {
//         Serial.println(pinLabels[i] + " touched");
//         lastTouchTime[i] = currentMillis;
//         // You can add additional actions here if needed
//       }
//     }

//     // Update previous state for the next loop iteration
//     previousState[i] = currentState;
//   }

//   // No delay here; loop runs continuously
// }


#include <Arduino.h>

// Define the number of input pins
const int numPins = 4;

// Define input pins: Left, Right, Up, Down
const int inputPins[numPins] = {13, 14, 33, 15}; // GPIO13, GPIO14, GPIO32, GPIO15

// Define labels for serial output
const String pinLabels[numPins] = {"Left", "Right", "Up", "Down"};

// Debounce interval in milliseconds
const unsigned long debounceInterval = 200;

// Arrays to store the last detection time and previous states for each pin
unsigned long lastTouchTime[numPins] = {0, 0, 0, 0};
bool previousState[numPins] = {HIGH, HIGH, HIGH, HIGH};

void setup() {
  Serial.begin(9600); // Consider increasing to 115200 for faster communication

  // Initialize input pins with internal pull-up resistors
  for (int i = 0; i < numPins; i++) {
    pinMode(inputPins[i], INPUT_PULLUP);
  }
}

void loop() {
  unsigned long currentMillis = millis(); // Current time

  // Iterate through each pin
  for (int i = 0; i < numPins; i++) {
    int currentState = digitalRead(inputPins[i]); // Read each pin

    // Detect transition from HIGH to LOW (touch event)
    if (currentState == LOW && previousState[i] == HIGH) {
      // Check if debounce interval has passed since the last touch
      if (currentMillis - lastTouchTime[i] > debounceInterval) {
        Serial.println(pinLabels[i] + " touched");
        lastTouchTime[i] = currentMillis;
        // Add your action here (e.g., send a signal, control a device, etc.)
      }
    }

    // Update previous state for the next loop iteration
    previousState[i] = currentState;
  }

  // No delay here; loop runs continuously
}
