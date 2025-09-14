#include <MultitapKeypad.h>

// Pin definitions for the 4x4 keypad matrix
const byte ROW0 = 4;
const byte ROW1 = 5;
const byte ROW2 = 6;
const byte ROW3 = 7;
const byte COL0 = 15;
const byte COL1 = 16;
const byte COL2 = 17;
const byte COL3 = 18;
// const byte BEEP = 2; // Pin for the buzzer

// Create kpd as a MultitapKeypad object for the 4x4 keypad
MultitapKeypad kpd(ROW0, ROW1, ROW2, ROW3, COL0, COL1, COL2, COL3);

// Create key as a Key object to hold the state of the key being pressed
Key key;

// Global variables to store the input
int berat = 0;
String inputString = "";

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  Serial.println("Keypad Number Input");
  Serial.println("--------------------");
  Serial.println("Enter a number using the keypad.");
  Serial.println("Press # to save the value.");
  Serial.println("Press * to clear the input.");
  Serial.println();
}

void loop() {
  // Get the key being pressed
  key = kpd.getKey();

  // Check if a key is pressed (not NO_KEY)
  if (key.code != NO_KEY) {
    // Check if the key is a digit (0-9)
    if (key.character >= '0' && key.character <= '9') {
      // Append the digit to the input string
      inputString += key.character;
      Serial.print("Current Input: ");
      Serial.println(inputString);
      // tone(BEEP, 5000, 20); // Play a short beep for confirmation
    }
    // Check if the '#' key is pressed
    else if (key.character == KEY_DOWN) {
      if (inputString.length() > 0) {
        // Convert the string to an integer and store it in 'berat'
        berat = inputString.toInt();
        Serial.print("Value saved! The variable 'berat' is now: ");
        Serial.println(berat);
        // Clear the input string for the next entry
        inputString = "";
        // tone(BEEP, 4000, 100); // Play a longer beep for a successful save
      } else {
        Serial.println("No input to save.");
      }
    }
    // Check if the '*' key is pressed to clear the input
    else if (key.character == KEY_ASTERISK) {
      inputString = "";
      Serial.println("Input has been cleared.");
      // tone(BEEP, 4000, 50); // Play a short beep for a reset
    }
  }
}