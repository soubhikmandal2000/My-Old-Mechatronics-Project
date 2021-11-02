#include <LiquidCrystal.h> // includes the LiquidCrystal Library
#include <Servo.h>
LiquidCrystal lcd(27, 26, 25, 24, 23, 22); // Creates an LC object. Parameters: (rs, enable, d4, d5, d6, d7)
Servo servo1, servo2, servo3, servo4;   // DS04-NFC motors
// Stepper motors pins
#define dirPinVertical 0
#define stepPinVertical 1
#define dirPinHorizontal 2
#define stepPinHorizontal 3
#define coinDetector 9
#define button1 13
#define button2 12
#define button3 11
#define button4 10
#define microSwitchV 15
#define microSwitchH 14
int buttonPressed;
void setup() {
  lcd.begin(16, 2); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display
  servo1.attach(4);
  servo2.attach(5);
  servo3.attach(6);
  servo4.attach(7);
  pinMode(dirPinVertical, OUTPUT);
  pinMode(stepPinVertical, OUTPUT);
  pinMode(dirPinHorizontal, OUTPUT);
  pinMode(stepPinHorizontal, OUTPUT);
  pinMode(coinDetector, INPUT);
  // Activating the digital pins pull up resistors
  pinMode(button1, INPUT_PULLUP);
  pinMode(button2, INPUT_PULLUP);
  pinMode(button3, INPUT_PULLUP);
  pinMode(button4, INPUT_PULLUP);
  pinMode(microSwitchV, INPUT_PULLUP);
  pinMode(microSwitchH, INPUT_PULLUP);
  // Vertical starting position
  digitalWrite(dirPinVertical, HIGH); // Set the stepper to move in a particular direction
  while (true) {
    if (digitalRead(microSwitchV) == LOW) { // If the micro switch is pressed, move the platfor a little bit up and exit the while loop
      moveUp(70);
      break;
    }
    // Move the carrier up until the micro switch is pressed
    digitalWrite(stepPinVertical, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPinVertical, LOW);
    delayMicroseconds(300);
  }
  // Horizontal starting position
  digitalWrite(dirPinHorizontal, LOW);
  while (true) {
    if (digitalRead(microSwitchH) == LOW) {
      moveLeft(350);
      break;
    }
    digitalWrite(stepPinHorizontal, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPinHorizontal, LOW);
    delayMicroseconds(300);
  }
}
void loop() {
  // Print "Insert a coin!" on the LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Insert a coin!");
  
  // Wait until a coin is detected
  while (true) {
    if (digitalRead(coinDetector) == LOW) { // If a coin is detected, exit the from the while loop
      break;
    }
  }
  
  delay(10);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Select your item");
  lcd.setCursor(0, 1);
  lcd.print(" 1, 2, 3 or 4?");
  
  // Wait until a button is pressed
  while (true) {
    if (digitalRead(button1) == LOW) {
      buttonPressed = 1;
      break;
    }
    if (digitalRead(button2) == LOW) {
      buttonPressed = 2;
      break;
    }
    if (digitalRead(button3) == LOW) {
      buttonPressed = 3;
      break;
    }
    if (digitalRead(button4) == LOW) {
      buttonPressed = 4;
      break;
    }
  }
  
  // Print "Delivering..." 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Delivering...");
  
  // Depending on the pressed button, move the carrier to that position and discharge the selected item 
  switch (buttonPressed) {
    case 1:
      // Move the container to location 1
      moveUp(4900); // Move up 4900 steps (Note: the stepper motor is set in Quarter set resolution)
      delay(200);
      moveLeft(1700); // Move left 1700 steps
      delay(300);
      // Rotate the helical coil, discharge the selected item
      servo1.writeMicroseconds(2000); // rotate
      delay(950);
      servo1.writeMicroseconds(1500);  // stop
      delay(500);
      // Move the container back to starting position
      moveRight(1700);
      delay(200);
      moveDown(4900);
      break;
      
     case 2:
      // Move the container to location 2
      moveUp(4900);
      delay(200);
      // Rotate the helix, push the selected item
      servo2.writeMicroseconds(2000); // rotate
      delay(950);
      servo2.writeMicroseconds(1500);  // stop
      delay(500);
      moveDown(4900);
      break;
      case 3:
      // Move the container to location 3
      moveUp(2200); 
      delay(200);
      moveLeft(1700);
      delay(300);
      // Rotate the helix, push the selected item
      servo3.writeMicroseconds(2000); // rotate
      delay(950);
      servo3.writeMicroseconds(1500);  // stop
      delay(500);
      // Move the container back to starting position
      moveRight(1700);
      delay(200);
      moveDown(2200);
      break;
      case 4:
      // Move the container to location 4
      moveUp(2200); // Move verticaly 4800 steps
      delay(200);
      // Rotate the helix, push the selected item
      servo4.writeMicroseconds(2000); // rotate
      delay(950);
      servo4.writeMicroseconds(1500);  // stop
      delay(500);
      moveDown(2200);
      break;
  }
  
  lcd.clear(); // Clears the display
  lcd.setCursor(0, 0);
  lcd.print("Item delivered!"); // Prints on the LCD
  delay(2000);
}
// == Custom functions ==
void moveUp (int steps) {
  digitalWrite(dirPinVertical, LOW);
  for (int x = 0; x < steps; x++) {
    digitalWrite(stepPinVertical, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPinVertical, LOW);
    delayMicroseconds(300);
  }
}
void moveDown (int steps) {
  digitalWrite(dirPinVertical, HIGH);
  for (int x = 0; x < steps; x++) {
    digitalWrite(stepPinVertical, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPinVertical, LOW);
    delayMicroseconds(300);
  }
}
void moveLeft (int steps) {
  digitalWrite(dirPinHorizontal, HIGH);
  for (int x = 0; x < steps; x++) {
    digitalWrite(stepPinHorizontal, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPinHorizontal, LOW);
    delayMicroseconds(300);
  }
}
void moveRight (int steps) {
  digitalWrite(dirPinHorizontal, LOW);
  for (int x = 0; x < steps; x++) {
    digitalWrite(stepPinHorizontal, HIGH);
    delayMicroseconds(300);
    digitalWrite(stepPinHorizontal, LOW);
    delayMicroseconds(300);
  }
}