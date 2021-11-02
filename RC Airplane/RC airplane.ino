#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#define led 9
RF24 radio(3, 2);   // nRF24L01 (CE, CSN)
const byte address[6] = "00001";
unsigned long lastReceiveTime = 0;
unsigned long currentTime = 0;
Servo throttle;  // create servo object to control the ESC
Servo rudderServo;
Servo elevatorServo;
Servo aileron1Servo;
Servo aileron2Servo;
int throttleValue, rudderValue, elevatorValue, aileron1Value, aileron2Value, travelAdjust;
// Max size of this struct is 32 bytes - NRF24L01 buffer limit
struct Data_Package {
  byte j1PotX;
  byte j1PotY;
  byte j1Button;
  byte j2PotX;
  byte j2PotY;
  byte j2Button;
  byte pot1;
  byte pot2;
  byte tSwitch1;
  byte tSwitch2;
  byte button1;
  byte button2;
  byte button3;
  byte button4;
};
Data_Package data; //Create a variable with the above structure
void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.startListening(); //  Set the module as receiver
  resetData();
  throttle.attach(10);
  rudderServo.attach(4);   // CH1
  elevatorServo.attach(5); // CH2
  aileron1Servo.attach(6); // CH3
  aileron2Servo.attach(7); // CH4
  pinMode(led, OUTPUT);    // CH6
}
void loop() {
  // Check whether we keep receving data, or we have a connection between the two modules
  currentTime = millis();
  if ( currentTime - lastReceiveTime > 1000 ) { // If current time is more then 1 second since we have recived the last data, that means we have lost connection
    resetData(); // If connection is lost, reset the data. It prevents unwanted behavior, for example if a drone jas a throttle up, if we lose connection it can keep flying away if we dont reset the function
  }
  // Check whether there is data to be received
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package)); // Read the whole data and store it into the 'data' structure
    lastReceiveTime = millis(); // At this moment we have received the data
  }
  // Controlling throttle - brushless motor with ESC
  throttleValue = constrain(data.j1PotY, 80, 255); // Joysticks stays in middle. So we only need values the upper values from 130 to 255
  throttleValue = map(throttleValue, 80, 255, 1000, 2000);
  throttle.writeMicroseconds(throttleValue);
  // Adjusting the servos responsiveness
  travelAdjust = map(data.pot2, 0, 255, 0, 25);  
  
  // Elevator control
  elevatorValue = map(data.j2PotY, 0, 255, (85 - travelAdjust), (35 + travelAdjust));
  elevatorServo.write(elevatorValue);
  
  // Ailerons control
  aileron1Value = map(data.j2PotX, 0, 255, (10 + travelAdjust), (80 - travelAdjust));
  aileron1Servo.write(aileron1Value);
  aileron2Servo.write(aileron1Value);
  // Rudder trimming function
  if (data.j1PotX > 127) {
    rudderValue = data.pot1 + (data.j1PotX - 127);
  }
  if (data.j1PotX < 127) {
    rudderValue = data.pot1 - (127 - data.j1PotX);
  }
  // Rudder control
  rudderValue = map(rudderValue, 0, 255, (10 + travelAdjust), (90 - travelAdjust));
  rudderServo.write(rudderValue);
  // Monitor the battery voltage
  int sensorValue = analogRead(A3);
  float voltage = sensorValue * (5.00 / 1023.00) * 3; // Convert the reading values from 5v to suitable 12V i
  // If voltage is below 11V turn on the LED
  if (voltage < 11) {
    digitalWrite(led, HIGH);
  }
  else {
    digitalWrite(led, LOW);
  }
}
void resetData() {
  // Reset the values when there is no radio connection - Set initial default values
  data.j1PotX = 127;
  data.j1PotY = 80; // Motors stops // the central point of the joystick is not starting point for the throttle, its at value of 80 instead of 127
  data.j2PotX = 127;
  data.j2PotY = 127;
  data.j1Button = 1;
  data.j2Button = 1;
  data.pot1 = 1;
  data.pot2 = 1;
  data.tSwitch1 = 1;
  data.tSwitch2 = 1;
  data.button1 = 1;
  data.button2 = 1;
  data.button3 = 1;
  data.button4 = 1;
}