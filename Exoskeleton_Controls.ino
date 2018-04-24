#include <SPI.h> // Include SPI Library

/* This code controls the motors through pressing the button,  
  once the button is pressed the motors will spin for a set
  amount of time until the arm reaches the desired position 
*/

/*
//If using EMGs these are the pin initalization for those.

int EMG1 = A9;   //Aduction EMG (located on Anterior Deltoid)
int EMG2 = A8;   //Flexion EMG (located on Medial Deltoid) 
int EMG1var = 0;  //Variable to store sensor value 
int EMG2var = 0;  //Variable to store sensor value
*/

// Button Pins 
const int FB1 = A9;  // Button for Flexion 
const int AB2 = A8;  // Button for Aduction 

// Motor Pins for Flexion
int MD1pos = 9;  // + REF IN
int MD1neg = 10;  // - REF IN
int MD1I = 30;     // Motor Inhibit
int MD1F= 32;      // Motor Fault

// Motor Pins for Abduction
int MD2pos = 11;   // + REF IN
int MD2neg = 12;  // - REF IN
int MD2I = 34;     // Motor Inhibit
int MD2F= 36;      // Motor Fault


// Encoder Pins for SPI Communication
//int MOSI = 51;     // Master (Arduino) Out Slave (Encoder) In Pin 
//int MISO = 50;     // Master (Arduino) In Slave (Encoder) Out Pin 
//int SCK = 52;      // Clock 
int CT1SS = 26;  // Slave Select for Motor 1 (Flexion)
int CT2SS = 24;  // Slave Select for Motor 2 (Abduction) 

void setup() {
/*
  // EMG Pin SetUp
  pinMode(EMG1, INPUT);
  pinMode(EMG2, INPUT);
*/

  // Button Pin Setup
  pinMode(FB1,  INPUT); //Sets the button pins as an input 
  pinMode(AB2, INPUT); //Sets the button pins as an input 
  digitalWrite(FB1, LOW); // Make button condition  
  digitalWrite(AB2, LOW); // Make button condition 

  //Motor Drivers Pin Setup 
  pinMode(MD1pos, OUTPUT);  //Sets the motor pins as an output 
  pinMode(MD1neg, OUTPUT);  //Sets the motor pins as an output 
  pinMode(MD2pos, OUTPUT);  //Sets the motor pins as an output 
  pinMode(MD2neg, OUTPUT);  //Sets the motor pins as an output 

  // SPI Com Pins
  pinMode(CT1SS, OUTPUT); //Sets the SPI Com Pins as outputs 
  pinMode(CT2SS, OUTPUT); //Sets the SPI Com Pins as outputs

  // initialize SPI
  SPI.begin();  //Initializes SPI lines
  Serial.begin(9600);   //Initializes serial monitor 
}

void loop() {
/*
// Reading EMG Values 
  EMG1var = analogRead(EMG1);
  EMG2var = analogRead(EMG2);

// When EMG Signal reaches a certain threshold, the motor will be turned on
 if (EMG1 > 700)  
*/

 if(digitalRead(FB1) == HIGH) // if the button is pressed 
 {
  //The motors run on PWM and are set to be 191 and 0 to create an 80% Duty Cycle 
  analogWrite(MD1pos, 191); 
  analogWrite(MD1neg, 0); 
  //Runs the motor for 1.3 seconds --> time was tested to provide three even increments
  delay(1300);  
  //stops the motors from turning once time the time in time delay passes
  analogWrite(MD1pos, 0); 
  analogWrite(MD1neg, 0); 
 
 }

if(digitalRead(AB2) == HIGH)  // if the button is pressed
{ 
  //The motors run on PWM and are set to be 191 and 0 to create an 80% Duty Cycle 
  analogWrite(MD2pos, 0); 
  analogWrite(MD2neg, 191); 
  //Runs the motor for 1.3 seconds --> time was tested to provide three even increments
  delay(500);
  //stops the motors from turning once time the time in time delay passes
  analogWrite(MD2pos, 0); 
  analogWrite(MD2neg, 0); 
}

  
}
