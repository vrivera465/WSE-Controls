/* This code to test to make sure the motors are running 
   when attached to the circuit board. It runs each motor 
   back and forth for a set amount of time and constantly loops.
*/

// Motor Pins for Flexion
int MD1pos = 9;  // + REF IN
int MD1neg = 10;  // - REF IN

// Motor Pins for Abduction
int MD2pos = 11;   // + REF IN
int MD2neg = 12;  // - REF IN

void setup() {
  //Sets the motor pins to be outputs
  pinMode(MD1pos, OUTPUT);  
  pinMode(MD1neg, OUTPUT);
  pinMode(MD2pos, OUTPUT);
  pinMode(MD2neg, OUTPUT);
}

void loop() {
  //Runs the first motor
  analogWrite(MD1pos, 191); 
  analogWrite(MD1neg, 0); 
  //Runs the second motor 
  analogWrite(MD2pos, 191); 
  analogWrite(MD2neg, 0); 
  //Runs each motor for 4 seconds 
  delay (4000);

  //Stops both motors 
  analogWrite(MD1pos, 0); 
  analogWrite(MD1neg, 0); 
  analogWrite(MD2pos, 0); 
  analogWrite(MD2neg, 0); 
  //Keeps motors off for 4 seconds
  delay(4000);
  
}
