/*Code to test if the button is working
Uses the Serial Monitor to print our 
different statements if the button is pushed
*/


// Button Pins 
const int FB1 = A9;  // Button for Flexion 
const int AB2 = A8;  // Button for Aduction 

void setup() {
 Serial.begin(9600); //Initializes the serial monitor

  // Button Pin Setup
  pinMode(FB1,  INPUT); //Sets this pin as an input 
  pinMode(AB2, INPUT);  //Sets this pin as an input 

}

void loop() {
  
 if(digitalRead(FB1) == HIGH) // if the button is pressed 
 {
  Serial.print("flex"); //Prints the work flex
 }

 
if(digitalRead(AB2) == HIGH)  //if the button is pressed 
{ 
  Serial.print("add"); //Prints the word add
}


}
