// Arduino adds main() at compile time:
// Snippet below:
/*
int main(void){
	setup();
	for (;;) loop();
	return 0;
}
*/

// Required libraries
#include <SoftwareSerial.h>

// Define pins
#define RX 9
#define TX 8
#define LED 12
#define WTR 7
#define BUZZ 13

// Instantiate global variables
SoftwareSerial mySerial(RX, TX); // receivePin, transmitPin

// Setup is called on time upon startup
void setup() {
  // Initialize serial communication with the Arduino (TTL)
  Serial.begin(9600);
  Serial.println("Setup");

  // Initialize MAX3232 RS232 COM port communication
  mySerial.begin(9600);

  // Configure LED pin
  pinMode(LED, OUTPUT);    // Configures the LED pin to OUTPUT voltage
  digitalWrite(LED, LOW);  // Output HIGH (5V) voltage to LED

  // Configure active buzzer pin
  pinMode(BUZZ, OUTPUT);    // Configures the LED pin to OUTPUT voltage
  digitalWrite(BUZZ, LOW);  // Output HIGH (5V) voltage to LED
}

// Loop is the main loop body
void loop() {
  Serial.println("Loop start");
  
  int val = digitalRead(WTR);
  Serial.println(val);

  if(val == LOW){
    Serial.println("Dry");
    alarm(false);
  } else {
    Serial.println("Wet");
    alarm(true);

    // TODO: Turn off the AC
    // mySerial...
  }

  // Wait 1 second
  delay(1000);
}

// Alarm function - LED and active buzzer
void alarm(bool state){
  bool HI=0x1;
  bool LO=0x0;
  if (state){
    for (int i=1; i <= 10; i++){
      if (i%2){
        digitalWrite(LED, HI);
        digitalWrite(BUZZ, HI);
      } else {
        digitalWrite(LED, LO);
        digitalWrite(BUZZ, LO);
      }
      delay(100);
    }
  }
  else {
    digitalWrite(LED, LOW);
    digitalWrite(BUZZ, LOW);
  }
}