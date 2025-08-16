// Arduino adds main() at compile time:
// Snippet below:
/*
int main(void){
	setup();
	for (;;) loop();
	return 0;
}
*/

#define SerialUSB Serial

// Required libraries
// #include <SoftwareSerialUSB.h>

#include <SPI.h>
#include <Ethernet.h>       // Ethernet library v2 is required
#include <ModbusEthernet.h>

// Arduino Zero only - fix Sserial Output
// #define Serial SerialUSB

// Modbus - Unit Id
#define SERVER_ADDRESS 0x0001
#define NUM_REGISTERS 1

// Define pins
#define RX 9
#define TX 8
#define LED 12
#define WTR 7
#define BUZZ 13

// Instantiate global variables
// https://github.com/emelianov/modbus-esp8266/blob/master/examples/TCP-Ethernet/client/client.ino
const uint16_t REG = 512;         // Modbus Hreg Offset
IPAddress remote(192, 168, 1, 1); // Address of Modbus Slave device
const int32_t showDelay = 5000;   // Show result every n'th mellisecond

// Enter a MAC address and IP address for your controller below.
byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0x34, 0x12};
IPAddress ip(192, 168, 1, 2);     // The IP address will be dependent on your local network:
ModbusEthernet mb;                // Declare ModbusTCP instance

// Setup is called on time upon startup
void setup() {
  // Initialize serial communication with the Arduino (TTL)
  SerialUSB.begin(9600);
  while (!SerialUSB.available()) {
     ; // wait for serial port to connect. Needed for native USB port only
  }

  // Initialize MAX3232 RS232 COM port communication
  // mySerialUSB.begin(9600);

  // Configure LED pin
  pinMode(LED, OUTPUT);    // Configures the LED pin to OUTPUT voltage
  digitalWrite(LED, LOW);  // Output HIGH (5V) voltage to LED

  // Configure active buzzer pin
  pinMode(BUZZ, OUTPUT);    // Configures the LED pin to OUTPUT voltage
  digitalWrite(BUZZ, LOW);  // Output HIGH (5V) voltage to LED

  // Configure Modbus Client (master)
  SerialUSB.println("Starting Ethernet Modbus TCP Client");

  Ethernet.init(5);         // SS pin
  Ethernet.begin(mac, ip);  // start the Ethernet connection
  delay(1000);              // give the Ethernet shield a second to initialize
  mb.client();              // Act as Modbus TCP client
}

uint16_t res = 0;
uint32_t showLast = 0;

// Loop is the main loop body
void loop() {
  SerialUSB.println("");

  // Read water level
  int val = digitalRead(WTR);
  // SerialUSB.println(val);

  if(val == LOW){
    SerialUSB.println("Dry");
    alarm(false);
  } else {
    SerialUSB.println("Wet");
    alarm(true);

    // TODO: Turn off the AC
    // mySerialUSB...
  }       

  // Test - read set temperature - not works
if (mb.isConnected(remote)) {   // Check if connection to Modbus Slave is established
    SerialUSB.println("Modbus connected");
    // TYPEID id, uint16_t offset, uint16_t* value, uint16_t numregs = 1, cbTransaction cb = nullptr, uint8_t unit = MODBUSIP_UNIT
    mb.readHreg(remote, REG, &res);  // Initiate Read Hreg from Modbus Slave
  } else {
    SerialUSB.println("Modbus not connected. Connecting...");
    mb.connect(remote, 502);           // Try to connect if not connected
  }
  delay(5000);                     // Pulling interval
  mb.task();                      // Common local Modbus task
  if (millis() - showLast > showDelay) { // Display register value every 5 seconds (with default settings)
    showLast = millis();
    SerialUSB.println(res);
  }

  // Wait 5 seconds
  // delay(5000);
}

// Alarm function - LED and active buzzer
void alarm(bool state){
  if (state){
    digitalWrite(BUZZ, HIGH);
    for (int i=1; i <= 10; i++){
      if (i%2){
        digitalWrite(LED, HIGH);
      } else {
        digitalWrite(LED, LOW);
      }
      delay(100);
    }
    digitalWrite(BUZZ, LOW);
  }
  else {
    digitalWrite(LED, LOW);
    digitalWrite(BUZZ, LOW);
  }
}