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
// #include <SoftwareSerial.h>

#include <Ethernet.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>

// Arduino Zero only
#define Serial SerialUSB

// Modbus - Unit Id
#define SLAVE_ADDRESS 1
#define NUM_REGISTERS 1

// Define pins
#define RX 9
#define TX 8
#define LED 12
#define WTR 7
#define BUZZ 13

// Instantiate global variables
// SoftwareSerial mySerial(RX, TX); // receivePin, transmitPin

// Enter a MAC address for your controller below.
// Newer Ethernet shields have a MAC address printed on a sticker on the shield
// The IP address will be dependent on your local network:
// https://github.com/arduino-libraries/ArduinoModbus/blob/master/examples/TCP/EthernetModbusClientToggle/EthernetModbusClientToggle.ino
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 1, 2);   // Static IP assigned to the Arduino - master, client.

EthernetClient ethernetClient;
ModbusTCPClient modbusTCPClient(ethernetClient);

IPAddress server(192, 168, 1, 1); // IP Address of your Modbus server - the AC (slave, server)

// EthernetServer ethServer(502);  // Port 502 should match AC's port
// ModbusTCPServer modbusTCPServer;

// Modbus global variables
// https://github.com/CONTROLLINO-PLC/CONTROLLINO_Library/tree/6ed585278ac7ea3c43b7daaabec5e6362125f7f5/examples/Expand_Modbus/TCP
// byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
// IPAddress ip(192, 168, 1, 1);       
// EthernetClass ethernetClient;
// ModbusTCPClient modbusTCPClient(ethernetClient);
// IPAddress server(192, 168, 1, 2);  // IP address of the Modbus slave device (AC).

// Setup is called on time upon startup
void setup() {
  // Initialize serial communication with the Arduino (TTL)
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Initialize MAX3232 RS232 COM port communication
  // mySerial.begin(9600);

  // Configure LED pin
  pinMode(LED, OUTPUT);    // Configures the LED pin to OUTPUT voltage
  digitalWrite(LED, LOW);  // Output HIGH (5V) voltage to LED

  // Configure active buzzer pin
  pinMode(BUZZ, OUTPUT);    // Configures the LED pin to OUTPUT voltage
  digitalWrite(BUZZ, LOW);  // Output HIGH (5V) voltage to LED

  // Configure Modbus Client (master)
  Serial.println("Starting Ethernet Modbus TCP Client");

  Ethernet.init(ETHERNET_CS);
  Ethernet.begin(mac, ip);

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield not found");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable not connected");
  }
}

// Loop is the main loop body
void loop() {
  Serial.println("Loop start");

  // Read water level
  int val = digitalRead(WTR);
  // Serial.println(val);

  if(val == LOW){
    Serial.println("Dry");
    alarm(false);
  } else {
    Serial.println("Wet");
    alarm(true);

    // TODO: Turn off the AC
    // mySerial...
  }       

  // Modbus test
  if (!modbusTCPClient.connected()) {
    Serial.println("Attempting to connect to Modbus TCP server");
    
    if (!modbusTCPClient.begin(server, 502)) {
      Serial.println("Modbus TCP Client failed to connect");
    } else {
      Serial.println("Modbus TCP Client connected successfully");
    }
  }

  // Test - read set temperature - not works
  // int temp = modbusTCPClient.inputRegisterRead(1, 0x0002);
  // Serial.print("Set temperature: ");
  // Serial.println(temp);

  // From the AC documentation 
  int transactionId = 0x0001; // A
  int protocolId = 0x0000; // B
  int messageLength = 4; // C - bytes after unitId
  int unitId = 1; // D - on the Ac preferences set to 1
  int functionCode = 0x03; // E - 0x03 for reading, 0x10 for writing to AC
  int registerAddress = 0x0002; // F - from AC PDF Table - current tempearture
  int registerQuantity = 1;// G - Specifies the quantity of registers to read or write.
  

  Serial.println("Reading Input (R) or Holding (R/W) Registers:");
  modbusTCPClient.requestFrom(1, INPUT_REGISTERS, 0x0100, 1);
  int nValues = modbusTCPClient.available();
  Serial.print("Available values: ");
  Serial.println(nValues);
  for(int i = 0; i < nValues; i++){
    long data = modbusTCPClient.read();
    Serial.println(data);
  }
  Serial.println(modbusTCPClient.lastError());
  // modbusTCPClient.requestFrom(0x0000, COILS, 0x0000, NUM_REGISTERS); // connection timed out - maybe wrong address? trying 0x0001
  // modbusTCPClient.requestFrom(0x0001 COILS, 0x0000, NUM_REGISTERS); // illegal function
  // modbusTCPClient.requestFrom(0x0001, DISCRETE_INPUTS, 0x0000, NUM_REGISTERS); // illegal function

  // long t = modbusTCPClient.inputRegisterRead(1, 0x0000);
  // Serial.println(t);

// (1, int type, int address, int nb);
    //   Serial.println(modbusTCPClient.lastError());
    // }

    // // write the value of 0x01, to the coil at address 0x00
    // if (!modbusTCPClient.coilWrite(0x00, 0x01)) {
    //   Serial.print("Failed to write coil! ");
    //   Serial.println(modbusTCPClient.lastError());
    // }
  // }  

  // Wait 5 seconds
  delay(5000);
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