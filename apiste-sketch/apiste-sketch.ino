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
// #include <SoftwareSerialUSB.h>

#include <Ethernet.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>

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

// Using official Arduino MODBUS example:
// https://github.com/arduino-libraries/ArduinoModbus/blob/master/examples/TCP/EthernetModbusClientToggle/EthernetModbusClientToggle.ino

// Arduino MAC Address (sticker on the back)
byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0x34, 0x12};
// Arduino IP address (master, client)
IPAddress ip(192, 168, 1, 2);

// Subnet mask
IPAddress subnet(255, 255, 255, 0);

EthernetClient ethernetClient;
ModbusTCPClient modbusTCPClient(ethernetClient);

// Apiste IP address (slave, server)
IPAddress ipApiste(192, 168, 1, 1);

// Setup is called on time upon startup
void setup() {
  // Initialize serial communication with the Arduino (TTL)
  SerialUSB.begin(9600);
  while (!SerialUSB.available()) {
     ; // wait for serial port to connect. Needed for native USB port only
  }

  // Mark program start
  SerialUSB.println(" _____     _ _        _____       _     _         ");
  SerialUSB.println("|  |  |___| | |___   |  _  |___ _| |_ _|_|___ ___ ");
  SerialUSB.println("|     | -_| | | . |  |     |  _| . | | | |   | . |");
  SerialUSB.println("|__|__|___|_|_|___|  |__|__|_| |___|___|_|_|_|___|)");
  SerialUSB.println("");
  
  // Initialize MAX3232 RS232 COM port communication
  // mySerialUSB.begin(9600);

  // Configure LED pin
  pinMode(LED, OUTPUT);    // Configures the LED pin to OUTPUT voltage
  digitalWrite(LED, LOW);  // Output HIGH (5V) voltage to LED

  // Configure active buzzer pin
  pinMode(BUZZ, OUTPUT);    // Configures the LED pin to OUTPUT voltage
  digitalWrite(BUZZ, LOW);  // Output HIGH (5V) voltage to LED

  // Initialize Ethernet shield
  SerialUSB.println("Establishing ethernet connection...");
  Ethernet.setSubnetMask(subnet);
  Ethernet.begin(mac, ip);

  // Check Ethernet hardware is present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    SerialUSB.println("Ethernet shield not found");
    while(true){;} // software abort, no point running without Ethernet hardware
  }

  // Check Ethernet cable is connected
  if (Ethernet.linkStatus() == LinkOFF) {
    SerialUSB.println("Ethernet cable not connected");
  }

  // Check and establish Modbus client connection
  // checkConnectModbus();
}

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
  // int temp = modbusTCPClient.inputRegisterRead(1, 0x0002);
  // SerialUSB.print("Set temperature: ");
  // SerialUSB.println(temp);

  // From the AC documentation 
  // int transactionId = 0x0001; // A
  // int protocolId = 0x0000; // B
  // int messageLength = 4; // C - bytes after unitId
  // int unitId = 1; // D - on the Ac preferences set to 1
  // int functionCode = 0x03; // E - 0x03 for reading, 0x10 for writing to AC
  // int registerAddress = 0x0002; // F - from AC PDF Table - current tempearture
  // int registerQuantity = 1;// G - Specifies the quantity of registers to read or write.
  
  // Ensure Modbus client is working
  checkConnectModbus();
  
  // Try stopping the AC
  SerialUSB.println("Stopping the AC - write 0x00 to 0x2F00");
  int result = modbusTCPClient.coilWrite(1, 0x2F00, 0x00); // int id, int address, uint8_t value
  SerialUSB.print("Result is ");
  SerialUSB.println(result);
  if (!result) {
      SerialUSB.print("Failed to write coil - ");
      SerialUSB.println(modbusTCPClient.lastError());
  }

  // Try reading something - not works
  /*
  SerialUSB.println("Reading Input (R) or Holding (R/W) Registers:");
  modbusTCPClient.requestFrom(SLAVE_ADDRESS, INPUT_REGISTERS, 0x0000, NUM_REGISTERS);
  int nValues = modbusTCPClient.available();
  SerialUSB.print("Available values: ");
  SerialUSB.println(nValues);
  for(int i = 0; i < nValues; i++){
    long data = modbusTCPClient.read();
    SerialUSB.println(data);
  }
  SerialUSB.println(modbusTCPClient.lastError());
  */
  // modbusTCPClient.requestFrom(0x0000, COILS, 0x0000, NUM_REGISTERS); // connection timed out - maybe wrong address? trying 0x0001
  // modbusTCPClient.requestFrom(0x0001 COILS, 0x0000, NUM_REGISTERS); // illegal function
  // modbusTCPClient.requestFrom(0x0001, DISCRETE_INPUTS, 0x0000, NUM_REGISTERS); // illegal function

  // long t = modbusTCPClient.inputRegisterRead(SLAVE_ADDRESS, 0x0100);
  // SerialUSB.println(t);
  // SerialUSB.println(modbusTCPClient.lastError());

  // Try writing something - start the AC
  // modbusTCPClient.holdingRegisterWrite(SERVER_ADDRESS, 0x2F00, 1); // illegal function 
  // modbusTCPClient.holdingRegisterWrite(SERVER_ADDRESS, 0x2F00, 1); // illegal function 
  // SerialUSB.println(modbusTCPClient.lastError());

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

// Function checks if Modbus is connected
void checkConnectModbus(){
  // Check if Modbus TCP client is connected
  while(!modbusTCPClient.connected()){
    SerialUSB.println("\nModbus TCP client is not conected");
    modbusTCPClient.begin(ipApiste, 502);
    // Delay 10 seconds
    SerialUSB.print("Connecting.");
    for (int i=0; i<10; i++){
      SerialUSB.print(".");
      delay(500);
    }
  }
  SerialUSB.println("\nModbus TCP client is connected");
}