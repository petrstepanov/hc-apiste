#include <Ethernet.h>
#include <ArduinoRS485.h>

// Custom version of the library
#include <ArduinoModbus.h>

// Define pins
#define WTR 7
#define BUZZ 13
#define LED 12

// Using official Arduino MODBUS example:
// https://github.com/arduino-libraries/ArduinoModbus/blob/master/examples/TCP/EthernetModbusClientToggle/EthernetModbusClientToggle.ino

// Arduino MAC Address (sticker on the back)
byte mac[] = {0xA8, 0x61, 0x0A, 0xAE, 0x34, 0x12};
// Arduino IP address (master, client)
IPAddress ip(192, 168, 0, 2);

// Apiste IP address (slave, server)
IPAddress ipApiste(192, 168, 0, 1);

EthernetClient ethernetClient;
ModbusTCPClient modbusTCPClient(ethernetClient);

// Setup is called on time upon startup
void setup() {
  // Initialize serial communication with the Arduino (TTL)
  
  SerialUSB.begin(9600);
  while (!SerialUSB.available()) {
     ; // wait for serial port to connect. Needed for native USB port only
  }

  // Pass printer to Modbus TCP client for debugging
  // modbusTCPClient.setPrinter(&SerialUSB);

  // Mark program start
  SerialUSB.println("");
  SerialUSB.println(" _____     _ _        _____       _     _         ");
  SerialUSB.println("|  |  |___| | |___   |  _  |___ _| |_ _|_|___ ___ ");
  SerialUSB.println("|     | -_| | | . |  |     |  _| . | | | |   | . |");
  SerialUSB.println("|__|__|___|_|_|___|  |__|__|_| |___|___|_|_|_|___|");
  SerialUSB.println("");
  
  // Initialize Ethernet shield
  SerialUSB.println("Establishing ethernet connection...");
  // Ethernet.setSubnetMask(subnetMask);
  Ethernet.begin(mac, ip);

  // Check Ethernet Shield board is present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    SerialUSB.println("Ethernet shield not found.");
    while(true){;} // abort
  }
}

void loop() {
  // Ensure Ethernet cable is connected
  checkConnectEthernet();

  // Ensure Modbus client is working
  checkConnectModbus();

  // Read set temerature
  int on = getOnOff();

  // Read set temerature
  getSetTemperature();

  // Read water level
  int val = digitalRead(WTR);
  if(val == HIGH){
    SerialUSB.println("Water detected!");
    alarm(true);
    if (on) setOnOff(0);
  } else {
    alarm(false);
    setOnOff(1);
  }  

  // Wait 20 seconds
  delay(5000);
}

// Function waits for Ethernet cable to get connected
void checkConnectEthernet(){
  if (Ethernet.linkStatus() == LinkOFF){
    SerialUSB.print("Ethernet cable not connected. Connect now.");
  }
  while (Ethernet.linkStatus() == LinkOFF) {
    SerialUSB.print(".");
    delay(1000);
  }
  SerialUSB.println("Ethernet cable is connected.");
}

// Function checks id MODBUS is connected. Connects if necessary.
void checkConnectModbus(){
  // Check if Modbus TCP client is connected
  SerialUSB.print("Modbus TCP client is not conected. Connecting.");  
  while(!modbusTCPClient.connected()){
    modbusTCPClient.stop();
    modbusTCPClient.begin(ipApiste, 502);
    // Delay 10 seconds
    for (int i=0; i<10; i++){
      SerialUSB.print(".");
      delay(1000);
    }
  }
  SerialUSB.println("\nModbus TCP client is connected.");
}

// Function outputs read request status
void printResult(int result){
  if (result == 0) {
    SerialUSB.println(modbusTCPClient.lastError());
    return;
  }
  SerialUSB.print("Request success. Available to read/write ");
  SerialUSB.print(result);
  SerialUSB.println(" values.");
}

double getSetTemperature(){
  // Read Set Temperature (Read Holding Register)
  SerialUSB.println("\nReading Setting Temperature");
  int result = modbusTCPClient.requestFrom(0x01, HOLDING_REGISTERS, 0x0002, 2); // id, type, address, nb - number of values
  printResult(result);
  for (int i=0; i < result-1; i++){
    SerialUSB.print((double)modbusTCPClient.read()/100, 2);
    SerialUSB.println(" C");
  }
}

void setSetTemperature(double t){
    // Write Set Temperature
  SerialUSB.println("\nWriting 22 C Set Temperature");
  int result = modbusTCPClient.beginTransmission(0x01, HOLDING_REGISTERS,  0x0002, 2); // int id, int type, int address, int nb
  printResult(result);
  modbusTCPClient.write((unsigned int)t*100);
  modbusTCPClient.endTransmission();
}

int getOnOff(){
  // Read ON/OFF Status
  SerialUSB.println("\nReading ON/OFF Status");
  int result = modbusTCPClient.requestFrom(0x01, HOLDING_REGISTERS, 0x2F00, 2); // id, type, address, nb - number of values
  printResult(result);
  int status = 0;
  for (int i=0; i < result-1; i++){
    status = modbusTCPClient.read();
    SerialUSB.println(status==1 ? "ON" : "OFF");
  }
  return status;
}

void setOnOff(int onOff){
  // Turn ON Test
  SerialUSB.println("\nWriting OFF Status");
  int result = modbusTCPClient.beginTransmission(0x01, HOLDING_REGISTERS, 0x2F00, 2); // int id, int type, int address, int nb
  printResult(result);
  modbusTCPClient.write(onOff);
  modbusTCPClient.endTransmission();
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