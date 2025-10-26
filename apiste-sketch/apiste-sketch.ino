// Define pins
#define PIN_WTR 7
#define PIN_BUZZ 13
#define PIN_LED 12
#define PIN_DHT 2     // Digital pin connected to the DHT sensor

// #define HAS_ETHERNET
#define HAS_BUZZER       // Active buzzer
#define HAS_LED          // LED for test
#define HAS_LCD          // LCD screen 1602A V2.0
#define HAS_DHT11        // Temperature and humidity sensor

#ifdef HAS_ETHERNET
  // Add library "ArduinoModbus" by Arduino
  #include <Ethernet.h>
  #include <ArduinoRS485.h>
  // Custom version of the library
  #include <ArduinoModbus.h>
#endif

#ifdef HAS_LCD
  // Add library "LiquidCrystal_I2C" by Martin Kubovcik
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x27, 16, 2); // address, columns, rows
#endif

#ifdef HAS_DHT11
  // Add library "DHT-sensor-library" by Adafruit
  // See latest DHT11 example here:
  // https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHT_Unified_Sensor/DHT_Unified_Sensor.ino
  #include <Adafruit_Sensor.h>
  #include <DHT.h>
  #include <DHT_U.h>
  DHT_Unified dht(PIN_DHT, DHT11);
  uint32_t delayMS;
#endif

// Uncomment if using Arduino Zero
// #define Serial SerialUSB

#ifdef HAS_ETHERNET
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
#endif

// Setup is called on time upon startup
void setup() {
  // 
  #ifdef HAS_LCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Initializing...");
    lcd.setCursor(0, 1);
    // lcd.clear();
  #endif

  // Initialize serial communication with the Arduino (TTL)
  Serial.begin(9600);
  while (!Serial.available()) {
     ; // wait for serial port to connect. Needed for native USB port only
  }

  // Mark program start
  Serial.println("");
  Serial.println(" _____     _ _        _____       _     _         ");
  Serial.println("|  |  |___| | |___   |  _  |___ _| |_ _|_|___ ___ ");
  Serial.println("|     | -_| | | . |  |     |  _| . | | | |   | . |");
  Serial.println("|__|__|___|_|_|___|  |__|__|_| |___|___|_|_|_|___|");
  Serial.println("");

  #ifdef HAS_ETHERNET  
    // Initialize Ethernet shield
    Serial.println("Establishing ethernet connection...");
    // Ethernet.setSubnetMask(subnetMask);
    Ethernet.begin(mac, ip);

    // Check Ethernet Shield board is present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet shield not found.");
      while(true){;} // abort
    }
  #endif

  #ifdef HAS_DHT11
    dht.begin();
    Serial.println(F("DHTxx Unified Sensor Example"));
    // Print temperature sensor details.
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    Serial.println(F("------------------------------------"));
    Serial.println(F("Temperature Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
    Serial.println(F("------------------------------------"));
    // Print humidity sensor details.
    dht.humidity().getSensor(&sensor);
    Serial.println(F("Humidity Sensor"));
    Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
    Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
    Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
    Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
    Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
    Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
    Serial.println(F("------------------------------------"));
    // Set delay between sensor readings based on sensor details.
    delayMS = sensor.min_delay / 1000;
  #endif  
}

void loop() {
  // Read ambient temperature and humidity
  float ambientTemperature = 0;
  float ambientHumidity = 0;
  #ifdef HAS_DHT11
    // Delay between measurements.
    delay(delayMS);  
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    ambientTemperature = event.temperature;
    dht.humidity().getEvent(&event);
    ambientHumidity = event.relative_humidity;
    if (isnan(ambientHumidity) || isnan(ambientTemperature)) {
      Serial.println("Failed to read from DHT sensor!");
      delay(5000);  
      return;
    }
  #endif

  #ifdef HAS_LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(ambientTemperature);
    lcd.print("*C");
    lcd.setCursor(0, 1);
    lcd.print("Humi: ");
    lcd.print(ambientHumidity);
    lcd.print("%");
    delay(2000);
  #endif

  #ifdef HAS_ETHERNET
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
      Serial.println("Water detected!");
      if (on) setOnOff(0);
      alarm(true);
    } else {
      setOnOff(1);
      alarm(false);
      delay(5000);
    }
  #endif

  // Delay
  delay(5000);
}

// Alarm function - LED and active buzzer
void alarm(bool state){
  if (state){
    digitalWrite(PIN_BUZZ, HIGH);
    for (int i=1; i <= 50; i++){
      if (i%2){
        digitalWrite(PIN_LED, HIGH);
      } else {
        digitalWrite(PIN_LED, LOW);
      }
      delay(100);
    }
    digitalWrite(PIN_BUZZ, LOW);
  }
  else {
    digitalWrite(PIN_LED, LOW);
    digitalWrite(PIN_BUZZ, LOW);
  }
}

#ifdef HAS_ETHERNET
  // Function outputs read request status
  void printResult(int result){
    if (result == 0) {
      Serial.println(modbusTCPClient.lastError());
      return;
    }
    Serial.print("Request success. Available to read/write ");
    Serial.print(result);
    Serial.println(" values.");
  }

  // Function waits for Ethernet cable to get connected
  void checkConnectEthernet(){
    if (Ethernet.linkStatus() == LinkOFF){
      Serial.print("Ethernet cable not connected. Connect now.");
    }
    while (Ethernet.linkStatus() == LinkOFF) {
      Serial.print(".");
      delay(1000);
    }
    Serial.println("Ethernet cable is connected.");
  }

  // Function checks id MODBUS is connected. Connects if necessary.
  void checkConnectModbus(){
    // Check if Modbus TCP client is connected
    Serial.print("Modbus TCP client is not conected. Connecting.");  
    while(!modbusTCPClient.connected()){
      modbusTCPClient.stop();
      modbusTCPClient.begin(ipApiste, 502);
      // Delay 10 seconds
      for (int i=0; i<10; i++){
        Serial.print(".");
        delay(1000);
      }
    }
    Serial.println("\nModbus TCP client is connected.");
  }

  double getSetTemperature(){
    // Read Set Temperature (Read Holding Register)
    Serial.println("\nReading Setting Temperature");
    int result = modbusTCPClient.requestFrom(0x01, HOLDING_REGISTERS, 0x0002, 2); // id, type, address, nb - number of values
    printResult(result);
    for (int i=0; i < result-1; i++){
      Serial.print((double)modbusTCPClient.read()/100, 2);
      Serial.println(" C");
    }
  }

  void setSetTemperature(double t){
      // Write Set Temperature
    Serial.println("\nWriting 22 C Set Temperature");
    int result = modbusTCPClient.beginTransmission(0x01, HOLDING_REGISTERS,  0x0002, 2); // int id, int type, int address, int nb
    printResult(result);
    modbusTCPClient.write((unsigned int)t*100);
    modbusTCPClient.endTransmission();
  }

  int getOnOff(){
    // Read ON/OFF Status
    Serial.println("\nReading ON/OFF Status");
    int result = modbusTCPClient.requestFrom(0x01, HOLDING_REGISTERS, 0x2F00, 2); // id, type, address, nb - number of values
    printResult(result);
    int status = 0;
    for (int i=0; i < result-1; i++){
      status = modbusTCPClient.read();
      Serial.println(status==1 ? "ON" : "OFF");
    }
    return status;
  }

  void setOnOff(int onOff){
    // Turn ON Test
    Serial.println("\nWriting OFF Status");
    int result = modbusTCPClient.beginTransmission(0x01, HOLDING_REGISTERS, 0x2F00, 2); // int id, int type, int address, int nb
    printResult(result);
    modbusTCPClient.write(onOff);
    modbusTCPClient.endTransmission();
  }
#endif