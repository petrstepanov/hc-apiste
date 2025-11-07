// Define pins
#define PIN_WTR 7
#define PIN_BUZZ 3
#define PIN_LED_RED 12
#define PIN_LED_GREEN 11
#define PIN_LED_YELLOW 9 // Pin 10 conflicts with Eth Shield: https://docs.arduino.cc/libraries/ethernet/
#define PIN_DHT 2     // Digital pin connected to the DHT sensor

#define HAS_ETHERNET
#define HAS_LED          // LED for test
#define HAS_LCD          // LCD screen 1602A V2.0
#define HAS_DHT11        // Temperature and humidity sensor
#define HAS_WTR
#define HAS_BUZZ
// #define HAS_SERIAL

#ifdef HAS_ETHERNET
  // Add library "ArduinoModbus" by Arduino
  #include <Ethernet.h>
  #include <ArduinoRS485.h>
  // Custom version of the library
  #include <ArduinoModbus.h>
#endif

#ifdef HAS_LCD
  #include <Wire.h>
  #include <hd44780.h>
  #include <hd44780ioClass/hd44780_I2Cexp.h>
  hd44780_I2Cexp lcd;
  const int LCD_COLS = 16;
  const int LCD_ROWS = 2;
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

#ifdef HAS_LCD
  void printLCD(char string[], bool clear = true){
    if (clear) lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(string);
  }

  void printLCD(char string1[], char string2[]){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(strlen(string1) != 0 ? string1 : "                ");
    lcd.setCursor(0, 1);
    lcd.print(string2);
  }
#endif

// Setup is called on time upon startup
void setup() {
  // Configure Pin Modes
  pinMode(PIN_WTR, INPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_DHT, INPUT);


  digitalWrite(PIN_LED_YELLOW, HIGH);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_RED, LOW);

  // LCD - could not get to work
  #ifdef HAS_LCD
    int status;
    status = lcd.begin(LCD_COLS, LCD_ROWS);
    if (status){
      #ifdef HAS_SERIAL
        Serial.println("LCD Error");
        Serial.println(status);
      #endif
      hd44780::fatalError(status); // does not return
    }
	  // Print a message to the LCD
    #ifdef HAS_SERIAL
      Serial.println("LCD OK");
    #endif
	  lcd.print("Initializing...");
  #endif

  // Initialize serial communication with the Arduino (TTL)
  #ifdef HAS_SERIAL
    Serial.begin(9600);
    while (!Serial.available()) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  #endif

  // Mark program start
  #ifdef HAS_SERIAL
    Serial.println("");
    Serial.println(" _____     _ _        _____       _     _         ");
    Serial.println("|  |  |___| | |___   |  _  |___ _| |_ _|_|___ ___ ");
    Serial.println("|     | -_| | | . |  |     |  _| . | | | |   | . |");
    Serial.println("|__|__|___|_|_|___|  |__|__|_| |___|___|_|_|_|___|");
    Serial.println("");
  #endif

  #ifdef HAS_ETHERNET  
    // Initialize Ethernet shield
    #ifdef HAS_SERIAL
      Serial.println("Establishing ethernet connection...");
    #endif
    // Ethernet.setSubnetMask(subnetMask);
    Ethernet.begin(mac, ip);

    // Check Ethernet Shield board is present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      #ifdef HAS_SERIAL
        Serial.println("Ethernet shield not found.");
      #endif
      #ifdef HAS_LCD
        printLCD("Ethernet shield", "not found - halt");
      #endif
      while(true){;} // abort
    }
  #endif

  #ifdef HAS_DHT11
    dht.begin();
    // Print temperature sensor details.
    sensor_t sensor;
    dht.temperature().getSensor(&sensor);
    #ifdef HAS_SERIAL
      Serial.println(F("------------------------------------"));
      Serial.println(F("Temperature Sensor"));
      Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
      Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
      Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
      Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
      Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
      Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
      Serial.println(F("------------------------------------"));
    #endif
    // Print humidity sensor details.
    dht.humidity().getSensor(&sensor);
    #ifdef HAS_SERIAL
      Serial.println(F("Humidity Sensor"));
      Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
      Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
      Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
      Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
      Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
      Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
      Serial.println(F("------------------------------------"));
    #endif
    // Set delay between sensor readings based on sensor details.
    delayMS = sensor.min_delay / 1000;
  #endif

  // Fade booting yellow pin
  digitalWrite(PIN_LED_YELLOW, LOW);
}

void loop() {
  // First check for water and alarm if overflow
  int wtrVal = LOW;
  #ifdef HAS_WTR
    wtrVal = digitalRead(PIN_WTR);
    if(wtrVal == HIGH){
      #ifdef HAS_SERIAL
        Serial.println("Wet!");
      #endif
      #ifdef HAS_LCD
        printLCD("Water overflow!", "Empty tank now!");
      #endif
      alarm(true);
    } else {
      #ifdef HAS_LCD
        printLCD("Water level OK", ":-)");
      #endif
      #ifdef HAS_SERIAL
        Serial.println("Dry.");
      #endif
      alarm(false);
      delay(3000);
    }
  #endif

  // Read ambient temperature and humidity (skip in case of the spill)
  float ambientTemperature = 0;
  float ambientHumidity = 0;
  if (wtrVal != HIGH){
    #ifdef HAS_DHT11
      // Delay between measurements.
      delay(delayMS);  
      sensors_event_t event;
      dht.temperature().getEvent(&event);
      ambientTemperature = event.temperature;
      dht.humidity().getEvent(&event);
      ambientHumidity = event.relative_humidity;
      if (isnan(ambientHumidity) || isnan(ambientTemperature)) {
        #ifdef HAS_SERIAL
          Serial.println("Failed to read from DHT sensor!");
        #endif
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
      delay(3000);
    #endif
  }

  #ifdef HAS_ETHERNET
    // Ensure Ethernet cable is connected
    checkConnectEthernet();

    // Ensure Modbus client is working
    checkConnectModbus();

    // Read set temerature
    int on = getOnOff();

    if(wtrVal == HIGH){
      // Leak - turn off and skip delay (alarm instant)
      if (on) setOnOff(0);
      return;
    } else {
      // No leak - turn on and set temperature
      if (!on) setOnOff(1);
      if (ambientTemperature != 0) setSetTemperature(ambientTemperature);
    }
  #endif

  // Delay
  delay(5000);
}

// Alarm function - LED and active buzzer
void alarm(bool state){
  // Alarm ON
  if (state){
    #ifdef HAS_LED
      digitalWrite(PIN_LED_GREEN, LOW);
      digitalWrite(PIN_LED_RED, HIGH);
    #endif
    for (int i=1; i <= 20; i++){
      #ifdef HAS_LED
        digitalWrite(PIN_LED_RED, i%2==0?LOW:HIGH);
      #endif
      #ifdef HAS_BUZZ
        digitalWrite(PIN_BUZZ, i%2==0?LOW:HIGH);
      #endif
      delay(200);
    }
    return;
  }
  // Alarm OFF
  #ifdef HAS_LED
    digitalWrite(PIN_LED_GREEN, HIGH);
    digitalWrite(PIN_LED_RED, LOW);
  #endif
  #ifdef HAS_BUZZ
    digitalWrite(PIN_BUZZ, LOW);
  #endif
}

#ifdef HAS_ETHERNET
  // Function outputs read request status
  void printResult(int result){
    if (result == 0) {
      #ifdef HAS_SERIAL
        Serial.println(modbusTCPClient.lastError());
      #endif
      return;
    }
    #ifdef HAS_SERIAL
      Serial.print("Request success. Available to read/write ");
      Serial.print(result);
      Serial.println(" values.");
    #endif
  }

  // Function waits for Ethernet cable to get connected
  void checkConnectEthernet(){
    if (Ethernet.linkStatus() == LinkOFF){
      #ifdef HAS_SERIAL
        Serial.print("Ethernet cable not connected. Connect now.");
      #endif
      #ifdef HAS_LCD
        printLCD("Ethernet offline", "Connect cable");
      #endif
      #ifdef HAS_LED
        digitalWrite(PIN_LED_YELLOW, HIGH);
      #endif      
    }
    while (Ethernet.linkStatus() == LinkOFF) {
      #ifdef HAS_SERIAL
        Serial.print(".");
      #endif
      delay(1000);
    }
    #ifdef HAS_LED
      digitalWrite(PIN_LED_YELLOW, LOW);
    #endif
    #ifdef HAS_SERIAL
      Serial.println("Ethernet cable is connected.");
    #endif
  }

  // Function checks id MODBUS is connected. Connects if necessary.
  void checkConnectModbus(){
    // Check if Modbus TCP client is connected
    while(!modbusTCPClient.connected()){
      #ifdef HAS_SERIAL
        Serial.print("Modbus TCP client is not conected. Connecting.");  
      #endif
      #ifdef HAS_LCD
        printLCD("MODBUS offline", "Reconnecting...");
      #endif
      modbusTCPClient.stop();
      modbusTCPClient.begin(ipApiste, 502);
      // Delay 10 seconds
      for (int i=0; i<20; i++){
        #ifdef HAS_LED
          digitalWrite(PIN_LED_YELLOW, i%2 == 0 ? HIGH : LOW);
        #endif
        #ifdef HAS_SERIAL
          Serial.print(".");
        #endif
        delay(500);
      }
    }
    #ifdef HAS_SERIAL
      Serial.println("\nModbus TCP client is connected.");
    #endif
    #ifdef HAS_LCD
      printLCD("MODBUS online", "Connection OK");
    #endif

  }

  double getSetTemperature(){
    // Read Set Temperature (Read Holding Register)
    #ifdef HAS_SERIAL
      Serial.println("\nReading Setting Temperature");
    #endif
    int result = modbusTCPClient.requestFrom(0x01, HOLDING_REGISTERS, 0x0002, 2); // id, type, address, nb - number of values
    printResult(result);
    for (int i=0; i < result-1; i++){
      #ifdef HAS_SERIAL
        Serial.print((double)modbusTCPClient.read()/100, 2);
        Serial.println(" C");
      #endif
    }
  }

  void setSetTemperature(double t){
    // Write Set Temperature
    #ifdef HAS_SERIAL
      Serial.print("\nWriting Set Temperature ");
      Serial.print(t);
      Serial.println("*C");
    #endif
    int result = modbusTCPClient.beginTransmission(0x01, HOLDING_REGISTERS,  0x0002, 2); // int id, int type, int address, int nb
    printResult(result);
    modbusTCPClient.write((unsigned int)(t*100));
    modbusTCPClient.endTransmission();
  }

  int getOnOff(){
    // Read ON/OFF Status
    #ifdef HAS_SERIAL
      Serial.println("\nReading ON/OFF Status");
    #endif
    int result = modbusTCPClient.requestFrom(0x01, HOLDING_REGISTERS, 0x2F00, 2); // id, type, address, nb - number of values
    printResult(result);
    int status = 0;
    for (int i=0; i < result-1; i++){
      status = modbusTCPClient.read();
      #ifdef HAS_SERIAL
        Serial.println(status==1 ? "ON" : "OFF");
      #endif
    }
    return status;
  }

  void setOnOff(int onOff){
    // Turn ON Test
    #ifdef HAS_SERIAL
      Serial.print("\nWriting ");
      Serial.println(onOff == 0 ? "OFF Status" : "ON Status");
    #endif
    int result = modbusTCPClient.beginTransmission(0x01, HOLDING_REGISTERS, 0x2F00, 2); // int id, int type, int address, int nb
    printResult(result);
    modbusTCPClient.write(onOff);
    modbusTCPClient.endTransmission();
  }
#endif