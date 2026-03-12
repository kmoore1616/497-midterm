#define MAIN_PROGRAM
#ifdef MAIN_PROGRAM

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <LiquidCrystal.h>

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
#define SERIAL_PORT Serial

#define AD0_VAL 1
#define ESP32_OTA_HOSTNAME "esp32-smart-watch"

ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object

// BLE custom service/characteristic UUIDs
static const char *ACTIVITY_SERVICE_UUID = "6cfb5360-8c88-4f50-9f24-6ed6bd8d3f8f";
static const char *STEP_COUNT_CHAR_UUID = "0b8dd7d2-e8ad-4a32-8f56-f191d0fc3c42";
static const char *TEMP_CHAR_UUID = "0b9ee8e3-f9be-5b43-9067-02a2e10d4d53";

//const char* ssid = "Pixel#";
//const char* password = "crazy1234";

const char* ssid = "TP-Link_BDF3";
const char* password = "57394206";

BLECharacteristic *stepCharacteristic;
BLECharacteristic *tempCharacteristic;
bool bleClientConnected = false;


int lastNotifiedSteps = -1; 

unsigned long lastCounterUpdateMs = 0;
unsigned long lastBlePublishMs = 0;

const unsigned long counterIntervalMs = 1000;   // increment once per second
const unsigned long blePublishIntervalMs = 200; // update BLE value 5 Hz

// Step counting vars

#define LED_pin 23

#define alpha 0.1f
// #define long_alpha 0.001f

#define step_threshold 500

bool counter_initializing = true;
bool going_up = false;
int steps_taken = 0;

int temperature = 0;

float ema = 0;
// float long_ema = 0;

float step_min = 0;
float step_max = 0;

int iteration = 0;


// ================================= BLUETOOTH ==================================================

class ActivityServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) override {
    bleClientConnected = true;
  }

  void onDisconnect(BLEServer *pServer) override {
    bleClientConnected = false;
    pServer->startAdvertising();
  }
};

void setupBLE() {
  BLEDevice::init("ESP32-StepCounter");

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ActivityServerCallbacks());

  BLEService *activityService = server->createService(ACTIVITY_SERVICE_UUID);
  stepCharacteristic = activityService->createCharacteristic(
    STEP_COUNT_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  stepCharacteristic->addDescriptor(new BLE2902());

   tempCharacteristic = activityService->createCharacteristic(
    TEMP_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  tempCharacteristic->addDescriptor(new BLE2902());

  
  tempCharacteristic->setValue((uint8_t *)&temperature, sizeof(temperature));
  stepCharacteristic->setValue((uint8_t *)&steps_taken, sizeof(steps_taken));
  
 
  
  activityService->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(ACTIVITY_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  SERIAL_PORT.println("BLE advertising started: ESP32-StepCounter");
}

// TESTING COUNTER THAT EMULATES STEPS
//void loopCounter() {
//  if (millis() - lastCounterUpdateMs >= counterIntervalMs) {
//    lastCounterUpdateMs = millis();
//    steps_taken = steps_taken + 1;
//
//    SERIAL_PORT.print("Simulated steps: ");
//    SERIAL_PORT.println(steps_taken);
//  }
//}


void loopBLE() {
  if (!stepCharacteristic) {
    return; // Ar
  }

  if (millis() - lastBlePublishMs < blePublishIntervalMs) {
    return;
  }

  
  lastBlePublishMs = millis();

  stepCharacteristic->setValue((uint8_t *)&steps_taken, sizeof(steps_taken));
  tempCharacteristic->setValue((uint8_t *)&temperature, sizeof(temperature));


  if (bleClientConnected && steps_taken != lastNotifiedSteps) {
    stepCharacteristic->notify();
    lastNotifiedSteps = steps_taken;
  }
}

// ============================================ OTA =================================================
void setupOTA(){
  // OTA Setup
  ArduinoOTA.setHostname(ESP32_OTA_HOSTNAME);

  ArduinoOTA
    .onStart([]() {
      Serial.println("Start updating...");
    })
    .onEnd([]() {
      Serial.println("\nUpdate Complete");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress * 100) / total);
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]\n", error);
    });

  ArduinoOTA.begin();

  Serial.println("Ready for OTA");
}

// ============================================ IMU ===========================

void setupICM20948() {
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
  ICM_20948_fss_t fss = {};

  bool initialized = false;
  while (!initialized)
  {


    myICM.begin(WIRE_PORT, AD0_VAL);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
  
    myICM.swReset();
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.print(F("Software Reset returned: "));
      SERIAL_PORT.println(myICM.statusString());
    }
    delay(250);
  
    // Now wake the sensor up
    myICM.sleep(false);
    myICM.lowPower(false);

    myICM.setSampleMode(ICM_20948_Internal_Acc, ICM_20948_Sample_Mode_Continuous);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.print(F("setSampleMode returned: "));
      SERIAL_PORT.println(myICM.statusString());
    }

     ICM_20948_fss_t myFSS;
     myFSS.a = gpm8;
     myICM.setFullScale(ICM_20948_Internal_Acc, myFSS);
     if (myICM.status != ICM_20948_Stat_Ok)
     {
       SERIAL_PORT.print(F("setFullScale returned: "));
       SERIAL_PORT.println(myICM.statusString());
     }

      ICM_20948_dlpcfg_t myDLPcfg;
     myDLPcfg.a = acc_d246bw_n265bw; // Digital low pass of 246 hz
     myICM.setDLPFcfg(ICM_20948_Internal_Acc, myDLPcfg);
      if (myICM.status != ICM_20948_Stat_Ok)
      {
        SERIAL_PORT.print(F("setDLPcfg returned: "));
        SERIAL_PORT.println(myICM.statusString());
      }

      ICM_20948_smplrt_t sample;
      sample.a = 100;
      myICM.setSampleRate(ICM_20948_Internal_Acc, sample);


  }
}

void loopICM20948() {
  if (myICM.dataReady())
  {
    myICM.getAGMT();

    float accX = myICM.accX();
    float accY = myICM.accY();
    float accZ = myICM.accZ();
    temperature = myICM.temp();

    // SERIAL_PORT.print(accX); SERIAL_PORT.print(", ");
    // SERIAL_PORT.print(accY); SERIAL_PORT.print(", ");
    // SERIAL_PORT.print(accZ);
    float acc_sum = pythagorean(accX,accY,accZ);
    doStepCounting(acc_sum);
    Serial.println();
    delay(8);
   
  }
  else
  {
    SERIAL_PORT.println("Waiting for data");
    delay(500);
  }

  float temp = myICM.temp();
  // Serial.print("temp = ");
  // Serial.println(temp);
}

////////////////////////////////////////////////////////////////////////
// Step Counting
////////////////////////////////////////////////////////////////////////


float pythagorean(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

void doStepCounting(float acc) {
  // Serial.print(acc);
  // Serial.print(", ");

  iteration++;
  if (iteration == 1) {
    step_min = acc;
    step_max = acc;
    ema = acc;
    // long_ema = acc;
    return;
  }
  float before = ema;
  applyToEMA(acc);
  Serial.print(ema);
  Serial.print(", ");

  // digitalWrite(LED_pin, ema > before ? HIGH : LOW);

  float x = ema;
  if (counter_initializing) {
    //EMAs may be off
    init_step(x);
    return;
  }
  //in the swing of things
  checkStep(x);
}

void applyToEMA(float acc) {
  ema = next_ema(ema, alpha, acc);
  // long_ema = next_ema(long_ema, long_alpha, acc);
}

void init_step(float x) {
  if (x > step_max) {
    step_max = x;
    if (step_max - step_min > step_threshold) {
      counter_initializing = false;
      begin_step();
    }
  }
  if (x < step_min) {
    step_min = x;
    if (step_max - step_min > step_threshold) {
      counter_initializing = false;
      end_step();
    }
  }
}


//
// if the ema has moved by a certain amount away from its peak, then same for the negative direction, that's counted as a step
//
void checkStep(float x) {
  Serial.print(step_min);
  Serial.print(", ");
  Serial.print(step_max);
  Serial.print(", ");
  if (x > step_max) {
    step_max = x;
    step_min = step_max - step_threshold;
    if (going_up) begin_step();
  }
  if (x < step_min) {
    step_min = x;
    step_max = step_min + step_threshold;
    if (!going_up) end_step();
  }

}

float next_ema(float ema, float a, float new_value) {
  return new_value * a + ema * (1 - a);
}

void begin_step() {
  digitalWrite(LED_pin, HIGH);
  going_up = false;
  steps_taken++;
}

void end_step() {
  digitalWrite(LED_pin, LOW);
  going_up = true;
}


////////////////////////////////////////////////////////////////////////
// LCD
////////////////////////////////////////////////////////////////////////

/*
  LiquidCrystal Library - Hello World

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.

 This sketch prints "Hello World!" to the LCD
 and shows the time.

  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe
 modified 7 Nov 2016
 by Arturo Guadalupi

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystalHelloWorld

*/

// include the library code:


// #define LED2_pin 12
// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 19, en = 18, d4 = 27, d5 = 26, d6 = 25, d7 = 33;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

unsigned long last_lcd_print = 0;
void setupLCD() {
  // set up the LCD's number of columns and rows:
  // pinMode(LED2_pin, OUTPUT);
  // digitalWrite(LED2_pin, LOW);
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("hello, world!");
  delay(1000);
  lcd.clear();
  delay(1000);
  last_lcd_print = millis();
}

char buff[16];

#define millis_per_print 500
void loopLCD() {
  if (millis() > last_lcd_print + millis_per_print) {
    // Serial.println("aaaaa");
    last_lcd_print = millis();
    // set the cursor to column 0, line 1
    // (note: line 1 is the second row, since counting begins with 0):
    lcd.clear();
    // delay(500);
    lcd.setCursor(0, 1);
    // print the number of seconds since reset:
    sprintf(buff, "%d steps", steps_taken);
    lcd.print(buff);
    // lcd.print("testtesttest");
  }
}




void setup() {
  SERIAL_PORT.begin(115200);
  while(!SERIAL_PORT);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  setupOTA();
  setupBLE();
  setupICM20948();
  setupLCD();
}

void loop() {
//  loopCounter(); // Steps emulation
  ArduinoOTA.handle();
  loopBLE();
  loopICM20948();
  
  loopLCD();

  
  long after = micros();
}

#endif
