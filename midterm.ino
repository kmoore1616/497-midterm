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

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

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

QueueHandle_t msgQueue;

struct Message {
  char buffer[100];
};

int interruptCounter = 0;

#include "driver/rtc_io.h"

#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
#define USE_EXT0_WAKEUP          1               // 1 = EXT0 wakeup, 0 = EXT1 wakeup
#define WAKEUP_GPIO              GPIO_NUM_32     // Only RTC IO are allowed - ESP32 Pin example
RTC_DATA_ATTR int bootCount = 0;

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


void loopBLE(void *pvParameters) {

  Serial.println("loopBLE begun");
  // char buffer[100];

  while(1) {
    if (!stepCharacteristic) {
      return; // Ar
    }

    //wait
    vTaskDelay(blePublishIntervalMs);
    // if (millis() - lastBlePublishMs < blePublishIntervalMs) {
    //   return;
    // }

    
    lastBlePublishMs = millis();

    stepCharacteristic->setValue((uint8_t *)&steps_taken, sizeof(steps_taken));
    tempCharacteristic->setValue((uint8_t *)&temperature, sizeof(temperature));

    if (bleClientConnected && steps_taken != lastNotifiedSteps) {
      stepCharacteristic->notify();
      lastNotifiedSteps = steps_taken;
    }
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

// ============================================ Heart Rate ===========================

//Example5_HeartRate.ino from SparkFun Max3010X


#include <Wire.h>
#include "MAX30105.h"

#include "heartRate.h"

#define BPM_LED_Pin 16

MAX30105 particleSensor;

const byte RATE_SIZE = 20; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
bool fullLoop = false;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

bool flagHighPrecision = false;

void setupHeartRate()
{
  Serial.begin(115200);
  Serial.println("Initializing...");

  pinMode(BPM_LED_Pin, OUTPUT);
  digitalWrite(BPM_LED_Pin, LOW);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loopHeartRate(void *pvParameters) {

  Serial.println("loopHeartRate begun");
  
  struct Message msg;

  while(1) {

    long irValue = particleSensor.getIR();
    bool beatDetected = checkForBeat(irValue);

    if (beatDetected)
    {
      //We sensed a beat!
      long delta = millis() - lastBeat;
      lastBeat = millis();

      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20)
      {
        rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
        rateSpot %= RATE_SIZE; //Wrap variable
        if (!fullLoop && rateSpot == 0) {
          fullLoop = true;
        }
        //Take average of readings
        beatAvg = 0;
        byte max = fullLoop ? RATE_SIZE : rateSpot;
        for (byte x = 0 ; x < max; x++)
          beatAvg += rates[x];
        beatAvg /= max;
      }
      else {

      }
    }
    if (!beatDetected) {
      if (!flagHighPrecision) {
        vTaskDelay(15 / portTICK_PERIOD_MS);
      }
      else {
        //1200 samples instead of 400
        vTaskDelay(5 / portTICK_PERIOD_MS);
      }
      continue;
    }
    
    char* empty = "";
    strcpy(msg.buffer, empty);

    char thing[100] = "";
    sprintf(thing, "IR=%ld, BPM=%f, Avg BPM = %d", irValue, beatsPerMinute, beatAvg);
    appendInBuffer(msg.buffer, thing);
    if (!fullLoop)
      appendInBuffer(msg.buffer, " (not yet max samples)");
    if (flagHighPrecision) {
      appendInBuffer(msg.buffer, " (precision)");
    }


    // Serial.print("IR=");
    // Serial.print(irValue);
    // Serial.print(", BPM=");
    // Serial.print(beatsPerMinute);
    // Serial.print(", Avg BPM=");
    // Serial.print(beatAvg);

    if (irValue < 50000)
      appendInBuffer(msg.buffer, " No finger?");
    
    xQueueSend(msgQueue, &msg, portMAX_DELAY);
    // Serial.println("heart rate msg sent");
    
    //400 samples / 60 sec
    vTaskDelay(15 / portTICK_PERIOD_MS);
  }
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
      vTaskDelay(500 / portTICK_PERIOD_MS);
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
    vTaskDelay(250 / portTICK_PERIOD_MS);
  
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

void loopICM20948(void *pvParameters) {
  
  Serial.println("loopICM20948 begun");
  
  struct Message msg;

  while (1) {
    if (myICM.dataReady())
    {
      char* empty = "";
      strcpy(msg.buffer, empty);
      myICM.getAGMT();

      float accX = myICM.accX();
      float accY = myICM.accY();
      float accZ = myICM.accZ();
      // SERIAL_PORT.print(accX); SERIAL_PORT.print(", ");
      // SERIAL_PORT.print(accY); SERIAL_PORT.print(", ");
      // SERIAL_PORT.print(accZ);
      float acc_sum = pythagorean(accX,accY,accZ);
      doStepCounting(acc_sum, msg.buffer);
      // Serial.print("msg.buffer:");
      // Serial.println(msg.buffer);
      xQueueSend(msgQueue, &msg, portMAX_DELAY);
      // Serial.println("ICM message sent");

      vTaskDelay(8 / portTICK_PERIOD_MS);
    }
    else
    {
      SERIAL_PORT.println("Waiting for data");
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    float temp = myICM.temp();
  }
}

float pythagorean(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

void appendInBuffer(char buffer[], char* addition) {
  // Serial.printf("before: buffer=\"%s\", addition=\"%s\"\n", buffer, addition);
  int start = strlen(buffer);//sizeof(buffer)/sizeof(char);
  int additionLength = strlen(addition);//sizeof(addition)/sizeof(char);
  for (int i = 0; i < additionLength; i++) {
    buffer[start + i] = addition[i];
  }
  buffer[start + additionLength] = '\0';
  // Serial.printf("start=%d, addition=%d, end = %d\n", start, additionLength, start + additionLength);
  // Serial.printf("after: buffer=\"%s\", addition=\"%s\"\n", buffer, addition);
}

// ============================================ Step Counting (extension of IMU task) ===========================


void doStepCounting(float acc, char buffer[]) {
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
  char thing[100] = "";
  sprintf(thing, "%f, ", ema);
  appendInBuffer(buffer, thing);

  // digitalWrite(LED_pin, ema > before ? HIGH : LOW);

  float x = ema;
  if (counter_initializing) {
    //EMAs may be off
    init_step(x);
    return;
  }
  //in the swing of things
  checkStep(x, buffer);
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

// if the ema has moved by a certain amount away from its peak, then same for the negative direction, that's counted as a step
void checkStep(float x, char buffer[]) {
  char* thing = "";
  sprintf(thing, "%d, ", step_min);
  appendInBuffer(buffer, thing);
  sprintf(thing, "%d, ", step_max);
  appendInBuffer(buffer, thing);
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


// ============================================ LCD ===========================

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

*/

// ============================================ Printer ===========================


//Must go at top of file?
// QueueHandle_t msgQueue;

// struct Message {
//   char buffer[100];
// };


// uses print, not println
void printFromQueue(void *pvParameters) {

  Serial.println("printFromQueue begun");

  while(1) {
    struct Message myMessage;
    while (xQueueReceive(msgQueue, &myMessage, portMAX_DELAY) == pdPASS) {
      // Serial.print("msg received! ->");
      Serial.println(myMessage.buffer);
    }

    vTaskDelay(100);
  }
}

void setupMsgQueue() {
  msgQueue = xQueueCreate(10, sizeof(struct Message));
}

// ============================================ Button Interrupt ===========================

//


#define BUTTON_INTERRUPT_PIN 32

void setupButton() {
  pinMode(BUTTON_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_INTERRUPT_PIN), buttonInterrupt, FALLING);
  
  ++bootCount;
  Serial.println("Boot count: " + String(bootCount));
  digitalWrite(BPM_LED_Pin, HIGH);
  delay(100);
  digitalWrite(BPM_LED_Pin, LOW);
  delay(100);
  // testButton();
}

// void testButton() {
//   while (1) {
//     Serial.println(digitalRead(BUTTON_INTERRUPT_PIN));
//     delay(100);
//   }
// }

bool flagSleep = false;
int cooldown = 0;
void buttonInterrupt() {
  flagSleep = true;
  // if (cooldown > 0) return;
  // flagHighPrecision = !flagHighPrecision;
  // interruptCounter++;
  // cooldown = 500;
}

void loopButton(void *pvParameters) {
  // float delayAmount = 100;
  // while(1) {
  //   digitalWrite(BPM_LED_Pin, flagHighPrecision);
  //   // Serial.printf("interrupt count: %d, flagHighPrecision:%s\n", interruptCounter, flagHighPrecision ? "true" : "false");
  //   if (cooldown > 0) {
  //     cooldown -= delayAmount;
  //   }
  //   vTaskDelay(delayAmount);
  // }

  while (1) {
    if (flagSleep) {
      goToSleep();
    }
    vTaskDelay(100);
  }
}

void goToSleep() {
  esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, LOW); //same as BUTTON_INTERRUPT_PIN
  pinMode(WAKEUP_GPIO, PULLUP);
  Serial.println("sleeping now...");
  delay(500);
  esp_deep_sleep_start();
}

// ============================================ General ===========================

void setupWiFi() {
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

}

void setup() {
  SERIAL_PORT.begin(115200);
  while(!SERIAL_PORT);

  // setupWiFi();
  Serial.println("a");
  delay(200);

  // setupOTA();
  Serial.println("b");
  delay(200);

  setupBLE();
  Serial.println("c");
  delay(200);

  setupMsgQueue();
  Serial.println("d");
  delay(200);

  setupICM20948();
  Serial.println("e");
  delay(200);

  setupHeartRate();
  Serial.println("e");
  delay(200);

//  setupLCD();
  Serial.println("f");
  delay(200);

  setupButton();
  Serial.println("a");
  delay(200);


  xTaskCreate(
  printFromQueue
  ,  "printFromQueue"  // A name just for humans
  ,  1028  // stack size
  ,  NULL
  ,  3  // Priority
  ,  NULL ); 
  Serial.println("h");
  delay(200);

  xTaskCreate(
  loopBLE
  ,  "loopBLE"  // A name just for humans
  ,  1028  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
  Serial.println("g");
  delay(200);

  // xTaskCreate(
  // loopICM20948
  // ,  "loopICM20948"  // A name just for humans
  // ,  4096  // stack size
  // ,  NULL
  // ,  2  // Priority
  // ,  NULL ); 
  // Serial.println("h");
  // delay(5000);

  xTaskCreate(
  loopHeartRate
  ,  "loopHeartRate"  // A name just for humans
  ,  4096  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
  Serial.println("h");
  delay(200);

  // xTaskCreate(
  // loopICM20948
  // ,  "loopICM20948"  // A name just for humans
  // ,  1028  // stack size
  // ,  NULL
  // ,  2  // Priority
  // ,  NULL ); 
  // Serial.println("h");
  // delay(200);
  
  // xTaskCreate(
  // loopLCD
  // ,  "loopLCD"  // A name just for humans
  // ,  1028  // stack size
  // ,  NULL
  // ,  2  // Priority
  // ,  NULL ); 
  // Serial.println("i");
  // delay(200);

  xTaskCreate(
  loopButton
  ,  "loopButton"  // A name just for humans
  ,  2048  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
  Serial.println("h");
  delay(200);


  Serial.println("setup complete");
  delay(200);
}

void loop() {

  
  // long after = micros();
}

#endif
