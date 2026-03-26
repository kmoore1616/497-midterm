#define MAIN_PROGRAM

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

const char* ssid = "Pixel#";
const char* password = "crazy1234";

// const char* ssid = "TP-Link_BDF3";
// const char* password = "57394206";

BLECharacteristic *stepCharacteristic;
BLECharacteristic *tempCharacteristic;
BLECharacteristic *heartBPMCharacteristic;
BLECharacteristic *batteryCharacteristic;
bool bleClientConnected = false;


int lastNotifiedSteps = -1; 

unsigned long lastCounterUpdateMs = 0;
unsigned long lastBlePublishMs = 0;

const unsigned long counterIntervalMs = 1000;   // increment once per second
const unsigned long blePublishIntervalMs = 200; // update BLE value 5 Hz

// Step counting vars

#define LED_pin 23
#define BATTERY_ADC_PIN 35
#define LCD_PIN 15

#define alpha 0.1f
// #define long_alpha 0.001f

#define step_threshold 300

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

/*
0 = step
1 = heart
2 = battery
*/
int maxDisplayMode = 2;
int displayMode = 0;
bool flagSleep = false;

float batteryFraction = 0;
int batteryPercentage = 0;

int beatAvg;

//#define awakeMillis 30000
#define awakeMillis 9000000

int awakeTimer;

// ================================= BATTERY ==================================================

const unsigned long simulatedStepIntervalMs = 1000;

void loopBattery(void *pvParameters) {
  while (1) {
    struct Message msg;
    batteryFraction = readBattery();
    batteryPercentage = (int)(batteryFraction * 100);
    snprintf(msg.buffer, sizeof(msg.buffer), "Battery: %d%%", batteryPercentage);
    xQueueSend(msgQueue, &msg, portMAX_DELAY);
    
//    digitalWrite(LED_pin, HIGH);
//    vTaskDelay(pdMS_TO_TICKS(100));
//    digitalWrite(LED_pin, LOW);

    vTaskDelay(pdMS_TO_TICKS(10000));
    
  }
}

float readBattery(void) {
  const float batteryMinVoltage = 3.0f;
  const float batteryMaxVoltage = 4.2f;
  int raw = analogRead(BATTERY_ADC_PIN);

  float batteryVoltage = 3.0f + (raw - 1100) * (1.2f / 500.0f);

  float fraction = (batteryVoltage - batteryMinVoltage) / (batteryMaxVoltage - batteryMinVoltage);

  if (fraction < 0.0f) {
    return 0.0f;
  }

  if (fraction > 1.0f) {
    return 1.0f;
  }

  return fraction;
}

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

  //create characteristics
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

  heartBPMCharacteristic = activityService->createCharacteristic(
    TEMP_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  heartBPMCharacteristic->addDescriptor(new BLE2902());

  batteryCharacteristic = activityService->createCharacteristic(
    TEMP_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  batteryCharacteristic->addDescriptor(new BLE2902());

  //attach values
  stepCharacteristic->setValue((uint8_t *)&steps_taken, sizeof(steps_taken));
  tempCharacteristic->setValue((uint8_t *)&temperature, sizeof(temperature));
  heartBPMCharacteristic->setValue((uint8_t *)&beatAvg, sizeof(beatAvg));
  batteryCharacteristic->setValue((uint8_t *)&batteryPercentage, sizeof(batteryPercentage));



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
    heartBPMCharacteristic->setValue((uint8_t *)&beatAvg, sizeof(beatAvg));
    batteryCharacteristic->setValue((uint8_t *)&batteryPercentage, sizeof(batteryPercentage));

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
void loopOTA(void *pvParameters) {
  while (1) {
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(500));
  }
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

  //https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/blob/master/src/MAX30105.cpp, 
  // particleSensor.setSampleRate((uint8_t) 0x08); //search for MAX30105_SAMPLERATE etc
  // particleSensor.setPulseWidth((uint8_t) 0x03); //0x00 to 0x03
  // particleSensor.setPulseAmplitudeRed((uint8_t) 0x7F);
  //this one makes it basically non-responsive. Keep at max
  // particleSensor.setPulseAmplitudeIR((uint8_t) 0xFF); //0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
  // particleSensor.setPulseAmplitudeGreen((uint8_t)0x00);
  
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
    else 
      awakeTimer = millis() + awakeMillis;

    xQueueSend(msgQueue, &msg, portMAX_DELAY);
    // Serial.println("heart rate msg sent");
    
    //400 samples / 60 sec
    vTaskDelay(15 / portTICK_PERIOD_MS);
  }
}



// ============================================ IMU ===========================

#if 1
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
      if (displayMode == 0)
        //xQueueSend(msgQueue, &msg, portMAX_DELAY);
      // Serial.println("ICM message sent");

      vTaskDelay(8 / portTICK_PERIOD_MS);
    }
    else
    {
      SERIAL_PORT.println("Waiting for data");
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }

    temperature = myICM.temp();
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
  char thing[100] = "";
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
  awakeTimer = millis() + awakeMillis;
}

void end_step() {
  digitalWrite(LED_pin, LOW);
  going_up = true;
  awakeTimer = millis() + awakeMillis;
}


#else

//moved to Battery section

// const unsigned long simulatedStepIntervalMs = 1000;

// void setupICM20948() {
//   SERIAL_PORT.println("ICM-20948 disabled; using simulated step counter");
// }

// void loopICM20948(void *pvParameters) {
//   while (1) {
//     float batteryFraction = readBattery();

//     steps_taken++;
//     temperature = 72 + (steps_taken % 5);

//     digitalWrite(LED_pin, HIGH);
//     vTaskDelay(pdMS_TO_TICKS(100));
//     digitalWrite(LED_pin, LOW);

//     SERIAL_PORT.print("Simulated steps: ");
//     SERIAL_PORT.print(steps_taken);
//     SERIAL_PORT.print(" | battery: ");
//     SERIAL_PORT.println(batteryFraction, 3);

//     vTaskDelay(pdMS_TO_TICKS(simulatedStepIntervalMs));
//   }
// }

#endif


// ============================================ LCD ===========================

// SPLC780D character LCDs are command-compatible with HD44780 displays,
// so the standard LiquidCrystal 4-bit interface works with the same pinout.
const int rs = 19, en = 18, d4 = 27, d5 = 26, d6 = 25, d7 = 33;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

char lcdBuff[17];

unsigned long last_lcd_print = 0;
void setupLCD() {
  pinMode(LCD_PIN, OUTPUT);
  digitalWrite(LCD_PIN, HIGH);
  delay(50);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SPLC780D LCD");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  delay(1000);
  lcd.clear();
  last_lcd_print = millis();
}

#define millis_per_print 500
void loopLCD(void *pvParameters) {
  while (1) {
    if (flagSleep) {
      lcdSleep();
      vTaskDelay(1000); //will be asleep by then
    }

    if (millis() > last_lcd_print + millis_per_print) {
      last_lcd_print = millis();
      switch (displayMode) {
        default: //0 or other
          lcdSteps();
          break;
        case 1: 
          lcdHeart();
          break;
        case 2:
          lcdBattery();
          break;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void lcdSteps() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Steps:");
  lcd.setCursor(0, 1);
  snprintf(lcdBuff, sizeof(lcdBuff), "%d", steps_taken);
  lcd.print(lcdBuff);
}
void lcdHeart() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("BPM:");
  lcd.setCursor(0, 1);
  snprintf(lcdBuff, sizeof(lcdBuff), "%d", beatAvg);
  lcd.print(lcdBuff);
}
void lcdBattery() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Battery:");
  lcd.setCursor(0, 1);
  snprintf(lcdBuff, sizeof(lcdBuff), "%.0f", batteryFraction * 100);
  lcd.print(lcdBuff);
}
void lcdSleep() {
  digitalWrite(rs, LOW);
  digitalWrite(en, LOW);
  digitalWrite(d4, LOW);
  digitalWrite(d5, LOW);
  digitalWrite(d6, LOW);
  digitalWrite(d7, LOW);
//  pinMode(rs, INPUT);
//  pinMode(en, INPUT);
//  pinMode(d4, INPUT);
//  pinMode(d5, INPUT);
//  pinMode(d6, INPUT);
//  pinMode(d7, INPUT);x  `
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.noDisplay();
  digitalWrite(LCD_PIN, LOW);


  // lcd.print("zzzzz");
}

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
      if (flagSleep) {
        break;
      }
      Serial.println(myMessage.buffer);
    }

    vTaskDelay(100);
  }
}

void setupMsgQueue() {
  msgQueue = xQueueCreate(10, sizeof(struct Message));
}

// ============================================ Control (Button Interrupt) ===========================

//


#define BUTTON_SLEEP_PIN 32
#define BUTTON_DISPLAY_MODE_PIN 14

void setupControl() {
  pinMode(BUTTON_SLEEP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_SLEEP_PIN), sleepInterrupt, FALLING);
  pinMode(BUTTON_DISPLAY_MODE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_DISPLAY_MODE_PIN), displayInterrupt, FALLING);
  
  ++bootCount;
  Serial.println("Boot count: " + String(bootCount));
  digitalWrite(BPM_LED_Pin, HIGH);
  delay(500);
  digitalWrite(BPM_LED_Pin, LOW);
  delay(100);
  // testButton();

  awakeTimer = millis() + awakeMillis + 5000;
}

// void testButton() {
//   while (1) {
//     Serial.println(digitalRead(BUTTON_INTERRUPT_PIN));
//     delay(100);
//   }
// }

bool flagResetTimer = false;
void sleepInterrupt() {
  flagSleep = true;
  // flagHighPrecision = !flagHighPrecision;
  // interruptCounter++;
}

int cooldown = 0;
void displayInterrupt() {
  // if (cooldown > 0) return; //basic debouncing
  displayMode = (displayMode + 1) % (maxDisplayMode + 1);
  cooldown = 400;
  flagResetTimer = true;
}

void loopControl(void *pvParameters) {
  // while(1) {
  //   digitalWrite(BPM_LED_Pin, flagHighPrecision);
  //   // Serial.printf("interrupt count: %d, flagHighPrecision:%s\n", interruptCounter, flagHighPrecision ? "true" : "false");
  // }

  float delayAmount = 100;

  while (1) {
    if (flagSleep) {
      goToSleep();
    }
    if (cooldown > 0) {
      cooldown -= delayAmount;
    }
    if (millis() > awakeTimer) {
      flagSleep = true;
    } 
    if (flagResetTimer) {
      awakeTimer = millis() + awakeMillis;
      flagResetTimer = false;
    }

    vTaskDelay(delayAmount);
  }
}

void goToSleep() {
  esp_sleep_enable_ext0_wakeup(WAKEUP_GPIO, LOW); //same as BUTTON_SLEEP_PIN
  pinMode(WAKEUP_GPIO, PULLUP);
  Serial.println("sleeping now...");
  digitalWrite(BPM_LED_Pin, HIGH);
  delay(500);
  digitalWrite(BPM_LED_Pin, LOW);
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

  setupWiFi();
  delay(50);

  setupOTA();
  delay(50);

  setupBLE();
  delay(50);

  setupMsgQueue();
  delay(50);

  setupICM20948();
  delay(50);

  setupHeartRate();
  delay(50);

  setupLCD();
  delay(50);

  setupControl();
  delay(50);


  xTaskCreate(
  printFromQueue
  ,  "printFromQueue"  // A name just for humans
  ,  1024  // stack size
  ,  NULL
  ,  3  // Priority
  ,  NULL ); 
  Serial.println("f");
  delay(200);

  xTaskCreate(
  loopBLE
  ,  "loopBLE"  // A name just for humans
  ,  1024  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
  Serial.println("g");
//  delay(200);

  
  xTaskCreate(
  loopLCD
  ,  "loopLCD"  // A name just for humans
  ,  4096  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
  Serial.println("h");
//  delay(200);

  xTaskCreate(
  loopOTA
  ,  "loopOTA"  // A name just for humans
  ,  15 24  // stack size
  ,  NULL
  ,  1  // Priority
  ,  NULL ); 
  Serial.println("i");
//  delay(200);

  xTaskCreate(
  loopControl
  ,  "loopControl"  // A name just for humans
  ,  2048  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
    Serial.println("j");

  
  xTaskCreate(
  loopICM20948
  ,  "loopICM20948"  // A name just for humans
  ,  3048  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
  Serial.println("k");
//  delay(200);

  xTaskCreate(
  loopHeartRate
  ,  "loopHeartRate"  // A name just for humans
  ,  2548  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
  Serial.println("l");
//  delay(200);

  BaseType_t ok = xTaskCreate(
  loopBattery
  ,  "loopBattery"  // A name just for humans
  ,  2548  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 

  Serial.println(ok == pdPASS ? "loopBattery created" : "loopBattery FAILED");

  Serial.println("setup complete");
//  delay(200);

//  digitalWrite(BPM_LED_Pin, HIGH);
//  delay(200);
//  digitalWrite(BPM_LED_Pin, LOW);
//  delay(200);
//  digitalWrite(BPM_LED_Pin, HIGH);
//  delay(200);
//  digitalWrite(BPM_LED_Pin, LOW);
}

void loop() {

  
  // long after = micros();
}
