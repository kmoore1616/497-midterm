
#define MAIN_PROGRAM

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
// #include "ICM_20948.h" // Disabled while running without the accelerometer
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <LiquidCrystal.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#define SERIAL_PORT Serial

#define ESP32_OTA_HOSTNAME "esp32-smart-watch"

// BLE custom service/characteristic UUIDs
static const char *ACTIVITY_SERVICE_UUID = "6cfb5360-8c88-4f50-9f24-6ed6bd8d3f8f";
static const char *STEP_COUNT_CHAR_UUID = "0b8dd7d2-e8ad-4a32-8f56-f191d0fc3c42";
static const char *TEMP_CHAR_UUID = "0b9ee8e3-f9be-5b43-9067-02a2e10d4d53";

const char* ssid = "Pixel#";
const char* password = "crazy1234";

//const char* ssid = "TP-Link_BDF3";
//const char* password = "57394206";

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
#define BATTERY_ADC_PIN 35

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

float readBattery(void) {
  const float r1 = 10000.0f;
  const float r2 = 5100.0f;
  const float batteryMinVoltage = 6.0f;
  const float batteryMaxVoltage = 8.4f;
  const float adcReferenceVoltage = 3.3f;
  const int adcMaxReading = 4095;

  int raw = analogRead(BATTERY_ADC_PIN);
  float dividerVoltage = ((float)raw / adcMaxReading) * adcReferenceVoltage;
  float batteryVoltage = dividerVoltage * ((r1 + r2) / r2);
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

void loopOTA(void *pvParameters) {
  while (1) {
    ArduinoOTA.handle();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ============================================ IMU ===========================

#if 0
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
      vTaskDelay((500 / portTICK_PERIOD_MS);
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
    vTaskDelay((250 / portTICK_PERIOD_MS);
  
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
  
  char buffer[100];

  while (1) {
    if (myICM.dataReady())
    {
      char* empty = "";
      strcpy(buffer, empty);
      myICM.getAGMT();

      float accX = myICM.accX();
      float accY = myICM.accY();
      float accZ = myICM.accZ();
      // SERIAL_PORT.print(accX); SERIAL_PORT.print(", ");
      // SERIAL_PORT.print(accY); SERIAL_PORT.print(", ");
      // SERIAL_PORT.print(accZ);
      float acc_sum = pythagorean(accX,accY,accZ);
      doStepCounting(acc_sum, buffer);
      appendInBuffer(buffer, "\n");
      vTaskDelay((8 / portTICK_PERIOD_MS);
    }
    else
    {
      SERIAL_PORT.println("Waiting for data");
      vTaskDelay((500 / portTICK_PERIOD_MS);
    }

    float temp = myICM.temp();
  }
}

float pythagorean(float x, float y, float z) {
  return sqrt(x * x + y * y + z * z);
}

void appendInBuffer(char buffer[], char* addition) {
  int start = sizeof(buffer)/sizeof(char);
  int additionLength = sizeof(addition)/sizeof(char);;
  for (int i = 0; i < additionLength; i++) {
    buffer[start + i] = addition[i];
  }
  buffer[start + additionLength] = '\0';
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
  char* thing = "";
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
#endif

const unsigned long simulatedStepIntervalMs = 1000;

void setupICM20948() {
  SERIAL_PORT.println("ICM-20948 disabled; using simulated step counter");
}

void loopICM20948(void *pvParameters) {
  while (1) {
    float batteryFraction = readBattery();

    steps_taken++;
    temperature = 72 + (steps_taken % 5);

    digitalWrite(LED_pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(100));
    digitalWrite(LED_pin, LOW);

    SERIAL_PORT.print("Simulated steps: ");
    SERIAL_PORT.print(steps_taken);
    SERIAL_PORT.print(" | battery: ");
    SERIAL_PORT.println(batteryFraction, 3);

    vTaskDelay(pdMS_TO_TICKS(simulatedStepIntervalMs));
  }
}


// ============================================ LCD ===========================

// SPLC780D character LCDs are command-compatible with HD44780 displays,
// so the standard LiquidCrystal 4-bit interface works with the same pinout.
const int rs = 19, en = 18, d4 = 27, d5 = 26, d6 = 25, d7 = 33;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

unsigned long last_lcd_print = 0;
void setupLCD() {
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

char buff[17];

#define millis_per_print 500
void loopLCD(void *pvParameters) {
  while (1) {
    if (millis() > last_lcd_print + millis_per_print) {
      last_lcd_print = millis();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Steps:");
      lcd.setCursor(0, 1);
      snprintf(buff, sizeof(buff), "%d", steps_taken);
      lcd.print(buff);
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================ Printer ===========================

QueueHandle_t msgQueue;

struct Message {
  char buffer[100];
};


// uses print, not println
void printFromQueue(void *pvParameters) {

  while(1) {
    struct Message myMessage;
    if (xQueueReceive(msgQueue, &myMessage, portMAX_DELAY) == pdPASS) {
      Serial.print(myMessage.buffer);
    }
    vTaskDelay(100);
  }
}

void setupMsgQueue() {
  msgQueue = xQueueCreate(10, sizeof(struct Message));
}

// ============================================ General ===========================


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
  setupMsgQueue();

  xTaskCreate(
  loopBLE
  ,  "loopBLE"  // A name just for humans
  ,  4096  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
  xTaskCreate(
  loopICM20948
  ,  "loopICM20948"  // A name just for humans
  ,  4096  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
  xTaskCreate(
  loopLCD
  ,  "loopLCD"  // A name just for humans
  ,  4096  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
  xTaskCreate(
  printFromQueue
  ,  "printFromQueue"  // A name just for humans
  ,  4096  // stack size
  ,  NULL
  ,  3  // Priority
  ,  NULL ); 
  xTaskCreate(
  loopOTA
  ,  "loopOTA"  // A name just for humans
  ,  4096  // stack size
  ,  NULL
  ,  2  // Priority
  ,  NULL ); 
  

}

void loop() {

  
  // long after = micros();
}
