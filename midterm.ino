#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>



#define SERIAL_PORT Serial
#define ESP32_OTA_HOSTNAME "esp32-smart-watch"

// BLE custom service/characteristic UUIDs
static const char *ACTIVITY_SERVICE_UUID = "6cfb5360-8c88-4f50-9f24-6ed6bd8d3f8f";
static const char *STEP_COUNT_CHAR_UUID = "0b8dd7d2-e8ad-4a32-8f56-f191d0fc3c42";


const char* ssid = "TP-Link_BDF3";
const char* password = "57394206";

BLECharacteristic *stepCharacteristic;
bool bleClientConnected = false;

int steps_taken = 0;
int lastNotifiedSteps = -1; 

unsigned long lastCounterUpdateMs = 0;
unsigned long lastBlePublishMs = 0;

const unsigned long counterIntervalMs = 1000;   // increment once per second
const unsigned long blePublishIntervalMs = 200; // update BLE value 5 Hz

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

void loopCounter() {
  if (millis() - lastCounterUpdateMs >= counterIntervalMs) {
    lastCounterUpdateMs = millis();
    steps_taken = steps_taken + 1;

    SERIAL_PORT.print("Simulated steps: ");
    SERIAL_PORT.println(steps_taken);
  }
}

void loopBLE() {
  if (!stepCharacteristic) {
    return; // Ar
  }

  if (millis() - lastBlePublishMs < blePublishIntervalMs) {
    return;
  }

  
  lastBlePublishMs = millis();

  stepCharacteristic->setValue((uint8_t *)&steps_taken, sizeof(steps_taken));

  if (bleClientConnected && steps_taken != lastNotifiedSteps) {
    stepCharacteristic->notify();
    lastNotifiedSteps = steps_taken;
  }
}

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
}

void loop() {
  loopCounter();
  loopBLE();
}





