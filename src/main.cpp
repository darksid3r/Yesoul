#include <Arduino.h>
#include <NimBLEDevice.h>

// GPIO pin for the LED
const int LED_PIN = 2; // Built-in LED on most ESP32 boards

// Variables for power, cadence, and resistance
short powerInstantaneous = 0;
short cadenceInstantaneous = 0;
short resistance = 0;
float powerScale = 1.00; // Allow calibration via serial or characteristic
unsigned char bleBuffer[8];
unsigned short revolutions = 0;
unsigned short timestamp = 0;
unsigned short flags = 0x20;   // Default flags for power and cadence
byte sensorlocation = 0x0D;    // Crank sensor location
bool isWatchConnected = false; // Track connection state for the watch
bool isBikeConnected = false;  // Track connection state for the bike

static NimBLEServer *pServer;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *foundBike = nullptr;

// BLE characteristics
NimBLECharacteristic *CyclingPowerMeasurement;
NimBLECharacteristic *CyclingPowerFeature;
NimBLECharacteristic *CyclingPowerSensorLocation;

// BLE characteristics for Fitness Machine Service
NimBLEService *fitnessMachineService;
NimBLECharacteristic *fitnessMachineFeature;
NimBLECharacteristic *indoorBikeData;
NimBLECharacteristic *trainingStatus;

// Add these at the top with other global variables
const unsigned long UPDATE_INTERVAL_MS = 4000; // Send to watch every 100ms
float cumulativeRevolutions = 0.0f;           // Use float for accurate revolution counting
float lastCadence = 0.0f;
float currentCadence = 0.0f;
unsigned long lastBikeUpdate = 0;

// Struct for storing latest bike data
struct BikeData
{
  short power;
  short cadence;
  short resistance;
  unsigned long lastUpdateTime;
} latestBikeData = {0, 0, 0, 0};

// Callback classes for BLE server
class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer) override
  {
    Serial.println("Client connected (Watch).");
    isWatchConnected = true;
    NimBLEDevice::startAdvertising();
  };

  void onDisconnect(NimBLEServer *pServer) override
  {
    Serial.println("Client disconnected (Watch).");
    isWatchConnected = false;
    NimBLEDevice::startAdvertising();
  };
};

// Callback for detecting bike during BLE scanning
class BikeAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice *advertisedDevice) override
  {
    Serial.printf("Found BLE Device: %s\n", advertisedDevice->toString().c_str());

    // Check if the device advertises the Fitness Machine Service (UUID: 0x1826)
    if (advertisedDevice->haveServiceUUID() &&
        advertisedDevice->isAdvertisingService(NimBLEUUID("1826")))
    {
      Serial.println("Found the bike!");
      foundBike = advertisedDevice;    // Save the reference to the bike
      NimBLEDevice::getScan()->stop(); // Stop scanning once the bike is found
    }
  }
};

// Function to send updates to the watch
void sendWatchUpdate()
{
  if (!isWatchConnected)
    return;

  static unsigned long lastWatchUpdate = 0;
  static float lastSentRevolutions = cumulativeRevolutions;

  unsigned long now = millis();
  unsigned long interval = now - lastWatchUpdate;

  if (interval >= UPDATE_INTERVAL_MS)
  {
    // Convert cadence to float only when calculating revolutions
    float revsPerSecond = (float)latestBikeData.cadence / 60.0f;
    float partialRevs = revsPerSecond * (interval / 1000.0f);

    // Add small increment to cumulative revolutions
    cumulativeRevolutions += partialRevs;

    // Update timestamp by actual interval
    timestamp = (timestamp + (interval * 1024 / 1000)) & 0xFFFF;

    uint16_t bleRevolutions = (uint16_t)cumulativeRevolutions & 0xFFFF;

    // 1. Send Cycling Power Measurement
    int bufferIndex = 0;
    bleBuffer[bufferIndex++] = flags & 0xFF;
    bleBuffer[bufferIndex++] = (flags >> 8) & 0xFF;
    bleBuffer[bufferIndex++] = latestBikeData.power & 0xFF;
    bleBuffer[bufferIndex++] = (latestBikeData.power >> 8) & 0xFF;

    if (flags & 0x20)
    {
      bleBuffer[bufferIndex++] = bleRevolutions & 0xFF;
      bleBuffer[bufferIndex++] = (bleRevolutions >> 8) & 0xFF;
      bleBuffer[bufferIndex++] = timestamp & 0xFF;
      bleBuffer[bufferIndex++] = (timestamp >> 8) & 0xFF;
    }

    CyclingPowerMeasurement->setValue(bleBuffer, bufferIndex);
    CyclingPowerMeasurement->notify();

    // 2. Send Indoor Bike Data
    if (indoorBikeData->getSubscribedCount() > 0)
    {
      uint8_t bikeData[8];
      uint16_t ftmsFlags = 0x0004; // Resistance level present flag

      bikeData[0] = ftmsFlags & 0xFF;
      bikeData[1] = (ftmsFlags >> 8) & 0xFF;
      bikeData[2] = 0;                         // More flags if needed
      bikeData[3] = 0;                         // More flags if needed
      bikeData[4] = latestBikeData.resistance; // Resistance level

      indoorBikeData->setValue(bikeData, 5);
      indoorBikeData->notify();
    }

    Serial.printf("Update sent - Power: %d W, Cadence: %d RPM, Resistance: %d, Revs: %.2f, Interval: %lu ms\n",
                  latestBikeData.power, latestBikeData.cadence,
                  latestBikeData.resistance, cumulativeRevolutions, interval);

    lastSentRevolutions = cumulativeRevolutions;
    lastWatchUpdate = now;
  }
}

// Simplified notification callback that just stores the data
static void notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData, size_t length, bool isNotify)
{

  unsigned long now = millis();
  Serial.printf("Time between bike updates: %lu ms\n", now - lastBikeUpdate);
  lastBikeUpdate = now;

  // Store the latest data
  latestBikeData.power = (pData[11] | (pData[12] << 8)) * powerScale;
  latestBikeData.cadence = (pData[4] | (pData[5] << 8)) / 2;
  latestBikeData.resistance = pData[9];
  latestBikeData.lastUpdateTime = now;

  Serial.printf("Received from bike: Power=%d W, Cadence=%d RPM, Resistance=%d\n",
                latestBikeData.power, latestBikeData.cadence, latestBikeData.resistance);
}

// Attempt to connect to the bike
bool connectToBike()
{
  if (!foundBike)
  {
    Serial.println("No bike found yet. Retrying...");
    return false;
  }

  Serial.println("Attempting to connect to the bike...");

  // Create BLE client
  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created BLE client");

  // Set connection parameters for faster updates
  pClient->setConnectionParams(7.5f, 7.5f, 0, 500); // min interval, max interval, latency, timeout
  // Requesting 7.5ms intervals (7.5 * 1.25ms = 9.375ms)

  // Attempt to connect to the bike
  if (!pClient->connect(foundBike))
  {
    Serial.println("Failed to connect to the bike. Restarting scan...");
    foundBike = nullptr; // Reset foundBike
    return false;
  }
  Serial.println(" - Connected to bike");

  // Obtain reference to the service
  BLERemoteService *pRemoteService = pClient->getService("1826"); // Fitness Machine UUID
  if (!pRemoteService)
  {
    Serial.println("Failed to find the bike's Fitness Machine Service.");
    pClient->disconnect();
    foundBike = nullptr; // Reset foundBike
    return false;
  }

  // Obtain reference to the characteristic
  pRemoteCharacteristic = pRemoteService->getCharacteristic("2ad2"); // Indoor Bike UUID
  if (!pRemoteCharacteristic)
  {
    Serial.println("Failed to find the bike's Indoor Bike Characteristic.");
    pClient->disconnect();
    foundBike = nullptr; // Reset foundBike
    return false;
  }

  // Register for notifications
  if (pRemoteCharacteristic->canNotify())
  {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    Serial.println("Notifications enabled for bike.");
  }

  isBikeConnected = true; // Set bike connection flag
  return true;
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting NimBLE Server");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  NimBLEDevice::init("Yesoul_CP");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create Cycling Power Service
  NimBLEService *cyclingPowerService = pServer->createService("1818");

  // Create characteristics for Cycling Power Service
  CyclingPowerFeature = cyclingPowerService->createCharacteristic(
      "2A65",
      NIMBLE_PROPERTY::READ);
  CyclingPowerSensorLocation = cyclingPowerService->createCharacteristic(
      "2A5D",
      NIMBLE_PROPERTY::READ);
  CyclingPowerMeasurement = cyclingPowerService->createCharacteristic(
      "2A63",
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Create Fitness Machine Service
  fitnessMachineService = pServer->createService("1826");

  // Create characteristics for Fitness Machine Service
  fitnessMachineFeature = fitnessMachineService->createCharacteristic(
      "2ACC",
      NIMBLE_PROPERTY::READ);

  indoorBikeData = fitnessMachineService->createCharacteristic(
      "2AD2",
      NIMBLE_PROPERTY::NOTIFY);

  trainingStatus = fitnessMachineService->createCharacteristic(
      "2AD3",
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Set initial values
  unsigned char cpFeatureValue[4] = {0x00, 0x00, 0x00, 0x08};
  CyclingPowerFeature->setValue(cpFeatureValue, 4);

  unsigned char locationValue[1] = {sensorlocation & 0xff};
  CyclingPowerSensorLocation->setValue(locationValue, 1);

  uint32_t ftmsFeatures = 0x00000008; // Resistance level supported
  fitnessMachineFeature->setValue((uint8_t *)&ftmsFeatures, 4);

  // Start both services
  cyclingPowerService->start();
  fitnessMachineService->start();

  // Start advertising both services
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(cyclingPowerService->getUUID());
  pAdvertising->addServiceUUID(fitnessMachineService->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->start();
  Serial.println("Advertising Started");
}

void loop()
{
  static unsigned long lastScanTime = 0;

  // Scan for the bike if not connected
  if (!isBikeConnected && millis() - lastScanTime > 5000)
  { // Scan every 5 seconds
    Serial.println("Scanning for bike...");
    NimBLEDevice::getScan()->setAdvertisedDeviceCallbacks(new BikeAdvertisedDeviceCallbacks());
    NimBLEDevice::getScan()->setInterval(1349);
    NimBLEDevice::getScan()->setWindow(449);
    NimBLEDevice::getScan()->setActiveScan(true);
    NimBLEDevice::getScan()->start(5, false); // Scan for 5 seconds
    lastScanTime = millis();
  }

  // Attempt to connect to the bike if found
  if (foundBike && !isBikeConnected)
  {
    connectToBike();
  }

  // Send updates to the watch at the configured interval
  sendWatchUpdate();

  // Handle LED feedback
  if (isBikeConnected)
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }
}
