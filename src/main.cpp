#include <Arduino.h>
#include <NimBLEDevice.h>

// GPIO pin for the LED
const int LED_PIN = 2; // Built-in LED on most ESP32 boards

// Variables for power, cadence, and resistance
short powerInstantaneous = 0;
short cadenceInstantaneous = 0;
short resistance = 0;
float powerScale = 1.28; // Allow calibration via serial or characteristic
unsigned char bleBuffer[8];
unsigned short revolutions = 0;
unsigned short timestamp = 0;
unsigned short flags = 0x20; // Default flags for power and cadence
byte sensorlocation = 0x0D;  // Crank sensor location
bool isWatchConnected = false; // Track connection state for the watch
bool isBikeConnected = false;  // Track connection state for the bike

static NimBLEServer *pServer;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *foundBike = nullptr;

// BLE characteristics
NimBLECharacteristic *CyclingPowerMeasurement;
NimBLECharacteristic *CyclingPowerFeature;
NimBLECharacteristic *CyclingPowerSensorLocation;

// Callback classes for BLE server
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer *pServer) override {
    Serial.println("Client connected (Watch).");
    isWatchConnected = true;
    NimBLEDevice::startAdvertising();
  };

  void onDisconnect(NimBLEServer *pServer) override {
    Serial.println("Client disconnected (Watch).");
    isWatchConnected = false;
    NimBLEDevice::startAdvertising();
  };
};

// Callback for detecting bike during BLE scanning
class BikeAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice *advertisedDevice) override {
    Serial.printf("Found BLE Device: %s\n", advertisedDevice->toString().c_str());

    // Check if the device advertises the Fitness Machine Service (UUID: 0x1826)
    if (advertisedDevice->haveServiceUUID() &&
        advertisedDevice->isAdvertisingService(NimBLEUUID("1826"))) {
      Serial.println("Found the bike!");
      foundBike = advertisedDevice; // Save the reference to the bike
      NimBLEDevice::getScan()->stop(); // Stop scanning once the bike is found
    }
  }
};

// Callback for receiving data from the bike
static void notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData, size_t length, bool isNotify) {
  powerInstantaneous = pData[11] | pData[12] << 8; // Extract power data
  powerInstantaneous = powerInstantaneous * powerScale; // Apply scaling
  cadenceInstantaneous = (pData[4] | pData[5] << 8) / 2; // Convert to RPM
  resistance = pData[9]; // Extract resistance
  Serial.printf("Power: %d W, Cadence: %d RPM, Resistance: %d\n",
                powerInstantaneous, cadenceInstantaneous, resistance);

  // Add this section to send data to the watch
  if (isWatchConnected) {
    // Format data according to Cycling Power Measurement characteristic
    bleBuffer[0] = flags & 0xFF;        // Flags (lower byte)
    bleBuffer[1] = flags >> 8;          // Flags (upper byte)
    bleBuffer[2] = powerInstantaneous & 0xFF;  // Power (lower byte)
    bleBuffer[3] = powerInstantaneous >> 8;    // Power (upper byte)
    
    // If using crank revolution data (optional)
    if (flags & 0x20) {  // Check if crank data flag is set
      bleBuffer[4] = revolutions & 0xFF;       // Crank revolutions (lower byte)
      bleBuffer[5] = revolutions >> 8;         // Crank revolutions (upper byte)
      bleBuffer[6] = timestamp & 0xFF;         // Last crank event time (lower byte)
      bleBuffer[7] = timestamp >> 8;           // Last crank event time (upper byte)
    }
    
    // Send notification to the watch
    CyclingPowerMeasurement->setValue(bleBuffer, (flags & 0x20) ? 8 : 4);
    CyclingPowerMeasurement->notify();
  }
}

// Attempt to connect to the bike
bool connectToBike() {
  if (!foundBike) {
    Serial.println("No bike found yet. Retrying...");
    return false;
  }

  Serial.println("Attempting to connect to the bike...");

  // Create BLE client
  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created BLE client");

  // Attempt to connect to the bike
  if (!pClient->connect(foundBike)) {
    Serial.println("Failed to connect to the bike. Restarting scan...");
    foundBike = nullptr; // Reset foundBike
    return false;
  }
  Serial.println(" - Connected to bike");

  // Obtain reference to the service
  BLERemoteService *pRemoteService = pClient->getService("1826"); // Fitness Machine UUID
  if (!pRemoteService) {
    Serial.println("Failed to find the bike's Fitness Machine Service.");
    pClient->disconnect();
    foundBike = nullptr; // Reset foundBike
    return false;
  }

  // Obtain reference to the characteristic
  pRemoteCharacteristic = pRemoteService->getCharacteristic("2ad2"); // Indoor Bike UUID
  if (!pRemoteCharacteristic) {
    Serial.println("Failed to find the bike's Indoor Bike Characteristic.");
    pClient->disconnect();
    foundBike = nullptr; // Reset foundBike
    return false;
  }

  // Register for notifications
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
    Serial.println("Notifications enabled for bike.");
  }

  isBikeConnected = true; // Set bike connection flag
  return true;
}

void setup() {
  Serial.begin(115200);
  Serial.println("Starting NimBLE Server");

  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW); // Ensure LED is off initially

  // Initialize BLE device
  NimBLEDevice::init("Yesoul_CP");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Set BLE power to +9dB

  // Create BLE server and set callbacks
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create the Cycling Power Service
  NimBLEService *pService = pServer->createService("1818"); // CPS UUID

  // Create characteristics
  CyclingPowerFeature = pService->createCharacteristic("2A65", NIMBLE_PROPERTY::READ);
  CyclingPowerSensorLocation = pService->createCharacteristic("2A5D", NIMBLE_PROPERTY::READ);
  CyclingPowerMeasurement = pService->createCharacteristic("2A63", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // Set initial values for characteristics
  unsigned char featureValue[4] = {0x00, 0x00, 0x00, 0x08}; // Example feature flags
  CyclingPowerFeature->setValue(featureValue, 4);

  unsigned char locationValue[1] = {sensorlocation & 0xff};
  CyclingPowerSensorLocation->setValue(locationValue, 1);

  // Start the service
  pService->start();

  // Start advertising
  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->setScanResponse(true); // Include full response in scan
  pAdvertising->start();
  Serial.println("Advertising Started");
}

void loop() {
  static unsigned long lastScanTime = 0;

  // Scan for the bike if not connected
  if (!isBikeConnected && millis() - lastScanTime > 5000) { // Scan every 5 seconds
    Serial.println("Scanning for bike...");
    NimBLEDevice::getScan()->setAdvertisedDeviceCallbacks(new BikeAdvertisedDeviceCallbacks());
    NimBLEDevice::getScan()->setInterval(1349);
    NimBLEDevice::getScan()->setWindow(449);
    NimBLEDevice::getScan()->setActiveScan(true);
    NimBLEDevice::getScan()->start(5, false); // Scan for 5 seconds
    lastScanTime = millis();
  }

  // Attempt to connect to the bike if found
  if (foundBike && !isBikeConnected) {
    connectToBike();
  }

  // Handle LED feedback
  if (isBikeConnected) {
    digitalWrite(LED_PIN, HIGH); // Turn LED on when connected to the bike
  } else {
    digitalWrite(LED_PIN, LOW); // Turn LED off otherwise
  }
}
