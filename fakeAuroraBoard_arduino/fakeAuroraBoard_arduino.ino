#include <ArduinoBLE.h>

#define DISPLAY_NAME "kilter"
#define SERIAL_NUMBER "0001"
#define API_LEVEL 3

// Open Serial 115200
#define ENABLE_DEBUG_LOGS 1
#define ENABLE_PACKET_PARSER 1

#define AURORA_ADVERTISING_SERVICE_UUID "4488B571-7806-4DF6-BCFF-A2897E4953FF"

#define DATA_TRANSFER_SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define DATA_TRANSFER_RX_CHARACTERISTIC_UUID "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// This empty service is advertised so apps can detect an Aurora/Kilter board.
BLEService auroraAdvertisingService(AURORA_ADVERTISING_SERVICE_UUID);

// This service receives the actual LED packets.
BLEService dataTransferService(DATA_TRANSFER_SERVICE_UUID);

BLECharacteristic dataTransferRxCharacteristic(
  DATA_TRANSFER_RX_CHARACTERISTIC_UUID,
  BLEWrite | BLEWriteWithoutResponse,
  20
);

const uint16_t MAX_PACKET_SIZE = 265;

uint8_t packetBuffer[MAX_PACKET_SIZE];
uint16_t packetLength = 0;
int16_t expectedPacketLength = -1;

uint16_t currentMessageHoldCount = 0;
bool wasConnected = false;

void setup() {
  Serial.begin(115200);

  unsigned long serialStart = millis();
  while (!Serial && millis() - serialStart < 3000);

#if ENABLE_DEBUG_LOGS
  Serial.println();
  Serial.println("Booting fake Aurora / Kilter board...");
#endif

  char boardName[32];
  snprintf(
    boardName,
    sizeof(boardName),
    "%s#%s@%d",
    DISPLAY_NAME,
    SERIAL_NUMBER,
    API_LEVEL
  );

  if (!BLE.begin()) {
#if ENABLE_DEBUG_LOGS
    Serial.println("Starting BLE failed");
#endif
    while (true);
  }

  BLE.setLocalName(boardName);
  BLE.setDeviceName(boardName);

  dataTransferRxCharacteristic.setEventHandler(BLEWritten, onDataTransferWritten);

  dataTransferService.addCharacteristic(dataTransferRxCharacteristic);

  BLE.addService(auroraAdvertisingService);
  BLE.addService(dataTransferService);

  BLE.setAdvertisedService(auroraAdvertisingService);

  BLE.advertise();

#if ENABLE_DEBUG_LOGS
  Serial.print("Advertising as: ");
  Serial.println(boardName);
  Serial.print("Advertising service: ");
  Serial.println(AURORA_ADVERTISING_SERVICE_UUID);
  Serial.print("Data transfer service: ");
  Serial.println(DATA_TRANSFER_SERVICE_UUID);
  Serial.print("RX characteristic: ");
  Serial.println(DATA_TRANSFER_RX_CHARACTERISTIC_UUID);
  Serial.print("API level: ");
  Serial.println(API_LEVEL);
#endif
}

void loop() {
  BLE.poll();

  bool isConnected = BLE.connected();

#if ENABLE_DEBUG_LOGS
  if (isConnected && !wasConnected) {
    Serial.println("BLE central connected");
  }

  if (!isConnected && wasConnected) {
    Serial.println("BLE central disconnected");
    Serial.println("Restarting advertising...");
    BLE.advertise();
  }
#endif

  wasConnected = isConnected;

  // Optional serial API-level response.
  // This matches the original fake_kilter_board behaviour:
  // if a desktop helper sends byte 4, respond with [4, API_LEVEL].
  if (Serial.available() > 0) {
    uint8_t value = Serial.read();

    if (value == 4) {
      sendApiLevelOverSerial();
    }
  }
}

void onDataTransferWritten(BLEDevice central, BLECharacteristic characteristic) {
  const uint8_t* data = characteristic.value();
  int length = characteristic.valueLength();

#if ENABLE_DEBUG_LOGS
  Serial.print("BLE write received, bytes: ");
  Serial.println(length);
#endif

#if ENABLE_PACKET_PARSER
  for (int i = 0; i < length; i++) {
    processIncomingByte(data[i]);
  }
#else
  Serial.write(data, length);
#endif
}

// -----------------------------------------------------------------------------
// Packet parser
//
// Packet format:
// [1][payload length][checksum][2][payload...][3]
//
// API level 3 payload:
// [packet type][hold...]
// packet type:
//   R = first packet
//   Q = middle packet
//   S = last packet
//   T = only packet
//
// hold:
//   byte 1: position low byte
//   byte 2: position high byte
//   byte 3: compressed RGB, RRR GGG BB
// -----------------------------------------------------------------------------

void processIncomingByte(uint8_t value) {
  if (packetLength == 0) {
    if (value != 1) {
#if ENABLE_DEBUG_LOGS
      Serial.print("Ignoring byte before packet start: ");
      Serial.println(value);
#endif
      return;
    }

    expectedPacketLength = -1;
  }

  if (packetLength >= MAX_PACKET_SIZE) {
#if ENABLE_DEBUG_LOGS
    Serial.println("Packet buffer overflow, resetting parser");
#endif
    resetPacketParser();
    return;
  }

  packetBuffer[packetLength++] = value;

  if (packetLength == 2) {
    uint8_t payloadLength = packetBuffer[1];

    // start + payloadLength + checksum + separator + payload + end
    expectedPacketLength = payloadLength + 5;

    if (expectedPacketLength > MAX_PACKET_SIZE) {
#if ENABLE_DEBUG_LOGS
      Serial.println("Packet too large, resetting parser");
#endif
      resetPacketParser();
      return;
    }
  }

  if (expectedPacketLength > 0 && packetLength == expectedPacketLength) {
    parsePacket(packetBuffer, packetLength);
    resetPacketParser();
  }
}

void resetPacketParser() {
  packetLength = 0;
  expectedPacketLength = -1;
}

void parsePacket(const uint8_t* packet, uint16_t length) {
  if (length < 6) {
#if ENABLE_DEBUG_LOGS
    Serial.println("Packet too short");
#endif
    return;
  }

  uint8_t payloadLength = packet[1];
  uint8_t checksum = packet[2];

  if (packet[0] != 1 || packet[3] != 2 || packet[length - 1] != 3) {
#if ENABLE_DEBUG_LOGS
    Serial.println("Invalid packet framing");
#endif
    return;
  }

  const uint8_t* payload = &packet[4];

  if (!isChecksumValid(payload, payloadLength, checksum)) {
#if ENABLE_DEBUG_LOGS
    Serial.println("Invalid packet checksum");
#endif
    return;
  }

  parseApiLevel3Payload(payload, payloadLength);
}

bool isChecksumValid(const uint8_t* data, uint8_t length, uint8_t expectedChecksum) {
  uint16_t sum = 0;

  for (uint8_t i = 0; i < length; i++) {
    sum = (sum + data[i]) & 0xFF;
  }

  uint8_t calculatedChecksum = (~sum) & 0xFF;

  return calculatedChecksum == expectedChecksum;
}

void parseApiLevel3Payload(const uint8_t* payload, uint8_t length) {
  if (length < 1) {
    return;
  }

  char packetType = (char)payload[0];

  bool isFirstPacket = packetType == 'R';
  bool isMiddlePacket = packetType == 'Q';
  bool isLastPacket = packetType == 'S';
  bool isOnlyPacket = packetType == 'T';

  if (!isFirstPacket && !isMiddlePacket && !isLastPacket && !isOnlyPacket) {
#if ENABLE_DEBUG_LOGS
    Serial.print("Unknown packet type: ");
    Serial.println(packetType);
#endif
    return;
  }

#if ENABLE_DEBUG_LOGS
  Serial.print("Packet type: ");
  Serial.println(packetType);
#endif

  if (isFirstPacket || isOnlyPacket) {
    clearBoard();
    currentMessageHoldCount = 0;
  }

  for (uint8_t i = 1; i + 2 < length; i += 3) {
    uint16_t position = payload[i] | (payload[i + 1] << 8);
    uint8_t colorByte = payload[i + 2];

    uint8_t red = decodeRed(colorByte);
    uint8_t green = decodeGreen(colorByte);
    uint8_t blue = decodeBlue(colorByte);

    handleHold(position, red, green, blue);
    currentMessageHoldCount++;
  }

  if (isLastPacket || isOnlyPacket) {
    showBoard();
  }
}

uint8_t decodeRed(uint8_t colorByte) {
  uint8_t value = (colorByte >> 5) & 0b00000111;
  return map(value, 0, 7, 0, 255);
}

uint8_t decodeGreen(uint8_t colorByte) {
  uint8_t value = (colorByte >> 2) & 0b00000111;
  return map(value, 0, 7, 0, 255);
}

uint8_t decodeBlue(uint8_t colorByte) {
  uint8_t value = colorByte & 0b00000011;
  return map(value, 0, 3, 0, 255);
}

// -----------------------------------------------------------------------------
// Board output hooks
// Replace these with your LED implementation later.
// -----------------------------------------------------------------------------

void clearBoard() {
#if ENABLE_DEBUG_LOGS
  Serial.println("Clear board");
#endif
}

void handleHold(uint16_t position, uint8_t red, uint8_t green, uint8_t blue) {
#if ENABLE_DEBUG_LOGS
  Serial.print("Position: ");
  Serial.print(position);
  Serial.print(" RGB: ");
  Serial.print(red);
  Serial.print(",");
  Serial.print(green);
  Serial.print(",");
  Serial.println(blue);
#endif
}

void showBoard() {
#if ENABLE_DEBUG_LOGS
  Serial.print("Show board, holds: ");
  Serial.println(currentMessageHoldCount);
#endif
}

void sendApiLevelOverSerial() {
  Serial.write(4);
  Serial.write(API_LEVEL);
}