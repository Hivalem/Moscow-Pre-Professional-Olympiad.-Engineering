#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint8_t PIN_CE = 9;
const uint8_t PIN_CSN = 10;

RF24 radio(PIN_CE, PIN_CSN);
const uint64_t address = 0xF0F0F0F0E1LL;

const uint8_t CMD_START_SCAN[2] = {0x55, 0xAA};
const uint8_t CMD_TEST_MODE[2] = {0x5A, 0xA5};
const uint8_t CMD_LASER_ON[2] = {0xAA, 0xAA};
const uint8_t CMD_LASER_OFF[2] = {0xAA, 0x55};

struct DataPacket {
  uint16_t deviceId;
  int8_t azimuth;
  int8_t elevation;
  uint8_t mode;
  uint8_t counter;
};

enum OperationMode {
  MODE_IDLE = 0,
  MODE_HORIZONTAL = 1,
  MODE_VERTICAL = 2,
  MODE_DIAGONAL_A = 3,
  MODE_DIAGONAL_B = 4,
  MODE_LASER_OFF = 5,
  MODE_LASER_ON = 6,
  MODE_TEST_SYSTEM = 7,
  MODE_TEST_MOVEMENT = 8
};

unsigned long lastPacketTime = 0;
uint32_t goodPackets = 0;
uint32_t badPackets = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  
  pinMode(LED_BUILTIN, OUTPUT); 
  if (!initRadio()) {
    Serial.println(F("RADIO INIT FAILED! Check connections."));
    while(1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
  }
  
  Serial.println(F("Commands:"));
  Serial.println(F("  S - Start scan (0x55 0xAA)"));
  Serial.println(F("  T - Test device (0x5A 0xA5)"));
  Serial.println(F("  L - Laser ON (0xAA 0xAA)"));
  Serial.println(F("  O - Laser OFF (0xAA 0x55)"));
  Serial.println(F("  C - Check connection"));
  Serial.println(F("  R - Reset stats"));
  Serial.println(F("  I - Radio info"));
  Serial.println(F("  H - This help"));
  Serial.println();
}

bool initRadio() {
  Serial.println(F("Initializing radio..."));
  
  if (!radio.begin()) {
    Serial.println(F("ERROR: Radio module not found!"));
    return false;
  }
  
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.setPayloadSize(10);
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  
  radio.openWritingPipe(address);
  radio.openReadingPipe(1, address);
  
  radio.startListening();
  
  Serial.println(F("Radio initialized:"));
  Serial.print(F("  Channel: ")); Serial.println(radio.getChannel());
  Serial.print(F("  Address: 0x"));
  uint8_t* addr = (uint8_t*)&address;
  for (int i = 4; i >= 0; i--) {
    if (addr[i] < 0x10) Serial.print("0");
    Serial.print(addr[i], HEX);
  }
  Serial.println();
  Serial.print(F("  Payload size: ")); Serial.println(radio.getPayloadSize());
  
  return true;
}

void loop() {
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  if (millis() - lastBlink > 1000) {
    lastBlink = millis();
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState);
  }
  
  if (radio.available()) {
    receiveData();
  }
  
  if (Serial.available()) {
    char cmd = Serial.read();
    processCommand(cmd);
  }
  
  if (lastPacketTime > 0 && millis() - lastPacketTime > 10000) {
    Serial.println(F("--- NO SIGNAL---"));
    lastPacketTime = millis();
  }
}

void receiveData() {
  DataPacket packet;
  uint8_t bytesRead = radio.getPayloadSize();
  
  radio.read(&packet, sizeof(packet));
  
  bool isValid = true;
  
  if (packet.deviceId != 12345) {
    isValid = false;
    Serial.print(F("BAD ID: "));
    Serial.println(packet.deviceId);
  }
  
  if (packet.deviceId == 12345) {
    isValid = true;
  }
  
  if (isValid) {
    goodPackets++;
    lastPacketTime = millis();
    
    int8_t displayAz = packet.azimuth;
    int8_t displayEl = packet.elevation;
    
    Serial.print(F("["));
    Serial.print(packet.counter);
    Serial.print(F("] ID:"));
    Serial.print(packet.deviceId);
    Serial.print(F(" "));
    
    switch(packet.mode) {
      case MODE_IDLE: Serial.print(F("IDLE")); break;
      case MODE_HORIZONTAL: Serial.print(F("HORIZ")); break;
      case MODE_VERTICAL: Serial.print(F("VERT")); break;
      case MODE_DIAGONAL_A: Serial.print(F("DIAG-A")); break;
      case MODE_DIAGONAL_B: Serial.print(F("DIAG-B")); break;
      case MODE_LASER_OFF: Serial.print(F("LASER-OFF")); break;
      case MODE_LASER_ON: Serial.print(F("LASER-ON")); break;
      case MODE_TEST_SYSTEM: Serial.print(F("TEST-SYS")); break;
      case MODE_TEST_MOVEMENT: Serial.print(F("TEST-MOVE")); break;
      default: Serial.print(F("M")); Serial.print(packet.mode); break;
    }
    
    Serial.print(F(" AZ:"));
    if (displayAz >= 0) Serial.print(F("+"));
    Serial.print(displayAz);
    Serial.print(F("° EL:"));
    if (displayEl >= 0) Serial.print(F("+"));
    Serial.print(displayEl);
    Serial.println(F("°"));
    
    digitalWrite(LED_BUILTIN, HIGH);
    delay(10);
    digitalWrite(LED_BUILTIN, LOW);
    
  } else {
    badPackets++;
    if (badPackets % 10 == 0) {
      Serial.print(F("Bad packets: "));
      Serial.println(badPackets);
    }
  }
}

void processCommand(char cmd) {
  cmd = toupper(cmd);
  
  switch(cmd) {
    case 'S':
      sendCommand(CMD_START_SCAN, sizeof(CMD_START_SCAN), "Start scan command");
      break;
      
    case 'T':
      sendTestCommand();
      break;
      
    case 'L':
      sendCommand(CMD_LASER_ON, sizeof(CMD_LASER_ON), "Laser ON command");
      break;
      
    case 'O':
      sendCommand(CMD_LASER_OFF, sizeof(CMD_LASER_OFF), "Laser OFF command");
      break;
      
    case 'C':
      testConnection();
      break;
      
    case 'R':
      goodPackets = 0;
      badPackets = 0;
      Serial.println(F("Statistics reset"));
      break;
      
    case 'I':
      printStats();
      break;
      
    case 'H':
      printHelp();
      break;
      
    case '\n':
    case '\r':
      break;
      
    default:
      Serial.println(F("Unknown command. Type H for help."));
      break;
  }
}

void sendCommand(const uint8_t* cmd, uint8_t size, const char* description) {
  Serial.print(F("\n>> Sending "));
  Serial.print(description);
  Serial.println(F("..."));
  
  Serial.print(F("  Command: 0x"));
  Serial.print(cmd[0], HEX);
  Serial.print(F(" 0x"));
  Serial.println(cmd[1], HEX);
  
  radio.stopListening();
  delay(50);
  
  bool success = radio.write(cmd, size);
  
  radio.startListening();
  delay(50);
  
  if (success) {
    Serial.println(F("  Result: SUCCESS (Command sent)"));
  } else {
    Serial.println(F("  Result: FAILED (No ACK)"));
  }
}

void sendTestCommand() {
  Serial.println(F("\n>> Sending DEVICE TEST command..."));
  Serial.println(F("  Command: 0x5A 0xA5"));
  Serial.println(F("  Device will:"));
  Serial.println(F("  1. Move servos ±15° in all directions"));
  Serial.println(F("  2. Turn laser ON for 5 seconds"));
  Serial.println(F("  3. Turn laser OFF"));
  Serial.println(F("  4. Return to center position"));
  
  radio.stopListening();
  delay(50);
  
  bool success = radio.write(CMD_TEST_MODE, sizeof(CMD_TEST_MODE));
  
  radio.startListening();
  delay(50);
  
  if (success) {
    Serial.println(F("  Result: TEST COMMAND SENT SUCCESSFULLY"));
    Serial.println(F("  Wait for device test sequence..."));
  } else {
    Serial.println(F("  Result: FAILED TO SEND TEST COMMAND"));
  }
}

void testConnection() {
  Serial.println(F("\n>> Testing connection..."));
  
  // Пробуем отправить тестовый пакет
  uint8_t testPacket[] = {0xAA, 0x55, 0x00};
  
  radio.stopListening();
  
  unsigned long start = millis();
  bool success = radio.write(testPacket, sizeof(testPacket));
  unsigned long duration = millis() - start;
  
  radio.startListening();
  
  Serial.print(F("  Response time: "));
  Serial.print(duration);
  Serial.println(F("ms"));
  
  if (success) {
    Serial.println(F("  Result: SUCCESS (ACK received)"));
  } else {
    Serial.println(F("  Result: FAILED (no ACK)"));
  }
}

void printStats() {
  Serial.println(F("\n=== STATISTICS ==="));
  Serial.print(F("Good packets: "));
  Serial.println(goodPackets);
  Serial.print(F("Bad packets:  "));
  Serial.println(badPackets);
  
  uint32_t total = goodPackets + badPackets;
  if (total > 0) {
    Serial.print(F("Success rate:  "));
    Serial.print((goodPackets * 100) / total);
    Serial.println(F("%"));
  }
  
  Serial.print(F("Radio channel: "));
  Serial.println(radio.getChannel());
  Serial.print(F("Payload size:  "));
  Serial.println(radio.getPayloadSize());
  
  Serial.println(F("\n=== COMMANDS SUMMARY ==="));
  Serial.println(F("0x55 0xAA - Start scan"));
  Serial.println(F("0x5A 0xA5 - Device test"));
  Serial.println(F("0xAA 0xAA - Laser ON"));
  Serial.println(F("0xAA 0x55 - Laser OFF"));
  Serial.println();
}

void printHelp() {
  Serial.println(F("\n=== GROUND STATION COMMANDS ==="));
  Serial.println(F("S - Start satellite scan (0x55 0xAA)"));
  Serial.println(F("T - Test device servos/laser (0x5A 0xA5)"));
  Serial.println(F("L - Turn laser ON (0xAA 0xAA)"));
  Serial.println(F("O - Turn laser OFF (0xAA 0x55)"));
  Serial.println(F("C - Test radio connection"));
  Serial.println(F("R - Reset packet statistics"));
  Serial.println(F("I - Show radio info and stats"));
  Serial.println(F("H - Show this help"));
  Serial.println(F("\n=== DEVICE TEST MODE ==="));
  Serial.println(F("When 'T' command is sent, device will:"));
  Serial.println(F("  1. Move X servo ±15°"));
  Serial.println(F("  2. Move Y servo ±15°"));
  Serial.println(F("  3. Test diagonal movements"));
  Serial.println(F("  4. Turn laser ON for 5 sec"));
  Serial.println(F("  5. Turn laser OFF"));
  Serial.println(F("  6. Return to center (0°,0°)"));
  Serial.println();
}