#include <Servo.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

const uint16_t NODE_ID = 12345;
const uint8_t PIN_SERVO_X = 7;
const uint8_t PIN_SERVO_Y = 8;
const uint8_t PIN_LASER = 2;
const uint8_t PIN_CE = 10;
const uint8_t PIN_CSN = 9;

const int8_t OFFSET_X = -3;
const int8_t OFFSET_Y = -4;

//Радио
RF24 radio(PIN_CE, PIN_CSN);
const uint64_t address = 0xF0F0F0F0E1LL;

//Структура данных
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

Servo actuatorX;
Servo actuatorY;

uint8_t packetCounter = 0;
bool systemActive = false;
bool laserEnabled = true;

void updateServos(int8_t x, int8_t y);
void executeRoutine();
void runX(int8_t dirX, int8_t dirY, uint8_t mode_idx);
void runY(int8_t dirX, int8_t dirY, uint8_t mode_idx);
void runDlv(int8_t dirX, int8_t dirY, uint8_t mode_idx);
void runDvl(int8_t dirX, int8_t dirY, uint8_t mode_idx);
void initRadio();
void sendTelemetry(int8_t az, int8_t el, uint8_t mode);
void runDeviceTest();

void setup() {
  Serial.begin(115200);

  actuatorX.attach(PIN_SERVO_X);
  actuatorY.attach(PIN_SERVO_Y);
  Serial.println(F("[OK] Servos initialized"));
  
  pinMode(PIN_LASER, OUTPUT);
  delay(2000);
  digitalWrite(PIN_LASER, HIGH);
  laserEnabled = true;
  Serial.println(F("[OK] Laser ON (default)"));
  
  initRadio();
  
  Serial.print(F("[INFO] Initial position - OFFSET_X: "));
  Serial.print(OFFSET_X);
  Serial.print(F(", OFFSET_Y: "));
  Serial.println(OFFSET_Y);
  
  updateServos(OFFSET_X, OFFSET_Y);
  
  Serial.println(F("[INFO] Waiting for commands..."));
  
  Serial.println(F("=== AVAILABLE COMMANDS ==="));
  Serial.println(F("  0x55 0xAA - Start scan"));
  Serial.println(F("  0x5A 0xA5 - Device test   ← НОВАЯ КОМАНДА"));
  Serial.println(F("  0xAA 0x55 - Laser OFF"));
  Serial.println(F("  0xAA 0xAA - Laser ON"));
  Serial.println(F("============================="));
}

void loop() {
  if (radio.available()) {
    uint8_t command[3];
    radio.read(&command, sizeof(command));
    
    Serial.print(F("[RADIO] CMD: 0x"));
    Serial.print(command[0], HEX);
    Serial.print(F(" 0x"));
    Serial.print(command[1], HEX);
    Serial.println();
    
    if (command[0] == 0x55 && command[1] == 0xAA) {
      Serial.println(F("[CMD] START command received!"));
      systemActive = true;
      executeRoutine();
      systemActive = false;
    }
    else if (command[0] == 0x5A && command[1] == 0xA5) {
      Serial.println(F("[CMD] TEST MODE command received!"));
      systemActive = true;
      runDeviceTest();
      systemActive = false;
    }
    else if (command[0] == 0xAA && command[1] == 0x55) {
      digitalWrite(PIN_LASER, LOW);
      laserEnabled = false;
      Serial.println(F("[CMD] LASER OFF command received!"));
      sendTelemetry(0, 0, MODE_LASER_OFF);
    }
    else if (command[0] == 0xAA && command[1] == 0xAA) {
      digitalWrite(PIN_LASER, HIGH);
      laserEnabled = true;
      Serial.println(F("[CMD] LASER ON command received!"));
      sendTelemetry(0, 0, MODE_LASER_ON);
    }
  }
  
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    if (!systemActive) {
      sendTelemetry(0, 0, MODE_IDLE);
    }
  }
}

void runDeviceTest() {
  Serial.println(F("\n=== DEVICE TEST MODE ==="));
  Serial.println(F("[TEST] 1. Testing X-axis movement (±15°)"));
  
  updateServos(15, 0);
  sendTelemetry(15, 0, MODE_TEST_MOVEMENT);
  delay(1000);
  
  updateServos(-15, 0);
  sendTelemetry(-15, 0, MODE_TEST_MOVEMENT);
  delay(1000);
  
  Serial.println(F("[TEST] 2. Testing Y-axis movement (±15°)"));
  
  updateServos(0, 15);
  sendTelemetry(0, 15, MODE_TEST_MOVEMENT);
  delay(1000);
  
  updateServos(0, -15);
  sendTelemetry(0, -15, MODE_TEST_MOVEMENT);
  delay(1000);
  
  Serial.println(F("[TEST] 3. Testing diagonal movements"));
  ;
  updateServos(15, 15);
  sendTelemetry(15, 15, MODE_TEST_MOVEMENT);
  delay(1000);
  
  updateServos(-15, -15);
  sendTelemetry(-15, -15, MODE_TEST_MOVEMENT);
  delay(1000);
  
  updateServos(15, -15);
  sendTelemetry(15, -15, MODE_TEST_MOVEMENT);
  delay(1000);
  
  updateServos(-15, 15);
  sendTelemetry(-15, 15, MODE_TEST_MOVEMENT);
  delay(1000);
  
  Serial.println(F("[TEST] 4. Testing laser (5 seconds)"));
  
  Serial.println(F("[TEST]   Turning laser ON"));
  digitalWrite(PIN_LASER, HIGH);
  laserEnabled = true;
  sendTelemetry(0, 0, MODE_TEST_SYSTEM);
  
  for (int i = 5; i > 0; i--) {
    Serial.print(F("[TEST]     Laser ON - "));
    Serial.print(i);
    delay(1000);
  }
  
  Serial.println(F("[TEST]   Turning laser OFF"));
  digitalWrite(PIN_LASER, LOW);
  laserEnabled = false;
  sendTelemetry(0, 0, MODE_LASER_OFF);
  delay(1000);
  
  Serial.println(F("[TEST] 5. Returning to initial position"));
  updateServos(OFFSET_X, OFFSET_Y);
  sendTelemetry(0, 0, MODE_IDLE);
  
  Serial.println(F("[TEST] === DEVICE TEST COMPLETE ==="));
  Serial.println(F("[TEST] All systems: OK"));
}

void initRadio() {
  Serial.println(F("[INFO] Initializing radio..."));
  
  if (!radio.begin()) {
    Serial.println(F("[ERROR] Radio hardware not found!"));
    while(1);
  }
  
  radio.setAutoAck(true);
  radio.setRetries(5, 15);
  radio.setPayloadSize(10);
  radio.setChannel(76);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_1MBPS);
  
  radio.openReadingPipe(1, address);
  radio.openWritingPipe(address);
  
  radio.powerUp();
  radio.startListening();
  
  Serial.print(F("[OK] Channel: ")); Serial.println(radio.getChannel());
  Serial.print(F("[OK] Address: 0x")); 
  uint8_t* addr = (uint8_t*)&address;
  for (int i = 4; i >= 0; i--) {
    if (addr[i] < 0x10) Serial.print("0");
    Serial.print(addr[i], HEX);
  }
  Serial.println();
  
  Serial.println(F("[OK] Radio READY"));
}

void executeRoutine() {
  Serial.println(F("\n=== STARTING SCAN ==="));

  Serial.println(F("Mode 1: Horizontal sweep"));
  runX(1, 0, MODE_HORIZONTAL);

  Serial.println(F("Mode 2: Vertical sweep"));
  runY(0, 1, MODE_VERTICAL);

  Serial.println(F("Mode 3: Diagonal A (+X, +Y)"));
  runDlv(1, 1, MODE_DIAGONAL_A);

  Serial.println(F("Mode 4: Diagonal B (+X, -Y)"));
  runDvl(1, -1, MODE_DIAGONAL_B);

  Serial.println(F("Returning to initial position"));
  updateServos(OFFSET_X, OFFSET_Y);
  sendTelemetry(0, 0, MODE_IDLE);
  
  Serial.println(F("=== SCAN COMPLETE ==="));
}

void runX(int8_t dirX, int8_t dirY, uint8_t mode_idx) {
    const int limit = 40;
    const int step = 10;

    for (int i = -limit; i <= limit; i += step) {
        int8_t targetX = OFFSET_X + ((dirX == 0) ? 0 : (i * dirX));
        int8_t targetY = OFFSET_Y + ((dirY == 0) ? 0 : (i * dirY));

        targetX = constrain(targetX, -40, 40);

        updateServos(targetX, targetY);
        sendTelemetry(targetX, targetY, mode_idx);
        
        delay(1000);
    }
}

void runY(int8_t dirX, int8_t dirY, uint8_t mode_idx) {
    const int limit = 40;
    const int step = 10;

    for (int i = -limit; i <= limit; i += step) {
        int8_t targetX = OFFSET_X + ((dirX == 0) ? 0 : (i * dirX));
        int8_t targetY = OFFSET_Y + ((dirY == 0) ? 0 : (i * dirY));

        targetY = constrain(targetY, -40, 40);

        updateServos(targetX, targetY);
        sendTelemetry(targetX, targetY, mode_idx);
        
        delay(1000);
    }
}

void runDlv(int8_t dirX, int8_t dirY, uint8_t mode_idx) {
    const int limit = 40;
    const int step = 10;

    for (int i = -limit; i <= limit; i += step) {
        int8_t targetX = OFFSET_X + ((dirX == 0) ? 0 : (i * dirX));
        int8_t targetY = OFFSET_Y + ((dirY == 0) ? 0 : (i * dirY));

        targetX = constrain(targetX, -40, 40);
        targetY = constrain(targetY, -40, 40);

        updateServos(targetX, targetY);
        sendTelemetry(targetX, targetY, mode_idx);
        
        delay(1000);
    }
}

void runDvl(int8_t dirX, int8_t dirY, uint8_t mode_idx) {
    const int limit = 40;
    const int step = 10;

    for (int i = -limit; i <= limit; i += step) {
        int8_t targetX = OFFSET_X + ((dirX == 0) ? 0 : (i * dirX));
        int8_t targetY = OFFSET_Y + ((dirY == 0) ? 0 : (i * dirY));

        targetX = constrain(targetX, -40, 40);
        targetY = constrain(targetY, -40, 40);

        updateServos(targetX, targetY);
        sendTelemetry(targetX, targetY, mode_idx);
        
        delay(1000);
    }
}

// Низкоуровневое управление сервоприводами
void updateServos(int8_t x, int8_t y) {
    int pwmX = 90 - x;
    int pwmY = 90 - y;

    pwmX = constrain(pwmX, 30, 150);
    pwmY = constrain(pwmY, 30, 150);

    actuatorX.write(pwmX);
    actuatorY.write(pwmY);

    Serial.print(F("[SERVO] Move to: X="));
    Serial.print(x);
    Serial.print(F("° Y="));
    Serial.print(y);
    Serial.print(F("° (PWM: "));
    Serial.print(pwmX);
    Serial.print(F(","));
    Serial.print(pwmY);
    Serial.println(F(")"));

    delay(3000); // Даем время сервам на перемещение
}

void sendTelemetry(int8_t az, int8_t el, uint8_t mode) {
  DataPacket packet;
  packet.deviceId = NODE_ID;
  packet.azimuth = az;
  packet.elevation = el;
  packet.mode = mode;
  packet.counter = packetCounter++;
  
  radio.stopListening();
  
  bool success = radio.write(&packet, sizeof(packet));
  
  radio.startListening();
  
  if (success) {
    Serial.print(F("[TX #"));
    Serial.print(packet.counter);
    Serial.print(F("] az="));
    Serial.print(az);
    Serial.print(F("° el="));
    Serial.print(el);
    Serial.print(F("° mode="));
    Serial.print(mode);
    Serial.print(F(" laser="));
    Serial.println(laserEnabled ? "ON" : "OFF");
  } else {
    Serial.println(F("[TX] FAILED!"));
  }
  
  delay(10);
}