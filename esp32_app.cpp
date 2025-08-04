#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ESP32Servo.h>
#include <driver/ledc.h> // Required for the low-level LEDC driver API

// --- BLE Configuration ---
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

// --- Global BLE Server Pointer ---
BLEServer *pServer = NULL;

// --- Pin Definitions ---
int rightMotorPin1 = 14, rightMotorPin2 = 27;
int leftMotorPin1  = 26, leftMotorPin2  = 25;
const int trigPin = 5;
const int echoPin = 18;
const int servoPin = 17;
Servo servo;

// --- Motor & Control Parameters ---
#define MAX_MOTOR_SPEED 255
const int PWMFreq = 1000;
const ledc_timer_bit_t PWMResolution = LEDC_TIMER_8_BIT;

// --- LEDC Channels for ESP32 Core v3.0.0+ ---
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_SPEED_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_R1         LEDC_CHANNEL_0
#define LEDC_CHANNEL_R2         LEDC_CHANNEL_1
#define LEDC_CHANNEL_L1         LEDC_CHANNEL_2
#define LEDC_CHANNEL_L2         LEDC_CHANNEL_3

// --- State Flags for App Commands ---
bool up_pressed = false;
bool down_pressed = false;
bool left_pressed = false;
bool right_pressed = false;

// --- Obstacle Avoidance ---
bool obstacleMode = false;
const int OBSTACLE_THRESHOLD = 25; // cm
const unsigned long TURN_DURATION = 600; // ms

// --- Connection Handling ---
volatile bool isConnected = false; // Use volatile for variables shared with callbacks

// Forward declarations for functions
void processCommand(String cmd);
void rotateMotor(int rightSpeed, int leftSpeed);

// --- BLE Callback Classes ---

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            value.trim();
            Serial.print("Received Command: ");
            Serial.println(value);
            processCommand(value);
        }
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      isConnected = true;
      Serial.println("Connection established.");
    }

    void onDisconnect(BLEServer* pServer) {
      isConnected = false;
      Serial.println("Connection lost. Stopping motors.");
      up_pressed = false;
      down_pressed = false;
      left_pressed = false;
      right_pressed = false;
      rotateMotor(0, 0);
      delay(500); 
      BLEDevice::startAdvertising();
      Serial.println("Advertising restarted.");
    }
};


// --- Core Functions ---

void setUpHardware() {
  // Configure Ultrasonic Sensor Pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Configure LEDC Timer for motors
  ledc_timer_config_t ledc_timer = {
      .speed_mode       = LEDC_SPEED_MODE,
      .duty_resolution  = PWMResolution,
      .timer_num        = LEDC_TIMER,
      .freq_hz          = PWMFreq,
      .clk_cfg          = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  // Configure LEDC Channels for each motor pin
  ledc_channel_config_t ledc_channel_r1 = { .gpio_num = rightMotorPin1, .speed_mode = LEDC_SPEED_MODE, .channel = LEDC_CHANNEL_R1, .timer_sel = LEDC_TIMER, .duty = 0 };
  ledc_channel_config(&ledc_channel_r1);

  ledc_channel_config_t ledc_channel_r2 = { .gpio_num = rightMotorPin2, .speed_mode = LEDC_SPEED_MODE, .channel = LEDC_CHANNEL_R2, .timer_sel = LEDC_TIMER, .duty = 0 };
  ledc_channel_config(&ledc_channel_r2);

  ledc_channel_config_t ledc_channel_l1 = { .gpio_num = leftMotorPin1, .speed_mode = LEDC_SPEED_MODE, .channel = LEDC_CHANNEL_L1, .timer_sel = LEDC_TIMER, .duty = 0 };
  ledc_channel_config(&ledc_channel_l1);
  
  ledc_channel_config_t ledc_channel_l2 = { .gpio_num = leftMotorPin2, .speed_mode = LEDC_SPEED_MODE, .channel = LEDC_CHANNEL_L2, .timer_sel = LEDC_TIMER, .duty = 0 };
  ledc_channel_config(&ledc_channel_l2);

  // Configure Servo
  servo.attach(servoPin, 500, 2500);
  servo.setPeriodHertz(50);
  servo.write(90);

  // Ensure motors are stopped initially
  rotateMotor(0, 0);
}

void setDuty(ledc_channel_t channel, uint32_t duty) {
    ledc_set_duty(LEDC_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_SPEED_MODE, channel);
}

void rotateMotor(int rightSpeed, int leftSpeed) {
  rightSpeed = constrain(rightSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  leftSpeed = constrain(leftSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

  if (rightSpeed > 0) { setDuty(LEDC_CHANNEL_R1, rightSpeed); setDuty(LEDC_CHANNEL_R2, 0); } 
  else if (rightSpeed < 0) { setDuty(LEDC_CHANNEL_R1, 0); setDuty(LEDC_CHANNEL_R2, abs(rightSpeed)); } 
  else { setDuty(LEDC_CHANNEL_R1, 0); setDuty(LEDC_CHANNEL_R2, 0); }

  if (leftSpeed > 0) { setDuty(LEDC_CHANNEL_L1, leftSpeed); setDuty(LEDC_CHANNEL_L2, 0); } 
  else if (leftSpeed < 0) { setDuty(LEDC_CHANNEL_L1, 0); setDuty(LEDC_CHANNEL_L2, abs(leftSpeed)); } 
  else { setDuty(LEDC_CHANNEL_L1, 0); setDuty(LEDC_CHANNEL_L2, 0); }
}

void processCommand(String cmd) {
    if (cmd == "U_PRESSED") up_pressed = true;
    else if (cmd == "U_RELEASED") up_pressed = false;
    else if (cmd == "D_PRESSED") down_pressed = true;
    else if (cmd == "D_RELEASED") down_pressed = false;
    else if (cmd == "L_PRESSED") left_pressed = true;
    else if (cmd == "L_RELEASED") left_pressed = false;
    else if (cmd == "R_PRESSED") right_pressed = true;
    else if (cmd == "R_RELEASED") right_pressed = false;
    else if (cmd == "START") { obstacleMode = true; Serial.println("Obstacle Avoidance Mode: ON"); }
    else if (cmd == "SELECT") { obstacleMode = false; Serial.println("Manual Control Mode: ON"); }
}

void runManualControl() {
  int rightSpeed = 0, leftSpeed = 0;
  if (up_pressed) { rightSpeed = MAX_MOTOR_SPEED; leftSpeed = MAX_MOTOR_SPEED; } 
  else if (down_pressed) { rightSpeed = -MAX_MOTOR_SPEED; leftSpeed = -MAX_MOTOR_SPEED; } 
  else if (left_pressed) { rightSpeed = -MAX_MOTOR_SPEED; leftSpeed = MAX_MOTOR_SPEED; } 
  else if (right_pressed) { rightSpeed = MAX_MOTOR_SPEED; leftSpeed = -MAX_MOTOR_SPEED; }
  rotateMotor(rightSpeed, leftSpeed);
}

long measureDistanceCM() {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 30000);
  return (duration == 0) ? 999 : duration * 0.034 / 2;
}

void runObstacleAvoidance() {
  long distance = measureDistanceCM();
  if (distance > OBSTACLE_THRESHOLD) {
    servo.write(90);
    rotateMotor(MAX_MOTOR_SPEED / 2, MAX_MOTOR_SPEED / 2);
  } else {
    rotateMotor(0, 0); delay(100);
    rotateMotor(-MAX_MOTOR_SPEED / 2, -MAX_MOTOR_SPEED / 2); delay(400);
    rotateMotor(0, 0); delay(100);

    servo.write(30); delay(300); long distLeft = measureDistanceCM();
    servo.write(150); delay(300); long distRight = measureDistanceCM();
    servo.write(90); delay(300);

    if (distLeft > distRight && distLeft > OBSTACLE_THRESHOLD) { rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED); delay(TURN_DURATION); } 
    else if (distRight > distLeft && distRight > OBSTACLE_THRESHOLD) { rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED); delay(TURN_DURATION); } 
    else { rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED); delay(TURN_DURATION * 1.5); }
    rotateMotor(0, 0); delay(200);
  }
}

// --- Main Setup and Loop ---
void setup() {
  Serial.begin(115200);
  Serial.println("Starting BLE Car Controller...");

  BLEDevice::init("ESP32_Car");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
                                       );
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pService->start();

  delay(100);

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("BLE Server started. Now setting up hardware...");

  setUpHardware();
  
  Serial.println("Setup complete. Waiting for a client connection...");
}

void loop() {
  if (isConnected) {
    // Timeout logic has been removed.
    
    if (obstacleMode) {
      runObstacleAvoidance();
    } else {
      runManualControl();
    }
  }
  
  delay(10); 
}
