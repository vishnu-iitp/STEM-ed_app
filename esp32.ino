#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ESP32Servo.h>
#include <driver/ledc.h>

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

// --- LEDC Channels ---
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_SPEED_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_R1         LEDC_CHANNEL_0
#define LEDC_CHANNEL_R2         LEDC_CHANNEL_1
#define LEDC_CHANNEL_L1         LEDC_CHANNEL_2
#define LEDC_CHANNEL_L2         LEDC_CHANNEL_3

// --- State Flags for Gamepad Commands ---
bool up_pressed = false;
bool down_pressed = false;
bool left_pressed = false;
bool right_pressed = false;

// --- State-Holding Variables for Accelerometer ---
int currentSpeed = 0;
int currentTurn = 0;

// --- Dead Zone Thresholds ---
const int SPEED_DEAD_ZONE = 120;
const int TURN_DEAD_ZONE = 120;

// --- Obstacle Avoidance ---
bool obstacleMode = false;
const int OBSTACLE_THRESHOLD = 25;
const unsigned long TURN_DURATION = 600;

// --- Connection Handling ---
volatile bool isConnected = false;

// Forward declarations
void processCommand(String cmd);
void rotateMotor(int rightSpeed, int leftSpeed);
void runAccelerometerControl();


// --- BLE Callback Classes ---

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            value.trim();
            Serial.print("Received: "); 
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
      // Reset accelerometer state on disconnect
      currentSpeed = 0;
      currentTurn = 0;
      rotateMotor(0, 0);
      delay(500); 
      BLEDevice::startAdvertising();
      Serial.println("Advertising restarted.");
    }
};


// --- Core Functions ---

void setUpHardware() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  ledc_timer_config_t ledc_timer = {
      .speed_mode       = LEDC_SPEED_MODE,
      .duty_resolution  = PWMResolution,
      .timer_num        = LEDC_TIMER,
      .freq_hz          = PWMFreq,
      .clk_cfg          = LEDC_AUTO_CLK
  };
  ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

  ledc_channel_config_t r1_conf = { .gpio_num=rightMotorPin1, .speed_mode=LEDC_SPEED_MODE, .channel=LEDC_CHANNEL_R1, .timer_sel=LEDC_TIMER, .duty=0 };
  ledc_channel_config(&r1_conf);
  ledc_channel_config_t r2_conf = { .gpio_num=rightMotorPin2, .speed_mode=LEDC_SPEED_MODE, .channel=LEDC_CHANNEL_R2, .timer_sel=LEDC_TIMER, .duty=0 };
  ledc_channel_config(&r2_conf);
  ledc_channel_config_t l1_conf = { .gpio_num=leftMotorPin1, .speed_mode=LEDC_SPEED_MODE, .channel=LEDC_CHANNEL_L1, .timer_sel=LEDC_TIMER, .duty=0 };
  ledc_channel_config(&l1_conf);
  ledc_channel_config_t l2_conf = { .gpio_num=leftMotorPin2, .speed_mode=LEDC_SPEED_MODE, .channel=LEDC_CHANNEL_L2, .timer_sel=LEDC_TIMER, .duty=0 };
  ledc_channel_config(&l2_conf);

  servo.attach(servoPin, 500, 2500);
  servo.setPeriodHertz(50);
  servo.write(90);

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


// --- Accelerometer Logic now sets state variables ---
void handleAccelerometerControl(int xPwm, int yPwm) {
    if (obstacleMode) return;

    // Apply dead zone for forward/backward speed
    if (abs(yPwm) > SPEED_DEAD_ZONE) {
        currentSpeed = yPwm;
    } else {
        currentSpeed = 0;
    }

    // Apply dead zone for turning
    if (abs(xPwm) > TURN_DEAD_ZONE) {
        currentTurn = xPwm;
    } else {
        currentTurn = 0;
    }
}


// --- Central Command Processor ---
void processCommand(String cmd) {
    if (cmd.indexOf(',') != -1) {
        int commaIndex = cmd.indexOf(',');
        String xValStr = cmd.substring(0, commaIndex);
        String yValStr = cmd.substring(commaIndex + 1);
        
        handleAccelerometerControl(xValStr.toInt(), yValStr.toInt());
    } 
    else {
        // When a gamepad button is released, reset the accelerometer state to 0.
        if (cmd == "U_RELEASED" || cmd == "D_RELEASED" || cmd == "L_RELEASED" || cmd == "R_RELEASED") {
            currentSpeed = 0;
            currentTurn = 0;
        }

        if (cmd == "U_PRESSED") up_pressed = true;
        else if (cmd == "U_RELEASED") up_pressed = false;
        else if (cmd == "D_PRESSED") down_pressed = true;
        else if (cmd == "D_RELEASED") down_pressed = false;
        else if (cmd == "L_PRESSED") left_pressed = true;
        else if (cmd == "L_RELEASED") left_pressed = false;
        else if (cmd == "R_PRESSED") right_pressed = true;
        else if (cmd == "R_RELEASED") right_pressed = false;
        else if (cmd == "START") { 
            obstacleMode = true; 
            Serial.println("Obstacle Avoidance Mode: ON"); 
            up_pressed = down_pressed = left_pressed = right_pressed = false;
            currentSpeed = 0;
            currentTurn = 0;
            rotateMotor(0,0);
        }
        else if (cmd == "SELECT") { 
            obstacleMode = false; 
            Serial.println("Manual Control Mode: ON"); 
            rotateMotor(0,0);
        }
    }
}

// --- Gamepad Control Logic ---
void runManualControl() {
  int rightSpeed = 0, leftSpeed = 0;
  if (up_pressed) { rightSpeed = MAX_MOTOR_SPEED; leftSpeed = MAX_MOTOR_SPEED; } 
  else if (down_pressed) { rightSpeed = -MAX_MOTOR_SPEED; leftSpeed = -MAX_MOTOR_SPEED; } 
  else if (left_pressed) { rightSpeed = MAX_MOTOR_SPEED; leftSpeed = -MAX_MOTOR_SPEED; } 
  else if (right_pressed) { rightSpeed = -MAX_MOTOR_SPEED; leftSpeed = MAX_MOTOR_SPEED; }
  rotateMotor(rightSpeed, leftSpeed);
}

// --- Persistent Accelerometer Control Logic ---
void runAccelerometerControl() {
    // *** FIX: Inverting the 'turn' component to correct the direction ***
    float rawLeft = currentSpeed - currentTurn;
    float rawRight = currentSpeed + currentTurn;

    float maxMagnitude = max(abs(rawLeft), abs(rawRight));

    float scale = 1.0;
    if (maxMagnitude > MAX_MOTOR_SPEED) {
        scale = MAX_MOTOR_SPEED / maxMagnitude;
    }

    int leftMotorSpeed = (int)(rawLeft * scale);
    int rightMotorSpeed = (int)(rawRight * scale);

    rotateMotor(rightMotorSpeed, leftMotorSpeed);
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

// --- UPDATED Main Loop with Control Priority ---
void loop() {
  if (isConnected) {
    if (obstacleMode) {
      runObstacleAvoidance();
    } else {
      // Check if any gamepad button is being held down
      if (up_pressed || down_pressed || left_pressed || right_pressed) {
        runManualControl(); // Gamepad has priority
      } else {
        runAccelerometerControl(); // Otherwise, use the persistent accelerometer state
      }
    }
  }
  
  delay(20);
}
