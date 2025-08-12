# STEMed Mobile Application & ESP32 Car Project

A versatile mobile application designed to control robotics and IoT devices over Bluetooth Low Energy (BLE), with a fully functional ESP32-based robotic car example.

---

## 1. Introduction

Welcome to **STEMed**, a mobile app that interfaces with and controls various hardware projects, with a primary focus on **robotics** and **IoT devices**. It connects to hardware via **Bluetooth Low Energy (BLE)** and provides multiple control interfaces to suit your project's needs.

This documentation covers:

- App features and screens
- BLE communication protocol
- Complete Arduino code for building your own **ESP32-based robotic car**

---

## 2. Core Features

- **Bluetooth LE Connectivity**: Scan, connect, and disconnect from BLE devices.
- **Multiple Control Interfaces**:
  - **Gamepad Mode**: Classic directional and action commands.
  - **Accelerometer Mode**: Tilt-based analog control.
- **Real-time Sensor Data**: View accelerometer, gyroscope, and magnetometer readings.
- **User-Friendly Interface**: Clean, intuitive navigation.

---

## 3. Screen-by-Screen Breakdown

### 3.1 Home Screen

- **Header**: App name, Bluetooth Device Scanner button, and Settings button.
- **Module Grid**: Access:
  - **Gamepad**: Opens Gamepad/Accelerometer control screen.
  - **Phone Sensors**: Opens real-time sensor data screen.
  - *(Other modules like LED Control, Motor Control, Camera, IoT are planned for future updates)*

### 3.2 Bluetooth Connection Modal

- **Scanning**: Auto-scans for 10 seconds.
- **Device List**: Shows discovered devices.
- **Actions**:
  - *Rescan*
  - *Disconnect*
  - *Close*

### 3.3 Gamepad Control Screen

Two modes:

1. **Gamepad Mode** (default)

   - **D-Pad Commands**:
     - `U_PRESSED` / `U_RELEASED`
     - `D_PRESSED` / `D_RELEASED`
     - `L_PRESSED` / `L_RELEASED`
     - `R_PRESSED` / `R_RELEASED`
   - **Action Buttons**:
     - △ → `T`
     - □ → `S`
     - ○ → `O`
     - × → `X`
     - `SELECT`, `START`

2. **Accelerometer Mode**

   - Sends `xPWM,yPWM` values based on tilt.
   - Ranges: `-255` to `255`.
   - Visual feedback joystick.
   - Action buttons work as in Gamepad Mode.

### 3.4 Phone Sensors Screen

- Displays live readings for:
  - Accelerometer (G-force)
  - Gyroscope (rad/s)
  - Magnetometer (μT)

### 3.5 Settings Screen

- Auto-connect toggle *(future)*
- Notifications toggle *(future)*

---

## 4. BLE Communication Protocol

To be compatible with STEMed, your device must:

- **Service UUID**: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
- **Characteristic UUID**: `beb5483e-36e1-4688-b7f5-ea07361b26a8`
- Accept string commands defined in Gamepad/Accelerometer sections.

---

# STEM-ed Car Kit Components

Below is a list of all the components included in your STEM-ed Car Kit. These parts are all available in the `assets` folder on GitHub.

| Image | Component | Quantity |
|-------|-----------|----------|
| ![ESP32 Board](assets/esp32.png) | ESP32 Development Board | 1 |
| ![Motor Driver](assets/drv8833.png) | L298N Motor Driver | 1 |
| ![DC Motor](assets/dc_motor.jpg) | DC Motors | 4 |
| ![Wheels](assets/dc_motor.jpg) | Wheels | 4 |
| ![Chassis](assets/chassis.jpg) | Acrylic Chassis | 1 |
| ![Battery Holder](assets/battery_holder.jpg) | 18650 Battery Holder | 1 |
| ![Batteries](assets/battery.png) | 18650 Batteries | 2 |
| ![Jumper Wires](assets/jumper_wire.webp) | Jumper Wires | 10 |
| ![Switch](assets/switch.jpg) | On/Off Switch | 1 |
| ![Servo](assets/servo.webp) | Servo motor | 1 |
| ![Ultra sounic Sensor](assets/sensor.jpg) | Sr04 sensor | 1 |
| ![3d Printed chassis](assets/3d_part.zip) | 3d printed chassis | 1 |
```

## 5. ESP32 Car Example

Below is a **fully working Arduino sketch** for an ESP32 robotic car compatible with STEMed.

```cpp
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ESP32Servo.h>
#include <driver/ledc.h>

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer *pServer = NULL;

int rightMotorPin1 = 14, rightMotorPin2 = 27;
int leftMotorPin1  = 26, leftMotorPin2  = 25;
const int trigPin = 5;
const int echoPin = 18;
const int servoPin = 17;
Servo servo;

#define MAX_MOTOR_SPEED 255
const int PWMFreq = 1000;
const ledc_timer_bit_t PWMResolution = LEDC_TIMER_8_BIT;

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_SPEED_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_R1         LEDC_CHANNEL_0
#define LEDC_CHANNEL_R2         LEDC_CHANNEL_1
#define LEDC_CHANNEL_L1         LEDC_CHANNEL_2
#define LEDC_CHANNEL_L2         LEDC_CHANNEL_3

bool up_pressed = false, down_pressed = false, left_pressed = false, right_pressed = false;
int currentSpeed = 0, currentTurn = 0;
const int SPEED_DEAD_ZONE = 120, TURN_DEAD_ZONE = 120;
bool obstacleMode = false;
const int OBSTACLE_THRESHOLD = 25;
const unsigned long TURN_DURATION = 600;
volatile bool isConnected = false;

void processCommand(String cmd);
void rotateMotor(int rightSpeed, int leftSpeed);
void runAccelerometerControl();

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue();
        if (value.length() > 0) {
            value.trim();
            processCommand(value);
        }
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { isConnected = true; }
    void onDisconnect(BLEServer* pServer) {
        isConnected = false;
        up_pressed = down_pressed = left_pressed = right_pressed = false;
        currentSpeed = currentTurn = 0;
        rotateMotor(0, 0);
        BLEDevice::startAdvertising();
    }
};

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

void handleAccelerometerControl(int xPwm, int yPwm) {
    if (obstacleMode) return;
    currentSpeed = (abs(yPwm) > SPEED_DEAD_ZONE) ? yPwm : 0;
    currentTurn  = (abs(xPwm) > TURN_DEAD_ZONE) ? xPwm : 0;
}

void processCommand(String cmd) {
    if (cmd.indexOf(',') != -1) {
        int commaIndex = cmd.indexOf(',');
        handleAccelerometerControl(cmd.substring(0, commaIndex).toInt(), cmd.substring(commaIndex + 1).toInt());
    } else {
        if (cmd == "U_PRESSED") up_pressed = true;
        else if (cmd == "U_RELEASED") up_pressed = false;
        else if (cmd == "D_PRESSED") down_pressed = true;
        else if (cmd == "D_RELEASED") down_pressed = false;
        else if (cmd == "L_PRESSED") left_pressed = true;
        else if (cmd == "L_RELEASED") left_pressed = false;
        else if (cmd == "R_PRESSED") right_pressed = true;
        else if (cmd == "R_RELEASED") right_pressed = false;
        else if (cmd == "START") { obstacleMode = true; up_pressed = down_pressed = left_pressed = right_pressed = false; currentSpeed = currentTurn = 0; rotateMotor(0,0); }
        else if (cmd == "SELECT") { obstacleMode = false; rotateMotor(0,0); }
    }
}

void runManualControl() {
    int rightSpeed = 0, leftSpeed = 0;
    if (up_pressed) { rightSpeed = MAX_MOTOR_SPEED; leftSpeed = MAX_MOTOR_SPEED; }
    else if (down_pressed) { rightSpeed = -MAX_MOTOR_SPEED; leftSpeed = -MAX_MOTOR_SPEED; }
    else if (left_pressed) { rightSpeed = MAX_MOTOR_SPEED; leftSpeed = -MAX_MOTOR_SPEED; }
    else if (right_pressed) { rightSpeed = -MAX_MOTOR_SPEED; leftSpeed = MAX_MOTOR_SPEED; }
    rotateMotor(rightSpeed, leftSpeed);
}

void runAccelerometerControl() {
    float rawLeft = currentSpeed - currentTurn;
    float rawRight = currentSpeed + currentTurn;
    float maxMagnitude = max(abs(rawLeft), abs(rawRight));
    float scale = (maxMagnitude > MAX_MOTOR_SPEED) ? (MAX_MOTOR_SPEED / maxMagnitude) : 1.0;
    rotateMotor((int)(rawRight * scale), (int)(rawLeft * scale));
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
        if (distLeft > distRight && distLeft > OBSTACLE_THRESHOLD) rotateMotor(-MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
        else if (distRight > distLeft && distRight > OBSTACLE_THRESHOLD) rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
        else rotateMotor(MAX_MOTOR_SPEED, -MAX_MOTOR_SPEED);
        delay(TURN_DURATION);
        rotateMotor(0, 0); delay(200);
    }
}

void setup() {
    Serial.begin(115200);
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
  
```
