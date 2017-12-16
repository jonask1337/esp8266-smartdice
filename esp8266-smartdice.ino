#include <WiFiManager.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <SoftwareSerial.h>
#include <ESP8266WebServer.h>
#include <Ticker.h>

#define LED1 D1
#define LED2 D2
#define LED3 D3
#define DELAY 100 // ms
#define PRESS_DURATION 1000 // ms
#define CONFIG_MODE_DURATION 300 // s
#define BUTTON D4
#define SCL D6
#define SDA D5

// MQTT server
const char* server = "test.mosquitto.org";

// MPU6050 variables
MPU6050 mpu;
int16_t ax, ay, az;
int16_t ax_offset, ay_offset, az_offset;

// WiFi
WiFiClient espClient;
// MQTT
PubSubClient client(espClient);
// Button
int buttonState = 0;

int motionless_count = 0;
int motion_count = 0;

Ticker ticker;

int modus = 0;
int press_cycle_count = 0;

/* FUNCTIONS */
void calibrate_acc() {
  int i = 0;
  int i2 = 0;

  int16_t ax_offset_ar[10];
  int16_t offset = 0;
  for (i; i < 10 ; i++) {
    delay(1000);
    mpu.getAcceleration(&ax, &ay, &az);
    // Serial.println(ax);
    ax_offset_ar[i] = ax;
  }

  for (i2; i2 < 10; i2++ ) {
    // Serial.println(ax_offset_ar[i2]);
    offset = offset + ax_offset_ar[i2];
  }
  // Serial.print("final ax ");
  // Serial.println(offset);
  ax_offset = offset / 10.0;
}

void detect_throw() {
  if (mpu.getMotionStatus() == 0) {
    if (motionless_count > 4) {
      if (motion_count > 4) {
        Serial.println("Throw detected.");
        publish_values();
      }
      motion_count = 0;
    }
    motionless_count++;
  } else {
    motionless_count = 0;
    motion_count++;
  }
}

void plot_gyro() {
  Serial.print(mpu.getRotationX());
  Serial.print(" ");
  Serial.print(mpu.getRotationZ());
  Serial.print(" ");
  Serial.println(mpu.getRotationY());
}

void plot_acc() {
  mpu.getAcceleration(&ax, &ay, &az);
  Serial.print((ax - 600) / 16384.0);
  Serial.print(" ");
  Serial.print((ay - 600) / 16384.0);
  Serial.print(" ");
  Serial.println((az) / 16384.0);
}

void turn_on_led(int led_name) {
  if (digitalRead(led_name) == LOW) {
    digitalWrite(led_name, HIGH);
  }
}

void turn_off_led(int led_name) {
  if (digitalRead(led_name) == HIGH) {
    digitalWrite(led_name, LOW);
  }
}

void toggle_all_led() {
  if (digitalRead(LED1) == HIGH) {
    turn_off_led(LED1);
    turn_off_led(LED2);
    turn_off_led(LED3);
  } else {
    turn_on_led(LED1);
    turn_on_led(LED2);
    turn_on_led(LED3);
  }

}

void change_mode() {
  switch (modus) {
    case 0:
      turn_on_led(LED1);
      turn_off_led(LED2);
      turn_off_led(LED3);
      modus = 1;
      break;
    case 1:
      turn_off_led(LED1);
      turn_on_led(LED2);
      turn_off_led(LED3);
      modus = 2;
      break;
    case 2:
      turn_on_led(LED1);
      turn_on_led(LED2);
      turn_off_led(LED3);
      modus = 3;
      break;
    case 3:
      turn_off_led(LED1);
      turn_off_led(LED2);
      turn_on_led(LED3);
      modus = 4;
      break;
    case 4:
      turn_on_led(LED1);
      turn_off_led(LED2);
      turn_on_led(LED3);
      modus = 5;
      break;
    case 5:
      turn_off_led(LED1);
      turn_on_led(LED2);
      turn_on_led(LED3);
      modus = 6;
      break;
    case 6:
      turn_on_led(LED1);
      turn_on_led(LED2);
      turn_on_led(LED3);
      modus = 7;
      break;
    default:
      turn_off_led(LED1);
      turn_off_led(LED2);
      turn_off_led(LED3);
      modus = 0;
  }
}

void configModeCallback (WiFiManager *myManager) {
  ticker.attach(0.2, toggle_all_led);
}

void publish_values() {
  // Try to connect to the MQTT Broker
  if (client.connect("ESP8266Client")) {
    Serial.println("Connected to mqtt broker");
    // Once connected, publish an announcement...
    String msg_buf = "leeroy: " + String(modus);
    char msg[20];
    msg_buf.toCharArray(msg, 20);
    client.publish("leeroy", msg);
    Serial.println("Message published");
    //delay(300);
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
  }
}

/* SETUP */
void setup() {
  // join i2c bus
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(SDA, SCL);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // set baud rate
  Serial.begin(74880);
  delay(10);

  Serial.println("Set pinMode for LEDs.");
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  // initialize MPU6050 sensor platform
  Serial.println("Initializing MPU6050…");
  mpu.initialize();

  // verify connection
  Serial.println("Testing device connections…");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // enable motion detection
  mpu.setIntMotionEnabled(true);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(1);
  Serial.println(mpu.getMotionDetectionThreshold());
  Serial.println(mpu.getMotionDetectionDuration());

  // Set mqtt server
  client.setServer(server, 1883);
}

/* MAIN CODE */
void loop() {
  // Check if button was is pressed
  buttonState = digitalRead(BUTTON);
  if (buttonState == 0) {
    press_cycle_count++;
    if (press_cycle_count == PRESS_DURATION / DELAY) {
      // Start configmode
      WiFiManager wifiManager;
      // Set callback to toggle the leds
      wifiManager.setAPCallback(configModeCallback);

      if (!wifiManager.startConfigPortal("SmartDice")) {
        Serial.println("Failed to connect..");
        delay(3000);
        ESP.reset();
        delay(5000);
      }
      Serial.println("Successfully connected to wifi");
      ticker.detach();
    }
  } else {
    if (press_cycle_count > 1 && press_cycle_count < PRESS_DURATION / DELAY) {
      change_mode();
    }
    press_cycle_count = 0;
  }
  detect_throw();
  delay(DELAY);
}

