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

#define DICE_ID 192356235
#define LED1 D1
#define LED2 D2
#define LED3 D3
#define DELAY 100 // ms
#define PRESS_DURATION 1000 // ms
#define CONFIG_MODE_DURATION 300 // s
#define BUTTON D4
#define SCL D6
#define SDA D5
#define CLIENT_ID "ESP8266Client1"

String mode_names[] = {"D6", "D12", "D20", "user"};

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
int modus = 0;
int press_cycle_count = 0;
// Motion detection
int motionless_count = 0;
int motion_count = 0;
// Battery Level
unsigned int raw = 0;
float volt = 0.0;

Ticker ticker;
/* FUNCTIONS */
void calibrate_acc() {
  int i = 0;
  int i2 = 0;

  int16_t ax_offset_ar[10];
  int16_t offset = 0;
  for (i; i < 10 ; i++) {
    delay(1000);
    mpu.getAcceleration(&ax, &ay, &az);
    ax_offset_ar[i] = ax;
  }

  for (i2; i2 < 10; i2++ ) {
    offset = offset + ax_offset_ar[i2];
  }
  ax_offset = offset / 10.0;
}

// Checks if a throw event occured if this is the case
// a message will be send to the /roll topic of the MQTT Broker.
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

// Prints out the values of each axis of the gyroscope
void plot_gyro() {
  Serial.print(mpu.getRotationX());
  Serial.print(" ");
  Serial.print(mpu.getRotationZ());
  Serial.print(" ");
  Serial.println(mpu.getRotationY());
}

// Prints out the values of each axis of the accelerometer
void plot_acc() {
  mpu.getAcceleration(&ax, &ay, &az);
  Serial.print((ax - 600) / 16384.0);
  Serial.print(" ");
  Serial.print((ay - 600) / 16384.0);
  Serial.print(" ");
  Serial.println((az) / 16384.0);
}

// Sets the pin with the given name to HIGH
void turn_on_led(int led_name) {
  if (digitalRead(led_name) == LOW) {
    digitalWrite(led_name, HIGH);
  }
}

// Sets the pin with the given name to LOW
void turn_off_led(int led_name) {
  if (digitalRead(led_name) == HIGH) {
    digitalWrite(led_name, LOW);
  }
}

// Depending on the current state it either sets the
// LED pins to HIGH or to LOW
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

// Depending on the current modus the LEDs will be turned off or on
void update_leds() {
  switch (modus) {
    case 0:
      turn_on_led(LED1);
      turn_off_led(LED2);
      turn_off_led(LED3);
      break;
    case 1:
      turn_off_led(LED1);
      turn_on_led(LED2);
      turn_off_led(LED3);
      break;
    case 2:
      turn_on_led(LED1);
      turn_on_led(LED2);
      turn_off_led(LED3);
      break;
    case 3:
      turn_off_led(LED1);
      turn_off_led(LED2);
      turn_on_led(LED3);
      break;
    case 4:
      turn_on_led(LED1);
      turn_off_led(LED2);
      turn_on_led(LED3);
      break;
    case 5:
      turn_off_led(LED1);
      turn_on_led(LED2);
      turn_on_led(LED3);
      break;
    case 6:
      turn_on_led(LED1);
      turn_on_led(LED2);
      turn_on_led(LED3);
      break;
    default:
      turn_off_led(LED1);
      turn_off_led(LED2);
      turn_off_led(LED3);
  }
}

// Switches to the next available modus. If the last modus is currently selected
// it changes to the first again. Updates the LEDs accordingly and sends the
// information that the modus was changed to the MQTT Broker.
void change_mode() {
  modus = (modus + 1) % (sizeof(mode_names) / sizeof(String));
  Serial.println(modus);
  update_leds();
  set_mode(mode_names[modus]);
}

// Callback function that toggles all LEDs when the Dice is in config mode.
void configModeCallback (WiFiManager *myManager) {
  ticker.attach(0.2, toggle_all_led);
}

// Sends some values to the smartdice/<dice_id>/roll topic.
void publish_values() {
  // Try to connect to the MQTT Broker
  if (!client.connected()) {
    client.connect(CLIENT_ID);
  }
  Serial.println("Connected to mqtt broker");
  String msg_buf = "leeroy: " + String(modus);
  char msg[20];
  String topic_buf = "smartdice/" + String(DICE_ID) + "/roll";
  char topic[50];
  topic_buf.toCharArray(topic, 50);
  msg_buf.toCharArray(msg, 20);
  client.publish(topic, msg);
  Serial.print("Message published");
}

// Sends the current mode of the dice to the smartdice/<dice_id>/setmode
// topic. Also publishes the battery level value to the smartdice/<dice_id>/battery topic
void set_mode(String dice_mode) {
  // Try to connect to the MQTT Broker
  if (!client.connected()) {
    client.connect(CLIENT_ID);
  }
  // Publish the dice mode
  String msg_buf = dice_mode;
  char msg[20];
  String topic_buf = "smartdice/" + String(DICE_ID) + "/setmode";
  char topic[50];
  topic_buf.toCharArray(topic, 50);
  msg_buf.toCharArray(msg, 20);
  client.publish(topic, msg);
  // Publish the battery level
  int level = get_level();
  String battery_buff = String(level);
  char battery[50];
  battery_buff.toCharArray(battery, 50);
  client.publish("smartdice/192356235/battery", battery);
  Serial.print("Message published");
}

// Callback for receiving messages from the subscribed /getmode topic.
// If the received message contains a valid mode name the dice modus
// will be changed and the LEDs will be updated.
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  // Convert the payload bytearray to a String
  String mode_string = String((char*)payload).substring(0, length);
  // Check if the received String equals to one of the mode names.
  int array_length = sizeof(mode_names) / sizeof(String);
  for (int i = 0; i < array_length; i++ ) {
    // If the reveived string is an valid mode name the
    // modus will be changed and the LEDs updated.
    if (mode_string.equals(mode_names[i])) {
      modus = i;
      update_leds();
    }
  }
  Serial.println(mode_string);
}

// Connects to the MQTT Broker sets the callback to be able to
// receive and handle messages from subscribed topics. And subcribes
// to the /getmode topic.
void connect_and_subscribe() {
  if (!client.connected()) {
    if (client.connect(CLIENT_ID)) {
      Serial.println("Connected to mqtt broker in main loop");
      boolean sub_result;
      client.setCallback(callback);
      sub_result =  client.subscribe("smartdice/192356235/getmode");
      Serial.println(sub_result);
    } else {
      Serial.print("failed to subscribe, rc=");
      Serial.println(client.state());
    }
  }
}

// If the battery is connected to the analog pin the current voltage
// is read and mapped to a number between 0 and 100 based on the voltage
// value when the battery is fully charged and when the battery is empty.
int get_level() {
  raw = analogRead(A0);
  volt = raw / 1023.0;
  volt = volt * 4.2;
  Serial.print("Voltage: ");
  Serial.println(volt);
  int value = int(volt * 1000);
  int level = map(value, 3000, 3900, 0, 100);
  if (level < 0) {
    level = 0;
  }
  if (level > 100) {
    level = 100;
  }
  Serial.println(level);
  return level;
}

/* SETUP */
void setup() {
  // Join i2c bus
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(SDA, SCL);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // Set baud rate
  Serial.begin(74880);
  delay(10);

  Serial.println("Set pinMode for LEDs.");
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);

  // Set pin mode for the analog pin
  pinMode(A0, INPUT);

  // Initialize MPU6050 sensor platform
  Serial.println("Initializing MPU6050…");
  mpu.initialize();

  // Verify connection to the MPU6050
  Serial.println("Testing device connections…");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // Enable motion detection
  mpu.setIntMotionEnabled(true);
  mpu.setMotionDetectionThreshold(2);
  mpu.setMotionDetectionDuration(1);
  Serial.println(mpu.getMotionDetectionThreshold());
  Serial.println(mpu.getMotionDetectionDuration());

  // Set mqtt server
  client.setServer(server, 1883);
  update_leds();
  connect_and_subscribe();
}

/* MAIN CODE */
void loop() {
  if (client.connected()) {
    client.loop();
  }
  // Check if button was is pressed
  buttonState = digitalRead(BUTTON);
  if (buttonState == 0) {
    press_cycle_count++;
    if (press_cycle_count == PRESS_DURATION / DELAY) {
      // Start configmode
      WiFiManager wifiManager;
      // Set callback to toggle the leds
      wifiManager.setAPCallback(configModeCallback);
      WiFi.disconnect(true);
      if (!wifiManager.startConfigPortal("SmartDice")) {
        Serial.println("Wifi Manager Failed to connect...");
        delay(3000);
        ESP.reset();
        delay(5000);
      }
      // Update LEDs
      Serial.println("Successfully connected to wifi");
      ticker.detach();
      update_leds();
    }
  } else {
    if (press_cycle_count > 1 && press_cycle_count < PRESS_DURATION / DELAY) {
      if (client.connected()) {
        change_mode();
      } else {
        Serial.print("Connection attempt!");
        // Flash led
        toggle_all_led();
        delay(50);
        toggle_all_led();
        delay(50);
        update_leds();
        connect_and_subscribe();
      }
    } else {
      detect_throw();
    }
    press_cycle_count = 0;
  }
  delay(DELAY);
}

