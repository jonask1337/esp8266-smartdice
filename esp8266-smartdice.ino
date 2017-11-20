#include "I2Cdev.h"
#include "MPU6050.h"
#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <SoftwareSerial.h>

#define BUTTON D7
 
const int history_size = 10;
// WiFi config
const char* ssid     = "your_ssid"; 
const char* password = "your_pw";
// MQTT server
const char* server = "mqtt_server_name";

// MPU6050 variables
MPU6050 mpu;
int16_t ax, ay, az;
int16_t ax_offset, ay_offset, az_offset;
int16_t value_history[history_size][3] = {{0}};
int history_offset = 0;

// WiFi
WiFiClient espClient;
// MQTT
PubSubClient client(espClient);
// Button
int buttonState = 0;

/* FUNCTIONS */
void calibrate_acc(){
    int i = 0;
    int i2 = 0;
    
    int16_t ax_offset_ar[10];
    int16_t offset = 0;
    for(i; i < 10 ; i++){
      delay(1000);
      mpu.getAcceleration(&ax, &ay, &az);
      // Serial.println(ax);
      ax_offset_ar[i] = ax;
    }

    for(i2; i2 < 10; i2++ ){
      // Serial.println(ax_offset_ar[i2]);
      offset = offset + ax_offset_ar[i2];
    }
    // Serial.print("final ax ");
    // Serial.println(offset);
    ax_offset = offset / 10.0;
}

void write_value_to_history(int16_t ax, int16_t ay, int16_t az ){
   value_history[history_offset][0] = ax;
   value_history[history_offset][1] = ay;
   value_history[history_offset][2] = az;
   history_offset = (history_offset + 1) % history_size;   
}

void detect_throw(){
  // collect data
  mpu.getAcceleration(&ax, &ay, &az);
  write_value_to_history(ax, ay, az);
  // TODO: calculate variance
  
  // TODO: check thresholds
}

void plot_acc()
{
  mpu.getAcceleration(&ax, &ay, &az);
  Serial.print((ax-600)/16384.0);
  Serial.print(" ");
  Serial.print((ay-600)/16384.0);
  Serial.print(" ");
  Serial.println((az)/16384.0); 
}

/* SETUP */
void setup() {
  // join i2c bus
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(D4, D5);
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
  #endif

  // set baud rate
  Serial.begin(74880);
  delay(10);

  // initialize MPU6050 sensor platform
  Serial.println("Initializing MPU6050…");
  mpu.initialize();

  // verify connection
  Serial.println("Testing device connections…");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // calibrate acceleration sensor
  Serial.println("Calibrate acceleration sensor");
  calibrate_acc();
  Serial.print("ax_offset: ");
  Serial.println(ax_offset);

  /*
  pinMode(BUTTON, INPUT);
  // Setup WIFI Connection
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
    
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.setServer(server, 1883);
*/
}

/* MAIN CODE */
void loop() {
  // put your main code here, to run repeatedly:
  /*
  // Check if the button was pressed and
  // if this is the case send a msg to the mqtt server
  buttonState = digitalRead(BUTTON);
  if(buttonState == HIGH){
    if (client.connect("ESP8266Client")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("leeroy", "hello");
      delay(300);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
   }*/
   plot_acc();
   delay(250);  
}

