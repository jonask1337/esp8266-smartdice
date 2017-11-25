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
String ssid     = ""; 
String password = "";
// MQTT server
const char* server = "mqtt_server_name";

ESP8266WebServer web_server(80);

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

void detect_throw(){
  if (mpu.getMotionStatus() == 0){
    if(motionless_count > 4){
      if(motion_count > 4){
        throw_detected();
      }
      motion_count = 0;  
    }
    motionless_count++;   
   } else {
    motionless_count = 0;
    motion_count++;   
   }
}

void throw_detected(){
  Serial.print("Throw detected");
}

void plot_gyro(){
  Serial.print(mpu.getRotationX());
  Serial.print(" ");
  Serial.print(mpu.getRotationZ());
  Serial.print(" ");
  Serial.println(mpu.getRotationY());
}

void plot_acc(){
  mpu.getAcceleration(&ax, &ay, &az);
  Serial.print((ax-600)/16384.0);
  Serial.print(" ");
  Serial.print((ay-600)/16384.0);
  Serial.print(" ");
  Serial.println((az)/16384.0);   
}

/*
 * Handle the configuration http request and set wifi credentials
 */
void handleConfig() {
  if (web_server.arg("ssid").length() > 0 && web_server.arg("pwd").length() > 0) {
    ssid = web_server.arg("ssid");
    password = web_server.arg("pwd");
  }
   
  String response = "<!DOCTYPE html><html><head><title>SmartDice Konfiguration</title>"
   "<meta name='viewport' content='width=device-width, initial-scale=1.0'></head>"
   "<body><h2>SmartDice Konfiguration</h2>"
   "Verbinde den SmartDice mit einem bestehenden WLAN Netzwerk.<br><br>"
   "<form method='post' action='/'><table>"
   "<tr><td>SSID:</td><td><input type='text' name='ssid' value='";
   response += ssid;
   response += "'></td></tr>"
   "<tr><td>Passwort:</td><td><input type='password' name='pwd'></td></tr>"
   "<tr><td></td><td><button style='width:100%;' type='submit'>SmartDice verbinden</button></td></tr>"
   "</table></form></body></html>";
   
   web_server.send(200, "text/html", response);
   Serial.println(ssid);
   Serial.println(password);
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

  // enable motion detection
  mpu.setIntMotionEnabled(true);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(1);
  Serial.println(mpu.getMotionDetectionThreshold());
  Serial.println(mpu.getMotionDetectionDuration());
  
  // calibrate acceleration sensor
  Serial.println("Calibrate acceleration sensor");
  calibrate_acc();
  Serial.print("ax_offset: ");
  Serial.println(ax_offset);

  // Enable AP and Server for configuration
  WiFi.softAP("SmartDice", "smartdice");
  web_server.on("/", handleConfig);
  web_server.begin();

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
   web_server.handleClient();
   detect_throw(); 
   plot_acc();
   //Serial.println(mpu.getMotionStatus());
   delay(250);  
}

