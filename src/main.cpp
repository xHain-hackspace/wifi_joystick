#include <Arduino.h>
#include <Wire.h>
#include <RunningMedian.h>
#include <I2Cdev.h>
#include <MPU6050.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#define UPDATE_SEND_DELAY 5
#define JOYSTICK_EVENT_SIZE_BYTES 4
#define DEBUG_PRINT 0

// JOYSTICK
#define JOYSTICK_ORIENTATION 1     // 0, 1 or 2 to set the angle of the joystick
#define JOYSTICK_DIRECTION   1     // 0/1 to flip joystick direction
#define JOYSTICK_DEADZONE    5     // Angle to ignore
int8_t joystickTilt = 0;              // Stores the angle of the joystick
int16_t joystickWobble = 0;            // Stores the max amount of acceleration (wobble)

// MPU
#define SCL_PIN 22
#define SDA_PIN 23
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;
RunningMedian MPUAngleSamples = RunningMedian(5);
RunningMedian MPUWobbleSamples = RunningMedian(5);

//UDP/Wifi
/* WiFi network name and password */
const char * ssid = "ubirch";//"Camp2019-insecure";
const char * pwd = "H3ll0Ub1rch!";
// IP address to send UDP data to.
const char * udpAddress = "192.168.8.155";
const int udpPort = 1337;
//create UDP instance
WiFiUDP udp;

void getInput(){
    
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    int a = (JOYSTICK_ORIENTATION == 0?ax:(JOYSTICK_ORIENTATION == 1?ay:az))/166;
    int g = (JOYSTICK_ORIENTATION == 0?gx:(JOYSTICK_ORIENTATION == 1?gy:gz));
    if(abs(a) < JOYSTICK_DEADZONE) a = 0;
    if(a > 0) a -= JOYSTICK_DEADZONE;
    if(a < 0) a += JOYSTICK_DEADZONE;
    MPUAngleSamples.add(a);
    MPUWobbleSamples.add(g);
    
    joystickTilt = MPUAngleSamples.getMedian();
    if(JOYSTICK_DIRECTION == 1) {
        joystickTilt = 0-joystickTilt;
    }
    joystickWobble = abs(MPUWobbleSamples.getHighest());
}

void reconnect_wifi(){
  Serial.println("WiFi not connected. Trying to reconnect");
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pwd);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void setup() {
  //LED
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN,LOW);
  delay(200);
  digitalWrite(LED_BUILTIN,HIGH);
  delay(200);
  digitalWrite(LED_BUILTIN,LOW);

  //Serial
  Serial.begin(9600);
  while (!Serial)
  {
    // do nothing
  }
  Serial.println("Initialzing I2C and MPU6050");
  // MPU
  Wire.begin(SDA_PIN,SCL_PIN);
  accelgyro.initialize();

  Serial.println("Connecting to WiFi");
  reconnect_wifi();
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  //This initializes udp and transfer buffer
  udp.begin(udpPort);
}

//Send Event: bytes: 0,105,til, wobble
void send_joystick_event_udp(int8_t tilt, uint8_t wobble){
  //assemble data buffer
  uint8_t buffer[JOYSTICK_EVENT_SIZE_BYTES];
  buffer[0] = 0;
  buffer[1] = 105;
  buffer[2] = tilt;
  buffer[3] = wobble;
  if (DEBUG_PRINT) Serial.printf("Sending UDP: tilt %d, wobble: %d\n",tilt,wobble);
  //send data
  udp.beginPacket(udpAddress, udpPort);
  udp.write(buffer, JOYSTICK_EVENT_SIZE_BYTES);
  udp.endPacket();
}

void loop() {
  //get joystick data
  getInput();
  if (DEBUG_PRINT) Serial.printf("joystickTilt: %d, joystickWobble: %d\n",joystickTilt,joystickWobble);
  
  //check wifi, send udp or reconnect wifi accordingly, flash led
  if(WiFi.status() == WL_CONNECTED){
    digitalWrite(LED_BUILTIN,HIGH);//indicate connection
    send_joystick_event_udp(joystickTilt,joystickWobble/(32767/255));
  }else{
    digitalWrite(LED_BUILTIN,LOW);//indicate disconnect
    reconnect_wifi();
  }
  delay(UPDATE_SEND_DELAY);
}

