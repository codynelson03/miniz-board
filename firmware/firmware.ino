#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#define PI 3.14159265

#include "network.h"
#include "packet.h"
#include "status_light.h"
#include "Adafruit_VL53L0X.h"

// TODO use SAMD timer interrupt to achieve higher control frequency

unsigned int localPort = 2390;
unsigned long last_packet_ts = 0;

// board pin layout
int encoder_s_pin = 14;

// NOTE this is inconsistent with schematic
// left
int steer_rev_pin = 2;
// right
int steer_fwd_pin = 3;

int drive_rev_pin = 5;
int drive_fwd_pin = 6;

// electronics calibration
// [-1,1]
float throttle = 0.0;
// left positive, radians
float steering = 0.0;
float throttle_deadzone = 0.05;

float full_left_angle_rad = 26.1/180.0*PI;
float full_right_angle_rad = -26.1/180.0*PI;
float full_left_encoder_value = 630.0;
float full_right_encoder_value = 410.0;
float steering_deadzone_rad = 1.0/180.0*PI;
unsigned long last_pid_ts = 0;
float last_err = 0.0;
float steering_integral = 0.0;
float steering_integral_limit = 1.0;

bool flag_failsafe = false;
StatusLed led;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;


// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  pinMode(encoder_s_pin, INPUT);
  pinMode(steer_rev_pin, OUTPUT);
  pinMode(steer_fwd_pin, OUTPUT);
  pinMode(drive_rev_pin, OUTPUT);
  pinMode(drive_fwd_pin, OUTPUT);
  led.init();
  led.off();
  setupWifi();
  Udp.begin(localPort);
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X"));
    while(1);
  }
}

void blinkTwice(){
  // LED blink to indicate we are ready for commands
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);
  delay(300);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(300);
  digitalWrite(LED_BUILTIN, LOW);

}

void setupWifi(){
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    while (true);
  }

  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  led.blink();
  
  printCurrentNet();
  printWifiData();
}

void loop() {
  led.update();
//  Serial.println(millis() - loop_time);
//  loop_time = millis();
  int packet_size = Udp.parsePacket();
  
  // process incoming packet
  if (packet_size) {
    if (packet_size != PACKET_SIZE){ Serial.println("err packet size"); }

    //Serial.print("Received packet of size ");
    //Serial.println(packetSize);
    //Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    int len = Udp.read(in_buffer, PACKET_SIZE);
    if (len != PACKET_SIZE){ Serial.println("err reading packet size"); }
    //Serial.println("parsing packet");
    parsePacket();
    last_packet_ts = millis();
    flag_failsafe = false;
    led.on();
    // response is handled by packet parser
  }

  if (millis() - last_packet_ts > 100 && !flag_failsafe){
    throttle = 0;
    steering = 0;
    flag_failsafe = true;
    //Serial.print(millis());
    //Serial.println(" failsafe");
    led.blink();
  } 

  actuateControls();   
      
  lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

  if (measure.RangeStatus != 4) {  // phase failures have incorrect data
    Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" out of range ");
  }
    
  //delay(5);
}


float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// using global variable throttle and steering
// set pwm for servo and throttle
void actuateControls(){
  if (flag_failsafe){
    // brake mode for all
    // NOTE digitalWrite no longer works
    analogWrite(drive_fwd_pin, 0);
    analogWrite(drive_rev_pin, 0);
    analogWrite(steer_fwd_pin, 255);
    analogWrite(steer_rev_pin, 255);
    return;
  }

  if (abs(throttle) < throttle_deadzone){
    analogWrite(drive_fwd_pin, 0);
    analogWrite(drive_rev_pin, 0);
  } else {
    int throttle_value = (int)fmap(throttle, -1.0, 1.0, -255.0, 255.0);
    throttle_value = constrain(throttle_value, -255, 255);
    if (throttle_value > 0) {
      analogWrite(drive_fwd_pin, throttle_value);
      analogWrite(drive_rev_pin, 0);
    } else {
      analogWrite(drive_rev_pin, -throttle_value);
      analogWrite(drive_fwd_pin, 0);
    }
  }

  float raw_encoder = analogRead(encoder_s_pin);
  steering_measured = fmap(raw_encoder, full_left_encoder_value, full_right_encoder_value, full_left_angle_rad,full_right_angle_rad);
  steering_measured = constrain(steering_measured, full_right_angle_rad, full_left_angle_rad);

  float err = steering - steering_measured;
  steering_requested = steering;
  /*
  Serial.print("raw: ");
  Serial.print(raw_encoder);
  Serial.print(" goal: ");
  Serial.print(steering/PI*180.0,5);
  Serial.print(" actual: ");
  Serial.print(steering_measured/PI*180.0,5);
  Serial.println();
  */
  // filter
  float dt = (float)(millis() - last_packet_ts)/1000.0;
  float freq = 10.0;
  float alfa = (2*PI*dt*freq)/(2*PI*dt*freq + 1.0);
  err = (1.0-alfa)*last_err + alfa*err;
  steering_integral += err * dt;
  steering_integral = constrain(steering_integral, -steering_integral_limit, steering_integral_limit);
  float output = err * param_steering_P + steering_integral * param_steering_I + (err - last_err)/dt * param_steering_D;

  if (abs(err) < steering_deadzone_rad){
    analogWrite(steer_fwd_pin, 0);
    analogWrite(steer_rev_pin, 0);
  } else if (output > 0){
    analogWrite(steer_rev_pin, constrain(err*param_steering_P, 0,255));
    analogWrite(steer_fwd_pin, 0);
  } else if (output < 0){
    analogWrite(steer_fwd_pin, constrain(-err*param_steering_P, 0,255));
    analogWrite(steer_rev_pin, 0);
  }

  last_pid_ts = millis();
  last_err = err;
}

