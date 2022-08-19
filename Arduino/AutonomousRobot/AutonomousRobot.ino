// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD       // [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      200         // was 300 [-] Maximum speed for testing
#define SPEED_STEP          100          // [-] Speed step
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

// ########################## INCLUDES ##########################
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
//#include <Wire.h> // Mag Sensor
#include <Adafruit_Sensor.h> // Mag Sensor
#include <Adafruit_HMC5883_U.h> // Mag Sensor

// ########################## INSTANTIATIONS ##########################
SoftwareSerial HoverSerial(15,14);       // RX, TX Serial Connection For Hoverboard
// The TinyGPSPlus object
TinyGPSPlus gps;
// The serial connection to the GPS device
SoftwareSerial ss(50, 51);
// Compass:
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); // Mag Sensor


// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

// Front Bottom Sensor
const int FBTrigger = 2;
const int FBEcho = 3;
long FBTime;
int FBDistance;

// Right Bottom Sensor
const int RBTrigger = 4;
const int RBEcho = 5;
long RBTime;
int RBDistance;

// Front Top Sensor
const int FTTrigger = 6;
const int FTEcho = 7;
long FTTime;
int FTDistance;

// Right Top Sensor
const int RTTrigger = 8;
const int RTEcho = 9;
long RTTime;
int RTDistance;

int frac = 0;
double calc = 0;
int increment = 0;

// Hoverboard Vars:
unsigned long iTimeSend = 0;
//int iTest = 0; // was 0
int iStep = SPEED_STEP;
double iSteer = 75;
int const_Steer = 1000;
// GPS Vars:
float latitude;
float longitude;
// Compass Vars: (Calibration)
float xMax, yMax, xMin, yMin = 0.0;
bool calibrated = false;

float calculatedAngle = 0;
float headingDegrees = 0;

typedef struct {
  uint16_t start;
  int16_t  steer;
  int16_t  speed;
  uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct {
  uint16_t start;
  int16_t  cmd1;
  int16_t  cmd2;
  int16_t  speedR_meas;
  int16_t  speedL_meas;
  int16_t  batVoltage;
  int16_t  boardTemp;
  uint16_t cmdLed;
  uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup()
{
  Serial.begin(SERIAL_BAUD);  
//  HoverSerial.begin(HOVER_SERIAL_BAUD);
  ss.begin(115200);
  mag.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  // Ultrasonic Sensors:
  pinMode(FTTrigger,OUTPUT);
  pinMode(FTEcho,INPUT);
  pinMode(RTTrigger,OUTPUT);
  pinMode(RTEcho,INPUT);
  pinMode(FBTrigger,OUTPUT);
  pinMode(FBEcho,INPUT);
  pinMode(RBTrigger,OUTPUT);
  pinMode(RBEcho,INPUT);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command));
}
// ########################## HELPERS ##########################
void buildSensors(){
// Front Bottom Sensor
  digitalWrite(FBTrigger, LOW);
  delayMicroseconds(2);
  digitalWrite(FBTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(FBTrigger, LOW);
  FBTime = pulseIn(FBEcho,HIGH);
  FBDistance = FBTime * 0.034/2;
// Right Bottom Sensor
  digitalWrite(RBTrigger, LOW);
  delayMicroseconds(2);
  digitalWrite(RBTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(RBTrigger, LOW);
  RBTime = pulseIn(RBEcho,HIGH);
  RBDistance = RBTime * 0.034/2;
// Front Top Sensor
  digitalWrite(FTTrigger, LOW);
  delayMicroseconds(2);
  digitalWrite(FTTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(FTTrigger, LOW);
  FTTime = pulseIn(FTEcho,HIGH);
  FTDistance = FTTime * 0.034/2;
// Right Top Sensor
  digitalWrite(RTTrigger, LOW);
  delayMicroseconds(2);
  digitalWrite(RTTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(RTTrigger, LOW);
  RTTime = pulseIn(RTEcho,HIGH);
  RTDistance = RTTime * 0.034/2;
}
void leftNinety(int space,int speedVal){
  frac = (2 * space / 3);
  calc = (1.0 / frac) * 5000;
  Send((calc + 30)* (-1),speedVal);
}
float degreesCalculation(float heading) {
  float declinationAngle = 0.226892803; // Based off location (subject to change)
  heading += declinationAngle;
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
  return (heading * 180 / M_PI);
}
char directionFacing(float headingDegrees) {
  if (headingDegrees > 315 && headingDegrees < 359) {
    return ('W');
  }
  if (headingDegrees > 0 && headingDegrees < 45) {
    return ('W');
  }
  if (headingDegrees > 45 && headingDegrees < 135) {
    return ('N');
  }
  if (headingDegrees > 135 && headingDegrees < 225) {
    return ('E');
  }
  if (headingDegrees > 225 && headingDegrees < 315) {
    return ('S');
  }
}
double truncateFour(double val) {
  return (trunc(val * 10000)); //6
}
void update(){
  while (ss.available()){
    gps.encode(ss.read());
  }
}
int iTest = 0; // was 0, defined here cuz of turnTo
void turnTo(float dir,float degr){
  if(dir == 0){
    if(degr > 0 && degr < 90){
      Send(-40,iTest);
    }else if(degr > 270 && degr < 359){
      Send(40,iTest);
    }else if(degr == 0){
      Send(0,0);
    }else{
      Send(-50,iTest);
    }
  }else{
    if(degr > dir){
      Send(-40,iTest);
      //Serial.println("Left 40");
    }else if(degr < dir){
      Send(40,iTest);
      //Serial.println("Right 40");
    }else{
      Send(0,iTest);
      //Serial.println("Straight");
    }
  }
}
static const double destLat = 37.344611989220574, destLong = -121.98937681613279;
// 37.344706823652594, -121.98943138232244 this
// 37.344611989220574,-121.98937681613279 other
float calcDirAngle(float lati, float longi,int quadrant){
  float latDiff = destLat - lati;
  float lngDiff = destLong - longi;
  float slope = latDiff/lngDiff;
  float theta = atan(slope);
  theta = theta * 180 / M_PI;
  Serial.print("Pure Theta:");
  Serial.println(theta);
  if(theta < 0){
    if(quadrant == 2){
      theta = theta * -1;
      theta += 180;
    }
    if(quadrant == 4){
      theta = theta * -1;
    }
  }else{
    if(quadrant == 1){
      theta = 360 - theta;
    }
    if(quadrant == 3){
      theta = 360 - theta;
      theta -= 180;
    }
  }
  Serial.print("Projected Theta: ");
  Serial.println(theta);
  return(theta);
}
bool atDestination(float lati, float longi){
  if(truncateFour(lati) == truncateFour(destLat) && truncateFour(longi) == truncateFour(destLong)){
    return true;
  }else{
    return false;
  }
}
int whatQuadrant(float lati, float longi){
  if(lati < destLat && longi > destLong){
    return(4);
  }
  if(lati < destLat && longi < destLong){
    return(3);
  }
  if(lati > destLat && longi < destLong){
    return(2);
  }
  if(lati > destLat && longi > destLong){
    return(1);
  }
}
void leftNinety(float space,int speedVal){
  frac = (2 * space / 3);
  calc = (1.0 / frac) * 5000;
  Send((calc + 30)* (-1),speedVal);
}


void makeDecision(int speedVal){
    if(FTDistance <= 80){
      Serial.println("front");
      leftNinety(FTDistance,speedVal);
    }
    else{
      adjust(speedVal);
    }
}
void adjust(int speedVal){
  if(RTDistance < 45){
    Serial.println("right top");
    Send(25 * (-1),speedVal);
  }else if(RBDistance < 50){ // 122
    Send(25 * (-1),speedVal);// steer was 20
    Serial.println("Going LEFT");
  }else{
    turnTo(calculatedAngle,headingDegrees);
    if(atDestination(latitude,longitude) == true){
      Send(0,0);
    }
  }
}
// ########################## LOOP ##########################
void loop() {
  buildSensors();
  unsigned long timeNow = millis();
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  iTest += iStep;
  // Making sure system clock is in sync ^^^
  sensors_event_t event;
  mag.getEvent(&event);
  update();
  if(timeNow < 1000){
    Send(0,iTest);
    Serial.println("Straight");
  }else if(timeNow < 10000){
    Serial.println("Calibrating");
    Send(120, iTest);
    if (xMax == 0.0) {
      xMax = event.magnetic.x;
    }
    if (yMax == 0.0){
      yMax = event.magnetic.y;
    }
    xMax = max(xMax, event.magnetic.x);
    yMax = max(yMax, event.magnetic.y);
    xMin = min(xMin, event.magnetic.x);
    yMin = min(yMin, event.magnetic.y);
  }else{
    float heading = atan2((event.magnetic.y - ((yMax + yMin) / 2.0)), (event.magnetic.x - ((xMax + xMin) / 2.0)));
    headingDegrees = degreesCalculation(heading);
    char facing = directionFacing(headingDegrees);
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    Serial.println(facing);
    int quadrant = whatQuadrant(latitude,longitude);
    Serial.print("Quadrant: ");
    Serial.println(quadrant);
    calculatedAngle = calcDirAngle(latitude, longitude,quadrant);
  }
  makeDecision(iTest);
  
  if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST){
    iStep = 0; // -iStep
  }
  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow % 2000) < 1000);

}
// ########################## END ##########################
