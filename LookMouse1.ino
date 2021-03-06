#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
   
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.

   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
   
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3-5V DC
   Connect GROUND to common ground
    
   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (5)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int ledPin = 13;
const float deadZone = 0.02;
const float moveMult[2] = {-8.0,13.0};
const float movePow[2] = {3.3,3.3};
const float moveMax[2] = {20,20};

float diff[3] = {0,0,0};
float zero[3] = {0,0,0};
int tick = 0;
int credit = 0;
bool toZero = true;

bool mouseOn = false;
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

float angleDiff(float x, float y) {
  return atan2(sin(x-y), cos(x-y));
}

inline float deg2Rad(float a) {
  return (a*71.0)/4068.0;
//  return a;
}

char weightedRound(float n) {
  float j = abs(n);
  char x = (char)j;
  if((j-x)*10000.0 <= random(10000)) {
    x++;
  }
  return x * (n<0?-1:1);
//  return (char)n;
}

float lowPassAngle(float a, float b, float weight) {
  float x = cos(a) * weight + cos(b)*(1-weight);
  float y = sin(a) * weight + sin(b)*(1-weight);
  return atan2(y,x);
}

float lowPass(float a, float b, float w) {
  return a*w+b*(1-w);
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
  Serial.begin(57600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  pinMode(ledPin, OUTPUT);
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  delay(1000);
    
  /* Display some basic information on this sensor */
  displaySensorDetails();
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void) 
{
  tick++;
  if(tick >= 100) tick = 0;
  delay(BNO055_SAMPLERATE_DELAY_MS);
  if (Serial.available() > 0) {
    // read the incoming byte:
    char incomingByte = Serial.read();
    if(incomingByte == '!') toZero = true;
    if(incomingByte == '.') {
      mouseOn = !mouseOn;
      toZero = true;
    }
    if(incomingByte == '+') {
      credit += 1000;
    }
    if(incomingByte == 'c') {
      credit = 3000;
    }
  }
  digitalWrite(ledPin,(mouseOn || (credit % 2 == 1)) ? HIGH : LOW);
  
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  
  if(credit > 0) {
    Serial.print("{\"orient\": [");
    Serial.print(event.orientation.x, 5);
    Serial.print(", ");
    Serial.print(event.orientation.y, 5);
    Serial.print(", ");
    Serial.print(event.orientation.z, 5);
    Serial.println("]}");
    credit--;
  }
  
  if(!mouseOn) return;
  
  float orient[3];
  orient[0] = deg2Rad(event.orientation.x);
  orient[1] = deg2Rad(event.orientation.y);
  orient[2] = deg2Rad(event.orientation.z);
  
  /* Board layout:
         +----------+
         |         *| RST   PITCH  ROLL  HEADING
     ADR |*        *| SCL
     INT |*        *| SDA     ^            /->
     PS1 |*        *| GND     |            |   
     PS0 |*        *| 3VO     Y    Z-->    \-X
         |         *| VIN 
         +----------+
  */
  
  if(toZero) {
    for(int i = 0; i < 3; i++) {
      zero[i] = orient[i];
    }
    toZero = false;
  }
  

  float vel[3];
  for(int i = 0; i < 3; i++) {
    vel[i] = angleDiff(zero[i],orient[i]);
    zero[i] = orient[i];
//    zero[i] = lowPassAngle(zero[i],orient[i],zeroBleed);
    diff[i] = lowPass(diff[i], vel[i]*100.0, 0.5);
  }
    
//  if(diff[0] > 100 || diff[0] < -100) toZero = true;

  /* The processing sketch expects data as roll, pitch, heading */
//  Serial.print(F("Orientation: "));
//  Serial.print(diff[0]);
//  Serial.print(F(" "));
//  Serial.print(diff[1]);
//  Serial.print(F(" "));
//  Serial.print(diff[2]);
//  Serial.println(F(""));
  
  float vec[2];
  vec[0] = diff[0]+diff[1]*1.0;
  vec[1] = diff[1];
  
  char mov[2];
  float v;
  for(int i = 0; i < 2; i++) {
    if(vec[i] < -deadZone || vec[i] > deadZone) {
      float multed = vec[i]*moveMult[i];
      v = constrain(pow(abs(multed),movePow[i]), 0, moveMax[i]);
      if(multed < 0) v *= -1;
      mov[i] = weightedRound(v);
    } else {
      mov[i] = 0;
    }
  }
  if(mov[0] != 0 || mov[1] != 0)
    Mouse.move(mov[0],mov[1]);
  
//  doZero(orient);
}
