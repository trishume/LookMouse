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
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int ledPin = 13;
const float deadZone = 2;
const float moveMult[2] = {-1.0,1.0};
//const float movePow[2] = {2.0,2.0};

float zero[3] = {0,0,0};
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

void doZero(const float *point) {
  zero[0] = point[0];
  zero[1] = point[1];
  zero[2] = point[2];
}

float angleDiff(float targetA, float sourceA) {
  float a = targetA - sourceA;
  a += (a>180) ? -360 : (a<-180) ? 360 : 0;
  return a;
}

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void) 
{
  Serial.begin(115200);
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
  /* Get a new sensor event */ 
  sensors_event_t event; 
  bno.getEvent(&event);
  float orient[3];
  orient[0] = event.orientation.x;
  orient[1] = event.orientation.y;
  orient[2] = event.orientation.z;
  
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
  
  if (Serial.available() > 0) {
    // read the incoming byte:
    char incomingByte = Serial.read();
    if(incomingByte == '!') toZero = true;
    if(incomingByte == '.') {
      mouseOn = !mouseOn;
      toZero = true;
    }
  }
  
  if(toZero) {
    doZero(orient);
    toZero = false;
  }
  
  float diff[3];
  for(int i = 0; i < 3; i++)
    diff[i] = angleDiff(zero[i],orient[i]);
    
  if(diff[0] > 100 || diff[0] < -100) toZero = true;

  /* The processing sketch expects data as roll, pitch, heading */
  Serial.print(F("Orientation: "));
  Serial.print(diff[0]);
  Serial.print(F(" "));
  Serial.print(diff[1]);
  Serial.print(F(" "));
  Serial.print(diff[2]);
  Serial.println(F(""));
  
  // compensate
  diff[0] += diff[1]*1.2;
  
  char mov[2];
  float v;
  for(int i = 0; i < 2; i++) {
    if(diff[i] < -deadZone || diff[i] > deadZone) {
      mov[i] = (char)(diff[i]*moveMult[i]);
    } else {
      mov[i] = 0;
    }
  }
  if((mov[0] != 0 || mov[1] != 0) && mouseOn)
    Mouse.move(mov[0],mov[1]);
  digitalWrite(ledPin,mouseOn ? HIGH : LOW);

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
