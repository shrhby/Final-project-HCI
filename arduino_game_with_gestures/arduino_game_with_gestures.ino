#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "pitches.h"

//######### game constants BEGIN #############
const int MPU_addr=0x68;  // I2C address of the MPU-6050
#define BOARD_WIDTH 10
#define BOARD_HEIGHT 100
#define OBJECT_NUM 8
#define MIN_TIME_BETWEEN_SAME_GESTURES 2000
#define BUZZER_PIN 8
#define MAX_POINTS 100
#define TONE_DELAY 50
//######### game constants END #############

//######### gesture recognition constants BEGIN #############
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//#include "Wire.h"
//#endif
// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
int16_t ax, ay, az;
int16_t gx, gy, gz;

//float left_melody= NOTE_F4;
//float spacial_melody= NOTE_C6;
//float right_melody = NOTE_A4;
float good_melody []={NOTE_C4,NOTE_E4,NOTE_G4};
#define good_duration 300
#define bad_duration 150
#define special_duration 50
float bad_melody []={NOTE_F4,NOTE_FS4,NOTE_F4,NOTE_FS4};

float special_gest_melody[]={  NOTE_B5,NOTE_A5,NOTE_G5,NOTE_F5,NOTE_E5,NOTE_D5,NOTE_C5,};
//NOTE_B4,NOTE_A4,NOTE_G4,NOTE_F4,NOTE_E4,NOTE_D4,NOTE_C4};

#ifdef DEBUG
 #define DEBUG_PRINT(x)  Serial.print (x)
 #define DEBUG_PRINTLN(x)  Serial.println (x)
#else
 #define DEBUG_PRINT(x)
 #define DEBUG_PRINTLN(x)
#endif

#define USE_ACCEL 3
// #define USE_GYRO 3

// Leds pins
#define POINTS_FAC 10

uint16_t lastReport;

const int numReadings = 25;

#if defined(USE_ACCEL) && defined(USE_GYRO)
const int numAxis = USE_ACCEL + USE_GYRO;
const int AX = 0;
const int AY = 1;
const int AZ = 2;
const int GX = 3;
const int GY = 4;
const int GZ = 5;
#elif defined(USE_ACCEL)
const int numAxis = USE_ACCEL;
const int AX = 0;
const int AY = 1;
const int AZ = 2;
#elif defined(USE_GYRO)
const int numAxis = USE_GYRO;
const int GX = 0;
const int GY = 1;
const int GZ = 2;
#endif

int32_t readings[numAxis][numReadings];  // the reading history
int32_t readIndex[numAxis];              // the index of the current reading
int32_t total[numAxis];                  // the running total
int32_t average[numAxis];                // the average

boolean flat = false;
uint32_t flatStarted = 0;
uint32_t flatDuration = 0;
uint32_t flatLastEnded = 0;


const uint32_t Duration = 500;

// For right motion gesture identification
boolean verticalRight = false;
uint32_t verticalRightStarted = 0;
uint32_t verticalRightDuration = 0;
uint32_t verticalRightLastEnded = 0;
boolean glowingRight = false;


// For left motion gesture identification
boolean verticalLeft = false;
uint32_t verticalLeftStarted = 0;
uint32_t verticalLeftDuration = 0;
uint32_t verticalLeftLastEnded = 0;
boolean glowingLeft = false;


// For Special motion gesture identification
boolean verticalSpecial = false;
uint32_t verticalSpecialStarted = 0;
uint32_t verticalSpecialDuration = 0;
uint32_t verticalSpecialLastEnded = 0;
boolean glowingSpecial = false;

//######### gesture recognition constants END #############



//  ######### gesture recognition functions BEGIN #############
void checkFlat(){
    #ifdef USE_ACCEL
    // 17500 < average[AX] < 19000 && 5000 < average[AY] < 7000 && -6000 < average[AZ] < -4500
    boolean AX_in_range = -14700 < average[AX] && average[AX] < -14100;
    boolean AY_in_range = 4800 < average[AY] && average[AY] < 7000;
    boolean AZ_in_range = -5500 < average[AZ] && average[AZ] < -3500;
    if(AX_in_range && AY_in_range && AZ_in_range){
        if(!flat){
            flatStarted = millis();
        }
        flatLastEnded = millis();
            
        flatDuration = millis() - flatStarted;
        
        flat = true;
    } else {
        flat = false;
    }
    #endif
}



void checkRight(){
    #ifdef USE_ACCEL
    boolean AZ_in_range = -1900 < average[AZ];
    boolean AY_in_range = average[AY] < 6000;
    if( AZ_in_range && AY_in_range ){
        if(!verticalRight){ // 
            verticalRightStarted = millis();
        }
        verticalRightLastEnded = millis();
            
        verticalRightDuration = millis() - verticalRightStarted;
        
        verticalRight = true;
    } else {
        verticalRight = false;
    }
    #endif
}

void checkLeft(){
    #ifdef USE_ACCEL
    boolean AZ_in_range = average[AZ] < -7000;
    boolean AY_in_range = average[AY] < 4600;
    if( AZ_in_range && AY_in_range ){
        if(!verticalLeft){
            verticalLeftStarted = millis();
        }
        verticalLeftLastEnded = millis();
            
        verticalLeftDuration = millis() - verticalLeftStarted;
        
        verticalLeft = true;
    } else {
        verticalLeft = false;
    }
    #endif
}

void checkSpecial(){
    #ifdef USE_ACCEL
    boolean AX_in_range = -12000 < average[AX];
    boolean AY_in_range = average[AY] < -1600 ;
    if(AX_in_range && AY_in_range ){
        if(!verticalSpecial){
            verticalSpecialStarted = millis();
        }
        verticalSpecialLastEnded = millis();
            
        verticalSpecialDuration = millis() - verticalSpecialStarted;
        
        verticalSpecial = true;
    } else {
        verticalSpecial = false;
    }
    #endif
}


boolean wasRightGest(){
    if(verticalRight && (verticalRightDuration < 10 || verticalRightDuration > MIN_TIME_BETWEEN_SAME_GESTURES)){
          // Sword of Omens, Give Me Sight Beyond Sight!MIN_TIME_BETWEEN_SAME_GESTURES
        return true;
    }
    return false;
}

boolean wasLeftGest(){
    if(verticalLeft && (verticalLeftDuration < 10 || verticalLeftDuration > MIN_TIME_BETWEEN_SAME_GESTURES)){
        // Sword of Omens, Give Me Sight Beyond Sight!_TIME_BETWEEN_SAME_GESTURES)){
        return true;
    }

    return false;
}

boolean wasSpecialGest(){
    if(verticalSpecial && (verticalSpecialDuration < 10 || verticalSpecialDuration > MIN_TIME_BETWEEN_SAME_GESTURES)){
        // Sword of Omens, Give Me Sight Beyond Sight!
        return true;
    }
    return false;
}



void smooth(int axis, int32_t val) {
    // pop and subtract the last reading:
    total[axis] -= readings[axis][readIndex[axis]];
    total[axis] += val;

    // add value to running total
    readings[axis][readIndex[axis]] = val;
    readIndex[axis]++;

    if(readIndex[axis] >= numReadings)
        readIndex[axis] = 0;

    // calculate the average:
    average[axis] = total[axis] / numReadings;
}
//  ######### gesture recognition END #############


float points = BOARD_HEIGHT/2;


// ########## Game classess ###########
class Point{
  public:
  float _x;
  float _y;
  float _z;
  Point(int x, int y, int z){
    this->_x = x;
    this->_y = 0;
    this->_z = z;
  }
  Point(){
    Point(0,0,0);
  }
  Point(int z){
    this->_x = random(1,BOARD_WIDTH-1);
    this->_y = 0;
    this->_z = z;
//    int x = random(1,BOARD_WIDTH-1);
//    int y = 0;
//    Point(x,y,z);
  }
};

class Shape{
  private:
  int idx_good_evil;
  public:
  Point base;
  int good_lvl;
  
  Shape(Point p){
    base = p;
//    idx_good_evil = (random(0,8));
    good_lvl = (random(0,OBJECT_NUM));//good_evil_vals[idx_good_evil];
  }
  void move(bool is_collision, bool is_good){
    if((base._z <=-4) && is_collision){
//      Serial.println("collision less -3");
      if (is_good){
        for(int thisNote=0;thisNote<3;thisNote++){
          tone(BUZZER_PIN, good_melody[thisNote],good_duration);
          delay(TONE_DELAY);
        }
        if (points<MAX_POINTS){
          points += POINTS_FAC;
        }
      }
      else{
        for(int thisNote=0;thisNote<4;thisNote++){
          tone(BUZZER_PIN, bad_melody[thisNote],bad_duration);                   
          delay(TONE_DELAY);
        }
        if (points > 0){
          points -= POINTS_FAC;
        }
      }
      good_lvl = (random(0,OBJECT_NUM));
      base._x = random(1,BOARD_WIDTH-1);
      base._z =BOARD_HEIGHT;
    }
    else if((base._z<=1)&&(!is_collision)){
//      Serial.print("no collision ");Serial.println(is_collision);

      good_lvl = (random(0,OBJECT_NUM));
      base._x = random(1,BOARD_WIDTH-1);
      base._z =BOARD_HEIGHT;
    }
    else{
      if (base._z >=(BOARD_HEIGHT*0.5)){
        base._z--;
      }
      
      else{
        base._z -=0.5;  
      }
        
    }
    
       
  }
};

class Pad {
  public:
  Point base;
  Pad(Point p){
    base = p;
  }
  Pad(){
    Pad(Point());
  }
  void Move_left(){
    if(base._x < BOARD_WIDTH -1){
      base._x ++;  
    }
  }
  void Move_right(){
    if(base._x > 1){
      base._x --;
    }
  }
  void randomize_loc(){
    int x=base._x;
    while(x==base._x){ //randomize to different value then current.
      x = random(1,BOARD_WIDTH-1);
    }
    base._x = x;
  }
};

// ########## Game classess - END ###########

// ##### initialization of the game #####
// divide the good_lvl to color - shape and randomize both to confuse the user. Use average to determine if it is good or bad.
int m = round(BOARD_WIDTH / 2);
Point middle = Point(m,0,0);
Pad pad = Pad(middle);

  

void setup() {
//  ######### gesture recognition setup BEGIN #############
  lastReport = millis();
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
  // it's really up to you depending on your project)
//  ######### gesture recognition END #############
  // put your setup code here, to run once:
  Serial.begin(9600);
//  ######### gesture recognition BEGIN #############
 // initialize device
  accelgyro.initialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  accelgyro.setXGyroOffset(-1100);
  accelgyro.setYGyroOffset(271);
  accelgyro.setZGyroOffset(-60);
  accelgyro.setXAccelOffset(-2509);
  accelgyro.setYAccelOffset(-101);
  accelgyro.setZAccelOffset(925); // 1688 factory default for my test chip

  
  // configure Arduino BUZZER for
    pinMode(BUZZER_PIN, OUTPUT);
  

  // zero-fill all the arrays:
  for (int axis = 0; axis < numAxis; axis++) {
      readIndex[axis] = 0;
      total[axis] = 0;
      average[axis] = 0;
      for (int i = 0; i<numReadings; i++){
          readings[axis][i] = 0;
      }
  }
//  ######### gesture recognition setup END #############

//  ######### game setup BEGIN #############


//  ######### game setup END #############
}
bool is_collision =false;
bool is_good = true;
Shape s1=Shape(Point(BOARD_HEIGHT));

void loop() {
//  ######### gesture recognition BEGIN #############
  // read raw accel/gyro measurements from device
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  #ifdef USE_ACCEL
      smooth(AX, ax);
      smooth(AY, ay);
      smooth(AZ, az);
  #endif

  #ifdef USE_GYRO
      smooth(GX, gx);
      smooth(GY, gy);
      smooth(GZ, gz);
  #endif

  #ifdef OUTPUT_READABLE_ACCELGYRO
      reportAcccelGyro();
  #endif

  checkFlat();
  checkRight(); // returns verticalRight boolean True/False
  checkLeft(); // returns verticalLeft boolean True/False
  checkSpecial(); // returns verticalSpecial boolean True/ False


  // Now we will check all the terms for right/ left/ special gesture
  
  if(wasRightGest()){
//      tone(BUZZER_PIN,right_melody,Duration);
      pad.Move_right();
  }
  else if (wasLeftGest()){
//      tone(BUZZER_PIN,left_melody,Duration);
      pad.Move_left(); // here the movement to the left!!
  }
  else if (wasSpecialGest())
  {
//    tone(BUZZER_PIN,spacial_melody,Duration);
    for(int thisNote=0;thisNote<14;thisNote++){
      tone(BUZZER_PIN, special_gest_melody[thisNote],special_duration);
      delay(TONE_DELAY);
    }
    pad.randomize_loc();
  }
//  Serial.print(average[AX]), Serial.print(","), Serial.print(average[AY]), Serial.print(","), Serial.println(average[AZ]);
//  ######### gesture recognition END #############

  
  is_collision =false;
  if(((abs(s1.base._z - pad.base._z)<=1)&&(s1.base._x == pad.base._x))||(pad.base._z > s1.base._z)){
      // the shape reached the bottom
      // collision!
      is_collision =true;
      if(s1.good_lvl>=(OBJECT_NUM/2)) { 
        //caught bad 
        is_good = false;
        
      }
      else {
          // caught good 
          is_good = true;
      }
    }
  
  s1.move(is_collision, is_good);
  Serial.print(s1.base._x);Serial.print(",");Serial.print(s1.base._y);Serial.print(",");Serial.print(s1.base._z);Serial.print(";");
  Serial.print(pad.base._x);Serial.print(",");Serial.print(pad.base._y);Serial.print(",");Serial.print(pad.base._z);Serial.print(";");
  Serial.print(points);Serial.print(";");
  Serial.print(s1.good_lvl);Serial.println(";");
  
  delay(50);
}
