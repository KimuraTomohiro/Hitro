#define BRAKEVCC 0
#define CW   1
#define CCW  2
#define BRAKEGND 3
#define CS_THRESHOLD 100

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include "EEPROM.h" 


int n=0;

int FL_zero;
int FR_zero;
int BL_zero;
int BR_zero;

const int address1 = 0;
const int address2 = sizeof(int);
const int address3 = 2 * sizeof(int);
const int address4 = 3 * sizeof(int);


char hello[13] = "hello world!";
int inApin[6] = {27,29,50,53,8,11};  // INA: Clockwise input
int inBpin[6] = {25,23,48,46,7,12}; // INB: Counter-clockwise input
int pwmpin[6] = {5,2,4,3,10,9}; // PWM input
//int cspin[6] = {0,1,2,3,4,5}; // CS: Current sense ANALOG input
//int enpin[2] = {0, 1}; // EN: Status of switches output (Analog pin)
int analogpin[4]={12,13,14,15};//potentio,
float motor_target_before[6]={0,0,0,0,0,0};
float motor_target[6]={0,0,0,0,0,0};
float flipper_zero_value[4]={0,0,0,0};
ros::NodeHandle  nh;
std_msgs::String str_msg;
std_msgs::Float32MultiArray motor_msg;
const int ARRAY_LENGTH=6;
const float drive_ratio=0.4;
const float flipper_ratio=0.1;

std_msgs::Float32MultiArray flip_msg;
const int flip_ARRAY_LENGTH=4;

std_msgs::Float32MultiArray flip_val_msg;
const int flip_val_ARRAY_LENGTH=4;

ros::Publisher pub_flip("flipper_deg", &flip_msg);
ros::Publisher pub_flip_val("flipper_val", &flip_val_msg);
//ros::Publisher chatter("messagefromarduino", &str_msg);
bool updating=false;

void MotorOff()
{
  // Initialize braked
  for (int i = 0; i < 6; i++)
  {
    digitalWrite(inApin[i], LOW);
    digitalWrite(inBpin[i], LOW);
      analogWrite(pwmpin[i], 0);
  }

}

/* motorGo() will set a motor going in a specific direction
  the motor will continue going in that direction, at that speed
  until told to do otherwise.

  motor: this should be either 0 or 1, will selet which of the two
  motors to be controlled

  direct: Should be between 0 and 3, with the following result
  0: Brake to VCC
  1: Clockwise
  2: CounterClockwise
  3: Brake to GND

  pwm: should be a value between ? and 1023, higher the number, the faster
  it'll go
*/
void motorGo(uint8_t motor, uint8_t direct, uint8_t pwm)
{
    if (direct <= 4)
    {
      // Set inA[motor]
      if (direct <= 1)
        digitalWrite(inApin[motor], HIGH);
      else
        digitalWrite(inApin[motor], LOW);

      // Set inB[motor]
      if ((direct == 0) || (direct == 2))
        digitalWrite(inBpin[motor], HIGH);
      else
        digitalWrite(inBpin[motor], LOW);

      analogWrite(pwmpin[motor], pwm);
    }
}

void MotorUpdate(){
  updating=true;
for(int i=0;i<6;i++){
if(motor_target_before[i]==motor_target[i]){
  
}
else{
  float pwm_value=0;
  if(i<2){
      pwm_value=abs(motor_target[i])*drive_ratio;
  }else if(i>1 && i<6){
      pwm_value=abs(motor_target[i])*flipper_ratio;
  }

  if(motor_target[i]>0)
  {
      motorGo(i, CW, 1023*pwm_value);
  }
  else if(motor_target[i]<0)
  {
      motorGo(i, CCW, 1023*pwm_value);
  }
  else if(motor_target[i]==0){
    motorGo(i,3,0);
  }
  //str_msg.data =String(MotorTarget.data[i],3);
 // chatter.publish(&str_msg);
motor_target_before[i]=motor_target[i];
}
}
  updating=false;
  }
  
void cb_motortarget(const std_msgs::Float32MultiArray& MotorTarget){
for(int i=0;i<6;i++){
  motor_target[i]=MotorTarget.data[i];
  }
}
void flipeer_cari(){
      int val1=analogRead(A12);//FL
    int val2=analogRead(A13);//FR
    int val3=analogRead(A14);//BL
    int val4=analogRead(A15);//BR
  EEPROM.put(address1, val1);
  EEPROM.put(address2, val2);
  EEPROM.put(address3, val3);
  EEPROM.put(address4, val4);
    FL_zero = val1; 
  FR_zero = val2; 
  BL_zero = val3; 
  BR_zero = val4; 
  
}
void cb_flipeer_cari(const std_msgs::Bool &msg){
  if(msg.data){
    int val1=analogRead(A12);//FL
    int val2=analogRead(A13);//FR
    int val3=analogRead(A14);//BL
    int val4=analogRead(A15);//BR
  EEPROM.put(address1, val1);
  EEPROM.put(address2, val2);
  EEPROM.put(address3, val3);
  EEPROM.put(address4, val4);
    FL_zero = val1; 
  FR_zero = val2; 
  BL_zero = val3; 
  BR_zero = val4; 
    }
  
}

ros::Subscriber<std_msgs::Float32MultiArray>sub_motor("hitro_joy_to_arduino",&cb_motortarget); 
ros::Subscriber<std_msgs::Bool>sub_flipper_cari("flipper_cari",&cb_flipeer_cari); 

void FlipUpdate(){
 
flip_msg.data[0]=analogRead(A12);//FL
flip_msg.data[1]=analogRead(A13);//FR
flip_msg.data[2]=analogRead(A14);//BL
flip_msg.data[3]=analogRead(A15);//BR
flip_val_msg.data[0]=FL_zero;//FL
flip_val_msg.data[1]=FR_zero;//FR
flip_val_msg.data[2]=BL_zero;//BL
flip_val_msg.data[3]=BR_zero;//BR

pub_flip.publish(&flip_msg);
pub_flip_val.publish(&flip_val_msg);
}
void setup()
{
  motor_msg.data = (float*)malloc(sizeof(float) *ARRAY_LENGTH );
  motor_msg.data_length = ARRAY_LENGTH;

  flip_msg.data = (float*)malloc(sizeof(float) *flip_ARRAY_LENGTH );
  flip_msg.data_length = flip_ARRAY_LENGTH;

  flip_val_msg.data = (float*)malloc(sizeof(float) *flip_val_ARRAY_LENGTH );
  flip_val_msg.data_length = flip_val_ARRAY_LENGTH;
  
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(13, OUTPUT);
  for (int i = 0; i < 6; i++)
  {
    pinMode(inApin[i], OUTPUT);
    pinMode(inBpin[i], OUTPUT);
    pinMode(pwmpin[i], OUTPUT);
  }

  int readValue1, readValue2, readValue3, readValue4;
  EEPROM.get(address1, readValue1);
  EEPROM.get(address2, readValue2);
  EEPROM.get(address3, readValue3);
  EEPROM.get(address4, readValue4);
  
  FL_zero = readValue1; 
  FR_zero = readValue2; 
  BL_zero = readValue3; 
  BR_zero = readValue4; 

flip_val_msg.data[0]=FL_zero;//FL
flip_val_msg.data[1]=FR_zero;//FR
flip_val_msg.data[2]=BL_zero;//BL
flip_val_msg.data[3]=BR_zero;//BR

  nh.getHardware()->setBaud(921600);
  nh.initNode();

  nh.subscribe(sub_motor);
  nh.subscribe(sub_flipper_cari);
  nh.advertise(pub_flip);
  nh.advertise(pub_flip_val);
    flipeer_cari();
}

void loop()
{
 
  nh.spinOnce();
 // if(!updating){MotorUpdate();}
  MotorUpdate();
 FlipUpdate();
   delay(100);
   //MotorOff();
}
