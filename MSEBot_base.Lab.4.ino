#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmMotor;    
Servo servo_GripMotor;
Servo servo_WristMotor;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_ENCODERS
//#define DEBUG_ULTRASONIC
//#define DEBUG_MOTOR_CALIBRATION


//PIN VARIABLES
const int pin_Ultrasonic_Ping = 2;   //input plug
const int pin_Ultrasonic_Data = 3;   //output plug
const int pin_Charlieplex_LED1 = 4;
const int pin_Charlieplex_LED2 = 5;
const int pin_Charlieplex_LED3 = 6;
const int pin_Charlieplex_LED4 = 7;
const int pin_HallEffect_rightCl = 4;
const int pin_HallEffect_leftCl = 5;
const int pin_HallEffect_rightCh = 6;
const int pin_HallEffect_middleCh = 7;
const int pin_HallEffect_leftCh = 8;
const int pin_Mode_Button = 7;
const int pin_Right_Motor = 9;
const int pin_Left_Motor = 10;
const int pin_Arm_Motor = 11;
const int pin_Wrist_Motor = 12;
const int pin_Grip_Motor = 13;
const int pin_Right_Line_Tracker = A0;
const int pin_Middle_Line_Tracker = A1;
const int pin_Left_Line_Tracker = A2;
const int pin_I2C_SDA = A4;         // I2C data = white
const int pin_I2C_SCL = A5;         // I2C clock = yellow

// Charlieplexing LED assignments
const int CP_Heartbeat_LED = 1;
const int CP_Indicator_LED = 4;
const int CP_Right_Line_Tracker_LED = 6;
const int CP_Middle_Line_Tracker_LED = 9;
const int CP_Left_Line_Tracker_LED = 12;

//constants

// EEPROM addresses
const int Left_Line_Tracker_Dark_Address_L = 0;
const int Left_Line_Tracker_Dark_Address_H = 1;
const int Left_Line_Tracker_Light_Address_L = 2;
const int Left_Line_Tracker_Light_Address_H = 3;
const int Middle_Line_Tracker_Dark_Address_L = 4;
const int Middle_Line_Tracker_Dark_Address_H = 5;
const int Middle_Line_Tracker_Light_Address_L = 6;
const int Middle_Line_Tracker_Light_Address_H = 7;
const int Right_Line_Tracker_Dark_Address_L = 8;
const int Right_Line_Tracker_Dark_Address_H = 9;
const int Right_Line_Tracker_Light_Address_L = 10;
const int Right_Line_Tracker_Light_Address_H = 11;

const int Left_Motor_Offset_Address_L = 12;
const int Left_Motor_Offset_Address_H = 13;
const int Right_Motor_Offset_Address_L = 14;
const int Right_Motor_Offset_Address_H = 15;

const int const_Grip_Motor_Open = 140;    
const int const_Grip_Motor_Closed = 90;      
const int const_Arm_Servo_Retracted = 55;      
const int const_Arm_Servo_Extended = 120;      
const int const_Display_Time = 500;
const int const_Line_Tracker_Calibration_Interval = 100;
const int const_Line_Tracker_Cal_Measures = 20;
const int const_Line_Tracker_Tolerance = 50;  
const int const_Motor_Calibration_Cycles = 3;
const int const_Motor_Calibration_Time = 5000;

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long Echo_Time;
unsigned int Left_Line_Tracker_Data;
unsigned int Middle_Line_Tracker_Data;
unsigned int Right_Line_Tracker_Data;
long Left_Motor_Position;
long Right_Motor_Position;

unsigned long Display_Time;
unsigned long Calibration_Time;
unsigned long Left_Motor_Offset;
unsigned long Right_Motor_Offset;

unsigned int Left_Line_Tracker_Dark;
unsigned int Left_Line_Tracker_Light;
unsigned int Middle_Line_Tracker_Dark;
unsigned int Middle_Line_Tracker_Light;
unsigned int Right_Line_Tracker_Dark;
unsigned int Right_Line_Tracker_Light;
unsigned int Line_Tracker_Tolerance;

unsigned int  Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  Mode_Indicator[6] = {
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run
  0x0F0F,  //B0000111100001111,  //Calibrate line tracker light level
  0x3333,  //B0011001100110011,  //Calibrate line tracker dark level
  0xAAAA,  //B1010101010101010,  //Calibrate motors
  0xFFFF   //B1111111111111111   //Unused
};

unsigned int  Mode_Indicator_Index = 0;

//display Bits 0,1,2,3, 4, 5, 6,  7,  8,  9,  10,  11,  12,  13,   14,   15
int  iArray[16] = {1,2,4,8,16,32,64,128,256,512,1024,2048,4096,8192,16384,65536};
int  iArrayIndex = 0;

boolean bt_Heartbeat = true;
boolean bt_3_S_Time_Up = false;
boolean bt_Do_Once = false;
boolean bt_Cal_Initialized = false;

void setup() {
  Wire.begin();        // Wire library required for I2CEncoder library
  Serial.begin(9600);

  CharliePlexM::setBtn(pin_Charlieplex_LED1, pin_Charlieplex_LED2, pin_Charlieplex_LED3, pin_Charlieplex_LED4, pin_Mode_Button);

  // set up ultrasonic
  pinMode(pin_Ultrasonic_Ping, OUTPUT);
  pinMode(pin_Ultrasonic_Data, INPUT);

  // set up drive motors
  pinMode(pin_Right_Motor, OUTPUT);
  servo_RightMotor.attach(pin_Right_Motor);
  pinMode(pin_Left_Motor, OUTPUT);
  servo_LeftMotor.attach(pin_Left_Motor);

  // set up arm motors
  pinMode(pin_Arm_Motor, OUTPUT);
  servo_ArmMotor.attach(pin_Arm_Motor);
  pinMode(pin_Grip_Motor, OUTPUT);
  servo_GripMotor.attach(pin_Grip_Motor);
  pinMode(pin_Wrist_Motor, OUTPUT);
  servo_WristMotor.attach(pin_Wrist_Motor);


  // set up encoders. Must be initialized in order that they are chained together, 
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward

  // set up line tracking sensors
  pinMode(pin_Right_Line_Tracker, INPUT);
  pinMode(pin_Middle_Line_Tracker, INPUT);
  pinMode(pin_Left_Line_Tracker, INPUT);
  Line_Tracker_Tolerance = const_Line_Tracker_Tolerance;

  // read saved values from EEPROM
  b_LowByte = EEPROM.read(Left_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Left_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Left_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Left_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Middle_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Middle_Line_Tracker_Dark = word(b_HighByte, b_LowByte); 
  b_LowByte = EEPROM.read(Middle_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Middle_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Right_Line_Tracker_Dark_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Right_Line_Tracker_Dark = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Right_Line_Tracker_Light_Address_L);
  b_HighByte = EEPROM.read(Left_Line_Tracker_Dark_Address_H);
  Right_Line_Tracker_Light = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Left_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(Left_Motor_Offset_Address_H);
  Left_Motor_Offset = word(b_HighByte, b_LowByte);
  b_LowByte = EEPROM.read(Right_Motor_Offset_Address_L);
  b_HighByte = EEPROM.read(Right_Motor_Offset_Address_H);
  Right_Motor_Offset = word(b_HighByte, b_LowByte);
}

void loop()
{
 
  // button-based mode selection
  if(CharliePlexM::ui_Btn)
  {
      if(bt_Do_Once == false)
      {
          bt_Do_Once = true;
          Robot_State_Index++;
          Robot_State_Index = Robot_State_Index & 7;
      }
  }

  
  // Button Modes 
  // 0 = default after power up/reset
  // 1 = Press mode button once to enter. Run robot.
  // 2 = Press mode button twice to enter. Calibrate line tracker light level.
  // 3 = Press mode button three times to enter. Calibrate line tracker dark level.


  switch(Robot_State_Index)
  {
    case 0:    //Robot stopped
    {
      readLineTrackers();
      UltrasonicPing();
      servo_LeftMotor.write(1500); 
      servo_RightMotor.write(1500); 
      
       //detach the claw and arm motors
       servo_GripMotor.detach();
       servo_WristMotor.detach();
       servo_ArmMotor.detach();

       //zero the encoder
      encoder_LeftMotor.zero();
      encoder_RightMotor.zero();
      
      break;
    } 

  
    case 1:  //Robot drive straight
    {
       //turn off the grip motor
       servo_GripMotor.detach();
       servo_WristMotor.detach();
       servo_ArmMotor.detach();
      
      //turn the motors on, drive straight until the bumper switch is hit
      servo_LeftMotor.write(1700); 
      servo_RightMotor.write(1700); 

      break;
    } 




























 
  } //end of switch statement

Serial.print("Mode: ");
Serial.println(Robot_State_Index);

  
}//end of loop() 
//---------------------------------------------------------------------------------


//---------FUNCTIONS--------------------------------------------------------------


// measure distance to target using ultrasonic sensor  
void UltrasonicPing()
{
  //Ping Ultrasonic
  //Send the Ultrasonic Range Finder a 10 microsecond pulse per tech spec
  digitalWrite(pin_Ultrasonic_Ping, HIGH);
  delayMicroseconds(10);  //The 10 microsecond pause where the pulse in "high"
  digitalWrite(pin_Ultrasonic_Ping, LOW);
  //use command pulseIn to listen to Ultrasonic_Data pin to record the
  //time that it takes from when the Pin goes HIGH until it goes LOW 
  Echo_Time = pulseIn(pin_Ultrasonic_Data, HIGH, 10000);

  // Print Sensor Readings
#ifdef DEBUG_ULTRASONIC
  Serial.print("Time (microseconds): ");
  Serial.print(Echo_Time, DEC);
  Serial.print(", Inches: ");
  Serial.print(Echo_Time/148); //divide time by 148 to get distance in inches
  Serial.print(", cm: ");
  Serial.println(Echo_Time/58); //divide time by 58 to get distance in cm 
#endif
}  

