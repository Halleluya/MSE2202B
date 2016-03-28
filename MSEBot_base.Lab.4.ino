#include <Servo.h>
#include <EEPROM.h>
#include <uSTimer2.h>
#include <CharliePlexM.h>
#include <Wire.h>
#include <I2CEncoder.h>

Servo servo_RightMotor;
Servo servo_LeftMotor;
Servo servo_ArmServoLeft;
Servo servo_ArmServoRight;
Servo servo_GripServo;
Servo servo_WristServo;

I2CEncoder encoder_RightMotor;
I2CEncoder encoder_LeftMotor;

// Uncomment keywords to enable debugging output

//#define DEBUG_MODE_DISPLAY
//#define DEBUG_MOTORS
//#define DEBUG_ENCODERS
#define DEBUG_ULTRASONIC
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
const int pin_HallEffect_leftCh = 8; ///*****************arm motor plugged into this one
const int pin_Mode_Button = 7;
const int pin_Right_Motor = 9;
const int pin_Left_Motor = 10;
const int pin_Arm_Servo_Right = 11;
const int pin_Arm_Servo_Left = 8;
const int pin_Wrist_Servo = 12;
const int pin_Grip_Servo = 13;
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
const int const_Grip_Servo_Open = 120;    
const int const_Grip_Servo_Closed = 45;
    
const int const_LeftArm_Servo_Down = 55;  
const int const_LeftArm_Servo_Up = 120;
const int const_RightArm_Servo_Down = 55;  
const int const_RightArm_Servo_Up = 120; 

const int const_Wrist_Servo_Down = 25; 
const int const_Wrist_Servo_Middle = 100;    
const int const_Wrist_Servo_Up = 170; 
 

//variables
byte b_LowByte;
byte b_HighByte;
unsigned long Echo_Time;
long Left_Motor_Position;
long Right_Motor_Position;

unsigned long Display_Time;
unsigned long Left_Motor_Offset;
unsigned long Right_Motor_Offset;

unsigned int  Robot_State_Index = 0;
//0123456789ABCDEF
unsigned int  Mode_Indicator[6] = 
{
  0x00,    //B0000000000000000,  //Stop
  0x00FF,  //B0000000011111111,  //Run Mode 1
  0x0F0F,  //B0000111100001111,  //Run Mode 2
  0x3333,  //B0011001100110011,  //Mode 3
  0xAAAA,  //B1010101010101010,  //Mode 4
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



//james' variables to keep wheels straight -----------------
int rightMotorSpeed = 1700;
int leftMotorSpeed = 1700;

int RoldE = 0;
int LoldE = 0;

int MODULARSPEED = 10;
//----------------------------------------------------------

//bridge's variable for the turn right function
int rightEncoder = 0;



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
  pinMode(pin_Arm_Servo_Right, OUTPUT);
  servo_ArmServoRight.attach(pin_Arm_Servo_Right);
  pinMode(pin_Arm_Servo_Left, OUTPUT);
  servo_ArmServoLeft.attach(pin_Arm_Servo_Left);
  pinMode(pin_Grip_Servo, OUTPUT);
  servo_GripServo.attach(pin_Grip_Servo);
  pinMode(pin_Wrist_Servo, OUTPUT);
  servo_WristServo.attach(pin_Wrist_Servo);


  // set up encoders. Must be initialized in order that they are chained together, 
  // starting with the encoder directly connected to the Arduino. See I2CEncoder docs
  // for more information
  encoder_LeftMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);
  encoder_LeftMotor.setReversed(false);  // adjust for positive count when moving forward
  encoder_RightMotor.init(1.0/3.0*MOTOR_393_SPEED_ROTATIONS, MOTOR_393_TIME_DELTA);  
  encoder_RightMotor.setReversed(true);  // adjust for positive count when moving forward
}


void loop()
{
  /*
    Serial.print(leftMotorSpeed);
    Serial.print(", ");
    Serial.println(rightMotorSpeed);
*/

  
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
  // 0 = default robot stopped
  // 1 = Press mode button once to run Mode 1 on the robot
  // 2 = Press mode button twice to run Mode 2 on the robot


      switch(Robot_State_Index)
      {
              case 0:    //Robot stopped
              {
                UltrasonicPing();
                servo_LeftMotor.write(1500); 
                servo_RightMotor.write(1500); 

                //close claw before detaching it
                servo_GripServo.write(45);
                delay(1000);
                
                 //detach the claw and arm motors
                 servo_GripServo.detach();
                 servo_WristServo.detach();
                 servo_ArmServoLeft.detach();
                 servo_ArmServoRight.detach();
          
                 //zero the encoder
                encoder_LeftMotor.zero();
                encoder_RightMotor.zero();
                
                break;
              } 
          
            
              case 1:  //Robot run mode 1
              {
                 //turn off the motors
                 servo_GripServo.detach();
                 servo_WristServo.detach();
                 servo_ArmServoLeft.detach();
                 servo_ArmServoRight.detach();

                 //turn the motors on, drive straight until hit a wall
                 stabalizeMotorSpeeds();

                 
                UltrasonicPing();          
                if ((Echo_Time/24 < 30) && (Echo_Time != 0))    //if distance is less than 10 and not during startup
                {
                     Serial.println("Just btw I'm close to the wall and stopping.");
                     leftMotorSpeed = 1500;
                     rightMotorSpeed = 1500;
                     
                     turnAroundRight();           //call function to turn around right
                }

                //keep driving straight (for testing purposes)
                servo_LeftMotor.write(leftMotorSpeed); 
                servo_RightMotor.write(rightMotorSpeed);
                
     




#ifdef DEBUG_ENCODERS           
        l_Left_Motor_Position = encoder_LeftMotor.getRawPosition();
        l_Right_Motor_Position = encoder_RightMotor.getRawPosition();

        Serial.print("Encoders L: ");
        Serial.print(l_Left_Motor_Position);
        Serial.print(", R: ");
        Serial.println(l_Right_Motor_Position);
#endif


                
               
          
                break;
              } 
          
              case 2:
              {
          
                GripServo();
                break;
                
              }
    
    
    
     
      } //end of switch statement

//Serial.print("Mode: ");
//Serial.println(Robot_State_Index);

}//end of loop() 
//---------------------------------------------------------------------------------


//---------FUNCTIONS--------------------------------------------------------------
 
void UltrasonicPing()       // measure distance to target using ultrasonic sensor 
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



//function to simply pick up tesseracts
void GripServo()
{
      servo_WristServo.write(const_Wrist_Servo_Middle);
      servo_GripServo.write(const_Grip_Servo_Open);
      delay(1000);
      servo_WristServo.write(const_Wrist_Servo_Down);
      delay(1000);
      servo_GripServo.write(const_Grip_Servo_Closed);
      delay(1000);
      servo_WristServo.write(const_Wrist_Servo_Middle);
      
  
}


void stabalizeMotorSpeeds()
{

  
  if (/*millis() % 10 == 0 && */leftMotorSpeed != 1500 && rightMotorSpeed != 1500)
  {

        int leftDiff = encoder_LeftMotor.getRawPosition() - LoldE;
        int rightDiff = encoder_RightMotor.getRawPosition() - RoldE;
    
        Serial.print("leftDiff: ");
        Serial.print(leftDiff);
        Serial.print("   rightDiff: ");
        Serial.println(rightDiff);   
      
        if (leftDiff < rightDiff){leftMotorSpeed += MODULARSPEED;}
        else if (leftDiff > rightDiff){leftMotorSpeed -= MODULARSPEED;}
    
        LoldE = encoder_LeftMotor.getRawPosition();
        RoldE = encoder_RightMotor.getRawPosition();
  }
 
}



void turnAroundRight()
{

        encoder_RightMotor.zero();
        rightEncoder = 0;


        //turn right parallel to wall
        while(rightEncoder > -12000)
        {
            servo_LeftMotor.write(1650);
            servo_RightMotor.write(1350);

            Serial.print("Encoder R: ");
            Serial.println(rightEncoder);
 
            rightEncoder = rightEncoder + encoder_RightMotor.getRawPosition();
                  
        }

        Serial.print("DONE FIRST TURN RIGHT");
        
        encoder_RightMotor.zero();
        rightEncoder = 0;

        //set motor speeds to 1650
        rightMotorSpeed = 1650;
        leftMotorSpeed = 1650;

         //drive straight parallel to wall
        while(rightEncoder < 10000)
        {
           
            servo_LeftMotor.write(leftMotorSpeed);
            servo_RightMotor.write(rightMotorSpeed);
            stabalizeMotorSpeeds();

            Serial.print("Encoder R: ");
            Serial.println(rightEncoder);
    
            rightEncoder = rightEncoder + encoder_RightMotor.getRawPosition();
                  
        }

        Serial.print("DONE DRIVE STRAIGHT");
              encoder_RightMotor.zero();
              rightEncoder = 0;


        
        //turn right again to start driving away from wall
        while(rightEncoder > -12000)
        {
            servo_LeftMotor.write(1650);
            servo_RightMotor.write(1350);

            Serial.print("Encoder R: ");
            Serial.println(rightEncoder);
 
            rightEncoder = rightEncoder + encoder_RightMotor.getRawPosition();      
        }
        
       Serial.print("DONE SECOND TURN RIGHT");
              encoder_RightMotor.zero();
              rightEncoder = 0;
}



void turnAroundLeft()
{

        encoder_RightMotor.zero();
        rightEncoder = 0;


        //turn left parallel to wall
        while(rightEncoder < 12000)
        {
            servo_LeftMotor.write(1350);
            servo_RightMotor.write(1650);

            Serial.print("Encoder R: ");
            Serial.println(rightEncoder);
 
            rightEncoder = rightEncoder + encoder_RightMotor.getRawPosition();
                  
        }

        encoder_RightMotor.zero();
        rightEncoder = 0;

        //set motor speeds to 1650
        rightMotorSpeed = 1650;
        leftMotorSpeed = 1650;

         //drive straight parallel to wall
        while(rightEncoder > -10000)
        {
           
            servo_LeftMotor.write(leftMotorSpeed);
            servo_RightMotor.write(rightMotorSpeed);
            stabalizeMotorSpeeds();

            Serial.print("Encoder R: ");
            Serial.println(rightEncoder);
    
            rightEncoder = rightEncoder + encoder_RightMotor.getRawPosition();
                  
        }

              encoder_RightMotor.zero();
              rightEncoder = 0;
        
        //turn left again to start driving away from wall
        while(rightEncoder < 12000)
        {
            servo_LeftMotor.write(1350);
            servo_RightMotor.write(1650);

            Serial.print("Encoder R: ");
            Serial.println(rightEncoder);
 
            rightEncoder = rightEncoder + encoder_RightMotor.getRawPosition();
                  
        }

              encoder_RightMotor.zero();
              rightEncoder = 0;
      

}
