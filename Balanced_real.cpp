#include "Balanced_real.h"
#include "Wire.h"
#include "Motor_real.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "KalmanFilter.h"
#include "sbus.h"

void HandleLeftMotorInterrupt();
void HandleRightMotorInterrupt();
int ParseEncoder(bool EncoderAPrev, bool EncoderBPrev
      ,bool EncoderASet, bool EncoderBSet);

MPU6050 MPU6050;
Mpu6050 Mpu6050;

Balanced Balanced;
KalmanFilter kalmanfilter;
Motor Motor;

int max_pwm = 700;

Balanced::Balanced()
{ 
  kp_balance = 60.0, kd_balance = 5.0;

  // kp_speed = 4.0, ki_speed = 0.1; //kp speedd s enerve a 7.5 (c'était à cause du signe)
  kp_speed = 4.00, ki_speed = 0.3; //kp speedd s enerve a 7.5 (c'était à cause du signe)

  kp_turn = 0.0, kd_turn = 0.0;
  
  offset_orientation=0.0;
}

double constrain_val(double val, double min_val, double max_val) {
  if (val < min_val) {
    return min_val;
  } else if (val > max_val) {
    return max_val;
  } else {
    return val;
  }
}

void Balanced::Total_Control()
{
  if (not arret_moteur){
    pwm_left = balance_control_output - speed_control_output - rotation_control_output;//Superposition of Vertical Velocity Steering Ring
    pwm_right = balance_control_output - speed_control_output + rotation_control_output;//Superposition of Vertical Velocity Steering Ring

    // pwm_left = - speed_control_output ;//Superposition of Vertical Velocity Steering Ring
    // pwm_right =  - speed_control_output ;//Superposition of Vertical Velocity Steering Ring


    pwm_left = constrain_val(pwm_left, -max_pwm, max_pwm);
    pwm_right = constrain_val(pwm_right, -max_pwm, max_pwm);

    

    // cmd_sens += 1;

    // if (cmd_sens%100==0)
    // {
    //   pwm_left_imp = -pwm_left_imp;
    //   pwm_right_imp = -pwm_right_imp;
    // }

    // pwm_left = pwm_left + pwm_left_imp;
    // pwm_right = pwm_right + pwm_right_imp;


    // while(EXCESSIVE_ANGLE_TILT || PICKED_UP) 
    // { 
    //   Mpu6050.DataProcessing();
    //   Motor.Stop();
    // }
    
    Motor.Control(pwm_left,pwm_right);
  }
  else{
    Motor.Stop();
  }
  
  
}

///// Codes encodeurs quadratures //////
#include <digitalWriteFast.h>

// Left encoder
#define c_LeftEncoderPinA 2
#define c_LeftEncoderPinB 3

#define c_RightEncoderPinA 18
#define c_RightEncoderPinB 19


volatile bool _LeftEncoderASet;
volatile bool _LeftEncoderBSet;
volatile bool _LeftEncoderAPrev;
volatile bool _LeftEncoderBPrev;
volatile long _LeftEncoderTicks = 0;
volatile long _LeftEncoderTicks_actu = 0;

volatile bool _RightEncoderASet;
volatile bool _RightEncoderBSet;
volatile bool _RightEncoderAPrev;
volatile bool _RightEncoderBPrev;
volatile long _RightEncoderTicks = 0;
volatile long _RightEncoderTicks_actu = 0;

void Balanced::Encoder_init()
{
  // Quadrature encoders
  // Left encoder
  pinMode(c_LeftEncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_LeftEncoderPinB, INPUT_PULLUP);      // sets pin B as input
  attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinA), HandleLeftMotorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(c_LeftEncoderPinB), HandleLeftMotorInterrupt, CHANGE);

  // Right encoder
  pinMode(c_RightEncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_RightEncoderPinB, INPUT_PULLUP);      // sets pin B as input
  attachInterrupt(digitalPinToInterrupt(c_RightEncoderPinA), HandleRightMotorInterrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(c_RightEncoderPinB), HandleRightMotorInterrupt, CHANGE);
}

// Interrupt service routines for the right motor's quadrature encoder
void HandleLeftMotorInterrupt(){
  // Test transition;
  _LeftEncoderBSet = digitalReadFast(c_LeftEncoderPinB);
  _LeftEncoderASet = digitalReadFast(c_LeftEncoderPinA);
  
  _LeftEncoderTicks+=ParseEncoder(_LeftEncoderAPrev, _LeftEncoderBPrev
                                  , _LeftEncoderASet, _LeftEncoderBSet);
  
  _LeftEncoderAPrev = _LeftEncoderASet;
  _LeftEncoderBPrev = _LeftEncoderBSet;
}

void HandleRightMotorInterrupt(){
  // Test transition;
  _RightEncoderBSet = digitalReadFast(c_RightEncoderPinB);
  _RightEncoderASet = digitalReadFast(c_RightEncoderPinA);
  
  _RightEncoderTicks+=ParseEncoder(_RightEncoderAPrev, _RightEncoderBPrev
                                  , _RightEncoderASet, _RightEncoderBSet);
  
  _RightEncoderAPrev = _RightEncoderASet;
  _RightEncoderBPrev = _RightEncoderBSet;
}


int ParseEncoder(bool EncoderAPrev, bool EncoderBPrev
                ,bool EncoderASet, bool EncoderBSet)
{
  if (EncoderAPrev && EncoderBPrev){
    if(!EncoderASet && EncoderBSet) return 1;
    if(EncoderASet && !EncoderBSet) return -1;
  }else if(!EncoderAPrev && EncoderBPrev){
    if(!EncoderASet && !EncoderBSet) return 1;
    if(EncoderASet && EncoderBSet) return -1;
  }else if(!EncoderAPrev && !EncoderBPrev){
    if(EncoderASet && !EncoderBSet) return 1;
    if(!EncoderASet && EncoderBSet) return -1;
  }else if(EncoderAPrev && !EncoderBPrev){
    if(EncoderASet && EncoderBSet) return 1;
    if(!EncoderASet && !EncoderBSet) return -1;
  }
}

void Balanced::Get_EncoderSpeed()
{
  /// Print VITESSE ///
  long vit_Left = _LeftEncoderTicks - _LeftEncoderTicks_actu;
  _LeftEncoderTicks_actu = _LeftEncoderTicks;

  long vit_Right = _RightEncoderTicks - _RightEncoderTicks_actu;
  vit_Right = -vit_Right;
  _RightEncoderTicks_actu = _RightEncoderTicks;

  Serial.print(" Vitesse: ");
  Serial.print(vit_Left);
  Serial.print(" ");
  Serial.println(vit_Right);


  // // encoder_left_pulse_num_speed += pwm_left < 0 ? (-Motor::encoder_count_left_a) : 
  // //                                                 Motor::encoder_count_left_a;
  // // encoder_right_pulse_num_speed += pwm_right < 0 ? (-Motor::encoder_count_right_a) :
  // //                                                 Motor::encoder_count_right_a;

  // // encoder_left_pulse_num_speed += pwm_left_imp < 0 ? (-Motor::encoder_count_left_a) : 
  // //                                                 Motor::encoder_count_left_a;
  // // encoder_right_pulse_num_speed += pwm_right_imp < 0 ? (-Motor::encoder_count_right_a) :
  // //                                                 Motor::encoder_count_right_a;

  // encoder_left_pulse_num_speed += balance_control_output < 0 ? (-Motor::encoder_count_left_a) : 
  //                                                 Motor::encoder_count_left_a;
  // encoder_right_pulse_num_speed += balance_control_output < 0 ? (-Motor::encoder_count_right_a) :
  //                                                 Motor::encoder_count_right_a;

  // // encoder_left_pulse_num_speed += balance_control_output > 0 ? (-Motor::encoder_count_left_a) : 
  // //                                                 Motor::encoder_count_left_a;
  // // encoder_right_pulse_num_speed += balance_control_output > 0 ? (-Motor::encoder_count_right_a) :
  // //                                                 Motor::encoder_count_right_a;

  // left_speed = encoder_left_pulse_num_speed;
  // right_speed = encoder_right_pulse_num_speed;
  // Motor::encoder_count_left_a=0;
  // Motor::encoder_count_right_a=0;
}



void Balanced::Speed_control(int trans, int turn)//pour avancer et tourner en même temps
{
  setting_car_speed = trans;
  setting_turn_speed = turn;
}

void Balanced::Motion_Control(Direction direction)
{
  switch(direction)
  {
    case STOP:
                  Stop();break;
    case FORWARD:
                  Forward(60);break;
    case BACK:
                  Back(60);break;
    case LEFT:
                  Left(75);break;
    case RIGHT:
                  Right(75);break;
    default:      
                  Stop();break;
  }
}

void Balanced::Stop()
{
  setting_car_speed = 0;
  setting_turn_speed = 0;
}

void Balanced::Forward(int speed)
{
  setting_car_speed = speed;
  setting_turn_speed = 0;
}

void Balanced::Back(int speed)
{
  setting_car_speed = -speed;
  setting_turn_speed = 0;
}


void Balanced::Left(int speed)
{
  setting_car_speed = 0;
  setting_turn_speed = speed;
}

void Balanced::Right(int speed)
{
  setting_car_speed = 0;
  setting_turn_speed = -speed;
}

void Balanced::PI_SpeedRing()
{
   double car_speed=(encoder_left_pulse_num_speed + encoder_right_pulse_num_speed) * 0.5;
  //  double car_speed=(encoder_left_pulse_num_speed);
   encoder_left_pulse_num_speed = 0;
   encoder_right_pulse_num_speed = 0;
   speed_filter = speed_filter_old * 0.0 + car_speed * 1.0;
   speed_filter_old = speed_filter;
   car_speed_integeral += speed_filter;
   car_speed_integeral += -setting_car_speed; 
   car_speed_integeral = constrain_val(car_speed_integeral, -200, 200);

   speed_control_output = -kp_speed * speed_filter - ki_speed * car_speed_integeral;
   speed_control_output = constrain_val(speed_control_output, -999, 999);
   speed_control_output = -speed_control_output;

  // speed_control_output = 150;
}

void Balanced::PD_VerticalRing()
{
  double ang_eq = -6.09;
  balance_control_output= kp_balance * (kalmanfilter.angle - ang_eq) + kd_balance * (kalmanfilter.Gyro_x - 0);
  balance_control_output = -balance_control_output; // SIgne vérifié le 26/04/2024 
}

void Balanced::PI_SteeringRing()
{  
   rotation_control_output = setting_turn_speed + kd_turn * kalmanfilter.Gyro_z;////control with Z-axis gyroscope
}


void Mpu6050::init()
{
   Wire.begin();         
   MPU6050.initialize();    
 }

Mpu6050::Mpu6050()
{
    dt = 0.005, Q_angle = 0.001, Q_gyro = 0.005, R_angle = 0.5, C_0 = 1, K1 = 0.05;
}

void Mpu6050::DataProcessing()
{  
  MPU6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);// Data acquisition of MPU6050 gyroscope and accelerometer
  kalmanfilter.Angletest(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1);// Obtaining Angle by Kalman Filter
}



