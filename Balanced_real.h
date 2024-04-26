#ifndef _BALANCED_h
#define _BALANCED_h

#include "MsTimer2.h"
#include "KalmanFilter.h"

enum Direction
{
  FORWARD,
  BACK,
  LEFT,
  RIGHT,
  STOP,
};

class Balanced
{
  public:

          
          Balanced();

          void Encoder_init();
          
          void Get_EncoderSpeed();
          void PD_VerticalRing();
          void PI_SpeedRing();
          void PI_SteeringRing();
          void Total_Control();
          
          void Motion_Control(Direction direction);
          void Stop();
          void Forward(int speed);
          void Back(int speed);
          void Left(int speed);
          void Right(int speed);
          void Speed_control(int trans, int turn);
  
/*Speed value*/
          double pwm_left=0.0;
          double pwm_right=0.0;
          double pwm_left_imp=0.0;
          double pwm_right_imp=0.0;
          int encoder_left_pulse_num_speed;
          int encoder_right_pulse_num_speed;

          int left_speed;
          int right_speed;

/*Cnt*/
          int interrupt_cnt;

/*PID parameter*/
         /*PD_VerticalRing*/
          double kp_balance, kd_balance;
         /*PI_SpeedRing*/
          double kp_speed, ki_speed;
         /*PI_SteeringRing*/
          double kp_turn, kd_turn;
          double offset_orientation;

          double speed_filter;
          double speed_filter_old;
          double car_speed_integeral = 0.0;
          double balance_control_output;
          double speed_control_output;
          double rotation_control_output;
          int setting_turn_speed;
          int setting_car_speed;
          int test_interrupt=0;
          bool arret_moteur;

          int cmd_sens=0;

          
   private:
   #define ANGLE_MIN -60 //27
   #define ANGLE_MAX 60 //27
   #define EXCESSIVE_ANGLE_TILT (kalmanfilter.angle < ANGLE_MIN || ANGLE_MAX < kalmanfilter.angle)
   #define PICKED_UP (kalmanfilter.angle6 < -10 || 22 < kalmanfilter.angle6)
};

class Timer2
{
  public:
          void init(int time);
          static void interrupt();
          #define TIMER 5
  private:       
         
};


class Mpu6050
{
  public:
          void init();
          void DataProcessing();
          Mpu6050();

  public:
         int ax, ay, az, gx, gy, gz;
         float dt, Q_angle, Q_gyro, R_angle, C_0, K1;
};












#endif
