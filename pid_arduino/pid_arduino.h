#ifndef PID_ARDUINO_H
#define PID_ARDUINO_H

#include <encoder.h>
#include <pid_controller.h>
#include <Arduino.h>
#include <stdint.h>

struct pid_pos_arduino{
  //private
  unsigned int pid_period;    //millis
  int PINNUM_POWER;
  int PINNUM_DIR;
  //public
  unsigned int pre_time;

  encoder ENC;
  pos_pid_controller PID;

};


struct pid_vel_arduino{
  //private
  int pid_period;    //millis
  int PINNUM_POWER;
  int PINNUM_DIR;
  //public
  unsigned int pre_time;

  encoder ENC;
  vel_pid_controller PID;

};



void init_pid_pos_arduino(const float kp,const float ki,const float kd,
                          const int max_output_pwm,const int max_i_value,
                          const int enc_resolution,
                          const unsigned int pid_period,
                          const int PINNUM_POWER,const int PINNUM_DIR,
                          pid_pos_arduino* p
                          );


void init_pid_vel_arduino(const float kp,const float ki,const float kd,
                          const int max_output_pwm,
                          const int enc_resolution,
                          const unsigned int pid_period,
                          const int PINNUM_POWER,const int PINNUM_DIR,
                          pid_vel_arduino* p
                          );


void write_to_motor(int pwm,
                    const int PINNUM_POWER,const int PINNUM_DIR);


void run_pid_pos(float target,int16_t now_count,pid_pos_arduino* p);
void run_pid_vel(float target,int16_t* now_count,pid_vel_arduino* p);





#endif
