#ifndef PID_ESP_H
#define PID_ESP_H

#include <encoder_esp32.h>
#include <pid_controller.h>
#include <Arduino.h>
#include <stdint.h>

struct pid_pos_esp{
  //private
  unsigned int pid_period;    //millis
  int PINNUM_POWER;
  int PINNUM_DIR;
  int motor_id;
  //public
  unsigned int pre_time;

  encoder ENC;
  pos_pid_controller PID;

};


struct pid_vel_esp{
  //private
  int pid_period;    //millis
  int PINNUM_POWER;
  int PINNUM_DIR;
  int motor_id;

  //public
  unsigned int pre_time;

  encoder ENC;
  vel_pid_controller PID;

};



void init_pid_pos_esp(const float kp,const float ki,const float kd,
                          const int max_output_pwm,const int max_i_value,
                          const int enc_resolution,
                          const unsigned int pid_period,
                          const int motor_id,
                          pid_pos_esp* p
                          );


void init_pid_vel_esp(const float kp,const float ki,const float kd,
                          const int max_output_pwm,
                          const int enc_resolution,
                          const unsigned int pid_period,
                          const int motor_id,
                          pid_vel_esp* p
                          );


void write_to_motor(int pwm,
                    const int PINNUM_POWER,const int PINNUM_DIR);


void run_pid_pos(float target,pid_pos_esp* p);
void run_pid_vel(float target,pid_vel_esp* p);





#endif
