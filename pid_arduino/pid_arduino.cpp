#include <pid_arduino.h>




void init_pid_pos_arduino(const float kp,const float ki,const float kd,
                          const int max_output_pwm,const int max_i_value,
                          const int enc_resolution,
                          const unsigned int pid_period,
                          const int PINNUM_POWER,const int PINNUM_DIR,
                          pid_pos_arduino* p
                          ){

  pos_pid_init(kp,ki,kd,max_output_pwm,max_i_value,&p->PID);
  enc_init(enc_resolution,&p->ENC);

  pinMode(p->PINNUM_POWER,OUTPUT);
  pinMode(p->PINNUM_DIR,OUTPUT);

  p->pid_period = pid_period;
  p->PINNUM_POWER = PINNUM_POWER;
  p->PINNUM_DIR = PINNUM_DIR;
}



void init_pid_vel_arduino(const float kp,const float ki,const float kd,
                          const int max_output_pwm,
                          const int enc_resolution,
                          const unsigned int pid_period,
                          const int PINNUM_POWER,const int PINNUM_DIR,
                          pid_vel_arduino* p
                          ){
  vel_pid_init(kp,ki,kd,max_output_pwm,&p->PID);
  enc_init(enc_resolution,&p->ENC);

  p->pid_period = pid_period;
  p->PINNUM_POWER = PINNUM_POWER;
  p->PINNUM_DIR = PINNUM_DIR;
}


//if you use arduino uno or mega
void write_to_motor(int pwm,
                    const int PINNUM_POWER,const int PINNUM_DIR){
  int dir = 0;
  if(pwm > 0){
    dir = 0;
  }
  if(pwm < 0){
    dir = 1;
    pwm = -pwm;
  }

  analogWrite(PINNUM_POWER,pwm);
  digitalWrite(PINNUM_DIR,dir);
}




void run_pid_pos(float target,int16_t now_count,pid_pos_arduino* p){
  unsigned int now_time = millis();

  if(now_time - p->pre_time > p->pid_period){
    float pos = update_pos(now_count,&p->ENC);
    int pos_pid = calc_pos_pid(target,pos,&p->PID);
    write_to_motor(pos_pid,
                   p->PINNUM_POWER,p->PINNUM_DIR);

    p->pre_time = now_time;
  }
}

void run_pid_vel(float target,int16_t* now_count,pid_vel_arduino* p){
  unsigned int now_time = millis();

  if(now_time - p->pre_time > p->pid_period){
    float vel = update_vel(now_count,&p->ENC);
    int vel_pid = calc_vel_pid(target,vel,&p->PID);
    write_to_motor(vel_pid,
                   p->PINNUM_POWER,p->PINNUM_DIR);

    p->pre_time = now_time;
  }
}





