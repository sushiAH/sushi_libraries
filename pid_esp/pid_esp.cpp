#include <pid_esp.h>

// ---- Config ----
const int PINNUM_POWER[4] = {33,25,27,12};
const int PINNUM_DIR[4] = {32,26,14,13};




void init_pid_pos_esp(const float kp,const float ki,const float kd,
                          const int max_output_pwm,const int max_i_value,
                          const int enc_resolution,
                          const unsigned int pid_period,
                          const int motor_id,
                          pid_pos_esp* p
                          ){

  pos_pid_init(kp,ki,kd,max_output_pwm,max_i_value,&p->PID);
  enc_init(enc_resolution,motor_id,&p->ENC);

  p->pid_period = pid_period;

  p->PINNUM_POWER = PINNUM_POWER[motor_id];
  p->PINNUM_DIR = PINNUM_DIR[motor_id];

  pinMode(PINNUM_POWER[motor_id],OUTPUT);
  pinMode(PINNUM_DIR[motor_id],OUTPUT);
  
  ledcSetup(0,20000,10);  //10bit
  ledcAttachPin(PINNUM_POWER[motor_id],0); 

}



void init_pid_vel_esp(const float kp,const float ki,const float kd,
                          const int max_output_pwm,  
                          const int enc_resolution,
                          const unsigned int pid_period,
                          const int motor_id,
                          pid_vel_esp* p
                          ){
  vel_pid_init(kp,ki,kd,max_output_pwm,&p->PID);
  enc_init(enc_resolution,motor_id,&p->ENC);

  p->pid_period = pid_period;

  p->PINNUM_POWER = PINNUM_POWER[motor_id];
  p->PINNUM_DIR = PINNUM_DIR[motor_id];

  pinMode(PINNUM_POWER[motor_id],OUTPUT);
  pinMode(PINNUM_DIR[motor_id],OUTPUT);

  ledcSetup(0,20000,10);  //10bit
  ledcAttachPin(PINNUM_POWER[motor_id],0); 
}


//if you use esp
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

  ledcWrite(PINNUM_POWER,pwm);
  digitalWrite(PINNUM_DIR,dir);
}




void run_pid_pos(float target,pid_pos_esp* p){
  unsigned int now_time = millis();

  if(now_time - p->pre_time > p->pid_period){
    float pos = update_pos(&p->ENC);
    int pos_pid = calc_pos_pid(target,pos,&p->PID);
    write_to_motor(pos_pid,
                   p->motor_id,p->PINNUM_DIR);

    p->pre_time = now_time;
  }
}

void run_pid_vel(float target,pid_vel_esp* p){
  unsigned int now_time = millis();

  if(now_time - p->pre_time > p->pid_period){
    float vel = update_vel(&p->ENC);
    int vel_pid = calc_vel_pid(target,vel,&p->PID);
    write_to_motor(vel_pid,
                   p->motor_id,p->PINNUM_DIR);

    p->pre_time = now_time;
  }
}





