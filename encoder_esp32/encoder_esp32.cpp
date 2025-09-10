#include "encoder_esp32.h"


void enc_init(const int enc_resolution,encoder* p){
  p->enc_resolution = enc_resolution;
}

float update_vel(int16_t now_count,pcnt_unit_t PCNT_UNIT,encoder* p){ 
  unsigned int now_time = millis();
  float dt = (now_time - p->pre_time)/1000.00;
  if(dt == 0 || p->pre_time == 0){
    dt = 0.01;   //10ms
  }

  float diff_count_sec = ((now_count - p->pre_count)/dt);

  float vel = (diff_count_sec/p->enc_resolution);

  p->pre_time = now_time;
  p->pre_count = 0;
  pcnt_counter_clear(PCNT_UNIT);
  p->vel = vel;


  return vel;
}

float update_pos(int16_t now_count,encoder* p){

  float pos = ((double)now_count/p->enc_resolution)*360;  //degree
  p->pos = pos;

  return pos;
}
