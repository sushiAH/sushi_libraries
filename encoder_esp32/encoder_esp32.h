#ifndef ENCODER_ESP32_H
#define ENCODER_ESP32_H

#include <Arduino.h>
#include <stdint.h>
#include <esp32_pcnt.h>


struct encoder{
  //private
  int enc_resolution; //分解能
  //public
  int pre_count;
  unsigned int pre_time;

  float vel;
  float pos;
};


void enc_init(const int enc_resolution,encoder* p);


float update_vel(int16_t now_count,pcnt_unit_t PCNT_UNIT,encoder* p);
float update_pos(int16_t now_count,encoder* p);



#endif
