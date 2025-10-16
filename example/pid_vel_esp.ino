#include <pid_esp.h>
#include <esp32_pcnt.h>
#include <Arduino.h>


void setup(){
  Serial.begin(115200);
}


void loop(){


  pid_vel_esp motor1;

  init_pid_vel_esp(200,8000,0,
                   1000,
                   1024,
                   10,
                   0,
                   &motor1
                   );

  float target_vel = 0.0; //degree

  int16_t ENC_COUNT_0 = 0;

  while(1){
    run_pid_vel(target_vel,&motor1);
    Serial.println(motor1.ENC.now_count);
  }
}
