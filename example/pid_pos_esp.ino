#include <pid_esp.h>
#include <esp32_pcnt.h>
#include <Arduino.h>


void setup(){
  Serial.begin(115200);
}


void loop(){

    


  pid_pos_esp motor1;

  init_pid_pos_esp(10,0,0,
                   1000,1000,
                   1024,
                   10,
                   0,
                   &motor1
                   );

  int target_pos = 0; //degree

  while(1){
    run_pid_pos(target_pos,&motor1);
    Serial.println(motor1.ENC.now_count);
  }
}