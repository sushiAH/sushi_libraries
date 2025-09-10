#include <pid_esp.h>
#include <esp32_pcnt.h>
#include <Arduino.h>


void setup(){
  Serial.begin(115200);
  qei_setup_x1(PCNT_UNIT_0,GPIO_NUM_22,GPIO_NUM_23);
  

  pinMode(33,OUTPUT);
  pinMode(32,OUTPUT);

  ledcSetup(0,20000,10);  //10bit
  ledcAttachPin(32,0); 

}


void loop(){

    


  pid_vel_esp motor1;

  init_pid_vel_esp(200,8000,0,
                   1000,
                   1024,
                   10,
                   0,33,
                   &motor1
                   );

  float target_vel = 0.0; //degree

  int16_t ENC_COUNT_0 = 0;

  while(1){
    char c = Serial.read();
    if(c == 'a'){
      Serial.print(c);
      Serial.print("");
      target_vel = target_vel + 0.1;
      Serial.println(target_vel);
    } 
    if(c == 'b'){
      Serial.print(c);
      Serial.print("");
      target_vel = target_vel - 0.1;
      Serial.println(target_vel);
    }
    pcnt_get_counter_value(PCNT_UNIT_0,&ENC_COUNT_0);
    run_pid_vel(target_vel,ENC_COUNT_0,PCNT_UNIT_0,&motor1);
  }
}
