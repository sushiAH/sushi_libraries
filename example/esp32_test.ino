#include <esp_uart.h>
#include <pid_esp.h>



void setup(){

  Serial.begin(115200,SERIAL_8N1,-1,-1,false,2048,2048);

  qei_setup_x1(PCNT_UNIT_0,GPIO_NUM_23,GPIO_NUM_22);
  qei_setup_x1(PCNT_UNIT_1,GPIO_NUM_23,GPIO_NUM_22);
  qei_setup_x1(PCNT_UNIT_2,GPIO_NUM_23,GPIO_NUM_22);
  qei_setup_x1(PCNT_UNIT_3,GPIO_NUM_23,GPIO_NUM_22);

  pinMode(33,OUTPUT);
  pinMode(32,OUTPUT);

  ledcSetup(0,20000,10);  //10bit
  ledcAttachPin(32,0); 

}


void loop(){

  int16_t ENC_COUNT_0;
  int16_t ENC_COUNT_1;
  int16_t ENC_COUNT_2;
  int16_t ENC_COUNT_3;

  pid_pos_esp motor_pos1;
  pid_pos_esp motor_pos2;
  pid_pos_esp motor_pos3;
  pid_pos_esp motor_pos4;

  init_pid_pos_esp(2,1,0,
                   1000,1000,
                   1024,
                   10,
                   33,32,
                   &motor_pos1
                   );

  init_pid_pos_esp(2,1,0,
                   1000,1000,
                   1024,
                   10,
                   33,32,
                   &motor_pos2
                   );

  init_pid_pos_esp(2,1,0,
                   1000,1000,
                   1024,
                   10,
                   33,32,
                   &motor_pos3
                   );

  init_pid_pos_esp(2,1,0,
                   1000,1000,
                   1024,
                   10,
                   33,32,
                   &motor_pos4
                   );
  
  pid_vel_esp motor_vel1;
  pid_vel_esp motor_vel2;
  pid_vel_esp motor_vel3;
  pid_vel_esp motor_vel4;

  init_pid_vel_esp(2,1,0,
                   1000,
                   1024,
                   10,
                   33,32,
                   &motor_vel1
                   );

  init_pid_vel_esp(2,1,0,
                   1000,
                   1024,
                   10,
                   33,32,
                   &motor_vel2
                   );

  init_pid_vel_esp(2,1,0,
                   1000,
                   1024,
                   10,
                   33,32,
                   &motor_vel3
                   );

  init_pid_vel_esp(2,1,0,
                   1000,
                   1024,
                   10,
                   33,32,
                   &motor_vel4
                   );

  int motor_status[4] = {0}; // 0 is vel 1 is pos
  int32_t motor_target[4] = {0};

  

  while(1){
    // receive_packet 
    uint8_t* packet_data = receive_packet();

    //update_motor_status and motor_target
    if(packet_data != NULL){
      motor_status[packet_data[2]] = packet_data[3];
      motor_target[packet_data[2]] = packet_data[4]<<24 | packet_data[5]<<16 | packet_data[6]<<8 | packet_data[7] ;  //4から7
      free(packet_data);
    }

    
    //write_to_motor
    if(motor_status[0] == 0){
      run_pid_pos(motor_target[0],ENC_COUNT_0,&motor_pos1);
      pcnt_get_counter_value(PCNT_UNIT_0,&ENC_COUNT_0);
    }
    else if(motor_status[0] == 1){
      run_pid_vel(motor_target[0],ENC_COUNT_0,PCNT_UNIT_0,&motor_vel1);
      pcnt_get_counter_value(PCNT_UNIT_0,&ENC_COUNT_0);
    }

    Serial.println(motor_target[0]);





 //   if(motor_status[1] == 0){
 //     run_pid_pos(motor_target[1],now_count,pid_pos_esp* motor2);
 //     pcnt_get_counter_value(PCNT_UNIT_1,&ENC_COUNT_1);
 //   }
 //   else if(motor_status[1] == 1){
 //     run_pid_vel(motor_target[1],now_count,pid_pos_esp* motor2);
 //     pcnt_get_counter_value(PCNT_UNIT_1,&ENC_COUNT_1);
 //   }


 //   if(motor_status[2] == 0){
 //     run_pid_pos(motor_target[2],now_count,pid_pos_esp* motor3);
 //     pcnt_get_counter_value(PCNT_UNIT_2,&ENC_COUNT_2);
 //   }
 //   else if(motor_status[2] == 1){
 //     run_pid_vel(motor_target[2],now_count,pid_pos_esp* motor3);
 //     pcnt_get_counter_value(PCNT_UNIT_2,&ENC_COUNT_2);
 //   }


 //   if(motor_status[3] == 0){
 //     run_pid_pos(motor_target[3],now_count,pid_pos_esp* motor4);
 //     pcnt_get_counter_value(PCNT_UNIT_3,&ENC_COUNT_3);
 //   }
 //   else if(motor_status[3] == 1){
 //     run_pid_vel(motor_target[3],now_count,pid_pos_esp* motor4);
 //     pcnt_get_counter_value(PCNT_UNIT_3,&ENC_COUNT_3);
 //   }

  

  }
}
