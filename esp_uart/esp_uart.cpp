#include <esp_uart.h>


//配列のラストバイト以外の全てを足し合わせる
int sum(uint8_t* array,int length){
  int sum_value = 0;
  for(int i=0;i<length-1;i++){
    sum_value = sum_value + array[i];
  }
  return sum_value;
}


//配列を0に初期化する
int reset_array(uint8_t* array,int length){
  for(int i=0;i<length;i++){
    array[i] = 0;
  }
}

//チェックサムを実行する
int check_sum(uint8_t* recv_data,int length){
  int sum_value = sum(recv_data,length);
  if((sum_value % 256) == (recv_data[length-1] % 256)){
    
    return 0; //正常終了
  }
  else{
    return 1; //異常終了
  }
  
}


uint8_t* receive_packet(){
  int header = 0xAA;
  int recv_header = 0;
  int recv_packet_length= 0;   

  const int recv_period = 5;
  static unsigned int pre_time = 0;

  unsigned int now_time = millis();
  if(now_time - pre_time >= recv_period){
    if(Serial.available() > 0){
      recv_header = Serial.read();
      if(recv_header == header){
        recv_packet_length = Serial.read();  //9が送られる
        uint8_t* packet_data = (uint8_t*)malloc(recv_packet_length);
        if(packet_data == NULL){
          free(packet_data);
          return NULL;
        }

        packet_data[0] = recv_header;
        packet_data[1] = recv_packet_length;

        for(int i=2;i<recv_packet_length;i++){
          packet_data[i] = Serial.read();
        }

        //チェックサム
        if(check_sum(packet_data,recv_packet_length) == 0){
          //正常終了
          return packet_data;
        }
        else{ 
          //異常終了
          free(packet_data);
          return NULL;
        }
      }
    }
    pre_time = now_time;
    return NULL;
  }
  return NULL;
}




  


