#ifndef ESP_UART_H
#define ESP_UART_H

#include <Arduino.h>
#include <stdint.h>





//パケットを受け取る
uint8_t* receive_packet();

//配列のラストバイト以外の全てを足し合わせる
int sum(uint8_t* array,int length);


//配列を0に初期化する
int reset_array(uint8_t* array,int length);

//チェックサムを実行する
int check_sum(uint8_t* recv_data,int data_length);


  


#endif
