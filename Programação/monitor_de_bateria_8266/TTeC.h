/*
 * OBDH_Aurora_1
 * TTeC.h
 * Created: 12/02/2022 22:25:26
 * Author : Rodrigo Matos
 */ 
#ifndef TTeC_H_
#define TTeC_H_

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>



#define SERVER_IP "http://200.137.130.88:8888/"//link para submissão ao servidor 

//#ifndef STASSID
#define STASSID "ALBERTO E ROSELIA" //ID do WiFi
#define STAPSK  "CHAMEX5050"        //Senha do WiFi
//#endif
void envia_payload(String postagem); //Prototipo da funsção
#endif //TTeC_H_
