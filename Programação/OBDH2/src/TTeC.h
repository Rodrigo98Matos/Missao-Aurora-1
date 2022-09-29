/*
 * OBDH_Aurora_1
 * TTeC.h
 * Created: 12/02/2022 22:25:26
 * Author : Rodrigo Matos
 */ 
#ifndef TTeC_H_
#define TTeC_H_

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>



#define SERVER_IP "http://200.137.130.88:8888/"//link para submissão ao servidor 
/*
//#ifndef STASSID
#define STASSID "aldebaran" //ID do WiFi
#define STAPSK  "21062021"        //Senha do WiFi
*/
//#ifndef STASSID
#define STASSID "AndroidAP498E" //ID do WiFi
#define STAPSK  "dkxt9050"        //Senha do WiFi

//#endif
void envia_payload(String postagem); //Prototipo da funsção
#endif //TTeC_H_