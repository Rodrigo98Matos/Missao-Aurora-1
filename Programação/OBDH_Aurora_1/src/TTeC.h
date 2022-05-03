#ifndef TTeC_H_
#define TTeC_H_

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>



#define SERVER_IP "200.137.130.88:8888"

//#ifndef STASSID
#define STASSID "ALBERTO E ROSELIA"
#define STAPSK  "CHAMEX5050"
//#endif
void envia_payload(String postagem);
#endif //TTeC_H_