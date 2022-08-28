/*
 * OBDH_Aurora_1
 * TTeC.cpp
 * Created: 12/02/2022 22:25:26
 * Author : Rodrigo Matos
 */ 
#include "TTeC.h"

void envia_payload(String postagem){//Envia uma string ao servidor 
WiFi.begin(STASSID, STAPSK);
int cont = 0;
  while (WiFi.status() != WL_CONNECTED && cont<120) {//se não conectar depois de 1 minuto, desiste de conectar
    delay(500);cont++;
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());

  if ((WiFi.status() == WL_CONNECTED)) {
    WiFiClient client;
    HTTPClient http;
    Serial.print("[HTTP] begin...\n");
    // configure traged server and url
    http.begin(client,SERVER_IP); //HTTP
    http.addHeader("Content-Type", "application/json");

    Serial.print("[HTTP] POST...\n");
    // start connection and send HTTP header and body
   int httpCode = http.POST(postagem);
 
    // httpCode will be negative on error
    if (httpCode > 0) {
      // HTTP header has been send and Server response header has been handled
      Serial.printf("[HTTP] POST... code: %d\n", httpCode);

      // file found at server
      if (httpCode == HTTP_CODE_OK) {
        const String& payload = http.getString();
        Serial.println("received payload:\n<<");
        Serial.println(payload);
        Serial.println(">>");
      }
    } else {
      Serial.printf("[HTTP] POST... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }

    http.end();
  }
  

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}
