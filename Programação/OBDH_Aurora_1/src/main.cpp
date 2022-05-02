#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "ADC.h"

#include <WiFiMulti.h>



#define SERVER_IP "200.137.130.88:8888"

WiFiMulti wifiMulti;

void setup() {
  Serial.begin(9600);
  /*wifiMulti.addAP("ALBERTO E ROSELIA", "CHAMEX5050");
  wifiMulti.addAP("aldebaran", "21062021");
  wifiMulti.addAP("Rodrigo Matos", "aero2019");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());*/
}

void loop() {
  double V_b1 = Vin_div_T(27,2000,6800);
  double T_b1 = ntc_10k(12);
  double V_b2 = Vin_b2(32, V_b1,5100,3300);
  double T_b2 = ntc_10k(14);

 /* if ((wifiMulti.run() == WL_CONNECTED)) {
    WiFiClient client;
    HTTPClient http;

    Serial.print("[HTTP] begin...\n");
    // configure traged server and url
    http.begin(client, "http://200.137.130.88:8888"); //HTTP
    http.addHeader("Content-Type", "application/json");

    Serial.print("[HTTP] POST...\n");
    // start connection and send HTTP header and body
    int httpCode = http.POST("{\"equipe\":1,\"Bateria1\" : ["+String(V_b1)+", "+String(T_b1)+"], \"Bateria2\" : ["+String(V_b2)+", "+String(T_b2)+"]}");

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
*/

  Serial.println("{\"Bateria1\" : ["+String(V_b1)+", "+String(T_b1)+"], \"Bateria2\" : ["+String(V_b2)+", "+String(T_b2)+"]}");
  delay(5000);
}