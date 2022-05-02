#include <Arduino.h>
#include "ADC.h"
#include "TTeC.h"

void setup() {
  Serial.begin(9600);
}

void loop() {
  double V_b1 = Vin_div_T(27,2000,6800);
  double T_b1 = ntc_10k(12);
  double V_b2 = Vin_b2(32, V_b1,5100,3300);
  double T_b2 = ntc_10k(14);

  String postagem = "{\"equipe\":2,\"Bateria1\" : ["+String(V_b1)+", "+String(T_b1)+"], \"Bateria2\" : ["+String(V_b2)+", "+String(T_b2)+"]}";

  envia_payload(postagem);

  Serial.println(postagem);

  delay(10000);
}