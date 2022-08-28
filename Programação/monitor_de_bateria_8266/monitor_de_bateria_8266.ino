#include <Arduino.h>
#include "TTeC.h"


#define tempo 60000
#define motor 5
#define MENOR_TENSAO 3000



/// prototipos de funções 
uint32_t tensao(unsigned char pin);
double Vin_div_T(unsigned char pin, double r1, double r2);
int baixo = 0;

void setup() {
  Serial.begin(115200);
  pinMode(motor,OUTPUT);
  pinMode(2,OUTPUT);

}

void loop() {
  //int inicio=millis();
  digitalWrite(2,0);
  String payload;
  double V_b = Vin_div_T(A0,200,680);    //Tensão em Volts da bateria


  payload = "{\"equipe\":"+String(8266)+",\"bat\":\""+String(V_b)+"mV\"}";

  envia_payload(payload);

  Serial.println(payload);
  if(V_b<=MENOR_TENSAO){
    baixo++;
  }else{
    digitalWrite(motor,0);
  }
  if (baixo>=MENOR_TENSAO){
    digitalWrite(motor,1);
    while(1);
  }
  digitalWrite(2,1);
  delay(tempo);
}

uint32_t tensao(unsigned char pin){

  uint32_t voltage = 0;
    for (int i = 0; i < 100; i++)
    {
      voltage += analogRead(pin);//Obtem o valor RAW do ADC
      ets_delay_us(30);
    }
    voltage /= 100;


    voltage = (3300.00*voltage)/1023.00;//Converte e calibra o valor lido (RAW) para mV
  return voltage;
}
double Vin_div_T(unsigned char pin, double r1, double r2){
  uint32_t tens = tensao(pin);
  double res = (r2/(r1+r2));
  return (tens/res); //tensão/(R2/(R1+R2)) retorna a tensão de entrada do divisor de tensão
}
