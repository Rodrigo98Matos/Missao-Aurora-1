#ifndef ADC_H_
#define ADC_H_

#include <Arduino.h>


double tensao(unsigned char);
double R1(unsigned char);
double R2(unsigned char);
double ntc_10k(unsigned char);
double Vin_div_T(unsigned char pin,double r1,double r2);
double Vin_b2(unsigned char pin, double b1, double r1, double r2);
double porcento(unsigned char pin);




#endif /*ADC_H_ */
