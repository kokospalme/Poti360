// #include "myDevice.h"
#include <Poti360.h>

#define PIN_POTA1 0
#define PIN_POTA2 1
#define SAMPLES 30
#define ADC_MAX  4095.0
#define VALID_MIN 0.258547
#define VALID_MAX 0.842324

Poti360 poti(PIN_POTA1,PIN_POTA2, SAMPLES, VALID_MIN, VALID_MAX);
float angle = 0;
float velocity = 0;
float acceleration = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Poti360 example. angle:");
  analogReadResolution(12);              // 12 Bit (0..4095)
  analogSetAttenuation(ADC_11db);        // ~0..3.3 V
}

void loop() {
  angle = poti.getPosition(); //angle
  velocity = poti.getVelocity(); //velocity
  acceleration = poti.getAcceleration();  //acceleration


  Serial.printf("%f\t%f\t%f\n", angle, velocity, acceleration);
  delay(50);
}





