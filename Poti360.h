#ifndef Poti360_h
#define Poti360_h
#include <Arduino.h>

#define CROSSPOINT1 135.0
#define CROSSPOINT2 315.0
// #define VALID_MIN 0.27
// #define VALID_MAX 0.86
#define VALID_TOLERANCE 0.1
#define VALID_BLEND 0.05
#define HSYSTERESIS 0.15

#define VELOCITY_TOLERANCE 0.15
#define ACCELERATION_TOLERANCE 6.0
#ifndef ADC_MAX

  // --- Arduino UNO / Nano (ATmega328P) ---
  #if defined(ARDUINO_AVR_UNO) || defined(__AVR_ATmega328P__) || defined(ARDUINO_AVR_NANO)
    #define ADC_MAX 1023.0   // 10-bit

  // --- ATtiny classic & modern ---
  #elif defined(__AVR_ATtiny13__) || defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || \
        defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || \
        defined(__AVR_ATtiny84__) || defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny4313__) || \
        defined(__AVR_ATtiny1614__) || defined(__AVR_ATtiny3216__) || defined(__AVR_ATtiny3217__)
    #define ADC_MAX 1023.0   // 10-bit ADCs

  // --- STM8 (Sduino etc.) ---
  #elif defined(STM8S103) || defined(STM8S105) || defined(__STM8__) || defined(__SDCC_STM8__)
    #define ADC_MAX 1023.0   // 10-bit ADC

  // --- STM32F103 ("Blue Pill") ---
  #elif defined(ARDUINO_ARCH_STM32) || defined(STM32F1xx) || defined(ARDUINO_BLUEPILL_F103C8)
    #define ADC_MAX 4095.0   // 12-bit

  // --- ESP32 ---
  #elif defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
    #define ADC_MAX 4095.0   // 12-bit

  // --- ESP8266 ---
  #elif defined(ESP8266)
    #define ADC_MAX 1023.0   // 10-bit

  // --- Fallback ---
  #else
    #warning "ADC_MAX unknown for this target, defaulting to 1023.0"
    #define ADC_MAX 1023.0
  #endif

#endif



class Poti360{
    public:
        Poti360(int pin1, int pin2, uint8_t samples, float valid_min, float valid_max);
        float getPosition();
        float getVelocity();
        float getAcceleration();
        float calibrateCrosspoint1();  //returns the nearest value to crosspoint1
        float calibrateCrosspoint2();  //returns the nearest value to crosspoint2

    private:
        int _pin1;
        int _pin2;
        uint8_t _samples;
        uint8_t _decimals;
        float _valid_min = 0;
        float _valid_max = 0;
        float _lastAngle_degree = 0;
        float _velocity = 0;
        float _acceleration = 0;
        unsigned long _lastMicros = 0;
        float _lastDifference = 1.0;

        float getAngle();
        float angleDiff(float a, float b);
        float roundValue(float value);
        float mapFloat(float x, float xLow, float xHigh, float yLow, float yHigh);
        float normalize(float raw, float vmin, float vmax);
        float oversample(uint8_t pin, uint8_t samples);
};



#endif