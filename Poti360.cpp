#include "Poti360.h"

void Poti360::init(){}

void Poti360::init(int pin1, int pin2, uint8_t samples, float valid_min, float valid_max){
    _pin1 = pin1;
    _pin2 = pin2;
    _samples = samples;
    _valid_min = valid_min;
    _valid_max = valid_max;
}

/**
 * @brief Updates the internal state of the Poti360 object.
 * @details This function should be called periodically to update the internal state of the Poti360 object.
 * It will update the angle and velocity of the potentiometer based on the current readings.
 */
void Poti360::update(){
  // Serial2.println("Update in poti class!");
    getPositionAbsolute();
}


 Poti360::Poti360(){}


/**
 * @brief Constructor for Poti360
 * @param pin1 The first potentiometer pin
 * @param pin2 The second potentiometer pin
 * @param samples The number of samples to take for averaging
 * @param valid_min The minimum valid voltage for the potentiometer
 * @param valid_max The maximum valid voltage for the potentiometer
 */
Poti360::Poti360(int pin1, int pin2, uint8_t samples, float valid_min, float valid_max){
    _pin1 = pin1;
    _pin2 = pin2;
    _samples = samples;
    _valid_min = valid_min;
    _valid_max = valid_max;
}

/**
 * @brief Gets the current angle of the potentiometer in radians.
 * @details This function returns the current angle of the potentiometer
 * in radians. The value is normalized to the range of -PI to PI.
 * @return The current angle of the potentiometer in radians.
 */
float Poti360::getSensorAngle(){
  // float pos = getPosition();
  // Serial2.println(_lastAngle_degree);
  return _lastAngle_degree * PI / 180.0;
}

float Poti360::getMechanicalAngle(){
    return getSensorAngle();
}

float Poti360::getAngle(){
  return _lastAngleAbsolut_degree * PI / 180.0;

}

double Poti360::getPreciseAngle(){
  double _pos = (double) _lastAngleAbsolut_degree;
  return (double) _pos / 180.0 * PI;
}


int32_t Poti360::getFullRotations(){
    return _rotations;
}


/**
 * @brief Gets the current position of the potentiometer in degrees.
 * @details This function returns the current position of the potentiometer
 * in degrees. The value is normalized to the range of 0 to 360 degrees.
 * @return The current position of the potentiometer in degrees.
 */
float Poti360::getPosition() {
    float _angle = getAngleInCircle();  // 0–360°
    unsigned long timedelta = micros() - _lastMicros;
    float timedelta_sec = (float)timedelta / 1e6;
    float delta_angle = 0;

    // --- 1. Überlauf-Logik (vollständige Rotationen erkennen) ---
    if ((_lastAngle_degree > 300.0f) && (_angle < 60.0f)) {
        _rotations++;   // Übergang 360° → 0°
    } else if ((_lastAngle_degree < 60.0f) && (_angle > 300.0f)) {
        _rotations--;   // Übergang 0° → 360° (rückwärts)
    }

    if (_lastAngle_degree > 260.0 && _angle < 60.0) {
        delta_angle = 360.0 + _angle - _lastAngle_degree;
    } else {
        delta_angle = _angle - _lastAngle_degree;
    }

    _velocity = (delta_angle * PI / 180.0) / timedelta_sec; // rad/sec
    _acceleration = _velocity / timedelta_sec;
    
    // Hysterese: Änderung nur bei relevantem Unterschied
    
    float diff = angleDiff(_angle, _lastAngle_degree);
    Serial.printf("diff:%f\t",diff );
    if (fabs(diff) > HSYSTERESIS) {
        _lastAngle_degree = _angle;
        _angle = roundValue(_angle);
        // tue etwas …
         _lastMicros = micros();
        //  Serial2.println(_lastAngle_degree);
        return _angle;
    }else{
      //  Serial2.println(_lastAngle_degree);
       return _lastAngle_degree;
    }
}


float Poti360::getPositionAbsolute(){
    float _angle = getAngleInCircle();  // 0–360°
    unsigned long timedelta = micros() - _lastMicros;
    float timedelta_sec = (float)timedelta / 1e6;
    float delta_angle = 0;

    // --- 1. Überlauf-Logik (vollständige Rotationen erkennen) ---
    if ((_lastAngle_degree > 300.0f) && (_angle < 60.0f)) {
        _rotations++;   // Übergang 360° → 0°
    } else if ((_lastAngle_degree < 60.0f) && (_angle > 300.0f)) {
        _rotations--;   // Übergang 0° → 360° (rückwärts)
    }

    // --- 2. Delta-Berechnung (wie gehabt) ---
    if (_lastAngle_degree > 260.0 && _angle < 60.0) {
        delta_angle = 360.0 + _angle - _lastAngle_degree;
    } else {
        delta_angle = _angle - _lastAngle_degree;
    }

    _velocity = (delta_angle * PI / 180.0f) / timedelta_sec; // rad/s
    _acceleration = _velocity / timedelta_sec;
    float absolute_angle_deg = _rotations * 360.0f + _angle;
    // --- 3. Hysterese ---
    float diff = angleDiff(_angle, _lastAngle_degree);
    if (fabs(diff) > HSYSTERESIS) {
        _lastAngle_degree = _angle;
        _lastAngleAbsolut_degree = absolute_angle_deg;
    }

    _lastMicros = micros();

    // --- 4. Rückgabewert: kombinierter Gesamtwinkel (in Grad) ---
    
    return absolute_angle_deg;
}

float Poti360::getVelocity(){
    if(abs(_velocity) <= VELOCITY_TOLERANCE) return 0.0;

    return roundValue(_velocity);
}


float Poti360::getAngleInCircle() {

  float _angle = -1;
  float _angle1 = 0;
  float _angle2 = 0;

  float raw1 = oversample(_pin1, _samples);
  float raw2 = oversample(_pin2, _samples);
  // Serial.printf("%f,%f\t", raw1, raw2);

  float v1 = normalize(raw1, 0.0, ADC_MAX);
  float v2 = normalize(raw2, 0.0, ADC_MAX);

  bool useV1 = (v1 >= (_valid_min - VALID_BLEND) && v1 <= (_valid_max + VALID_BLEND));
  bool useV2 = (v2 >= (_valid_min - VALID_BLEND) && v2 <= (_valid_max + VALID_BLEND));

  // if(useV1) Serial2.print("*");
  // Serial2.print("V1:");Serial2.print(v1);Serial2.print("\t");
  // if(useV2) Serial2.print("*");
  // Serial2.print("V2:");Serial2.print(v2);Serial2.print("\t");

  int _zone = -1;

  if(useV1){  //use V1
    if(v2 > v1){  //135 ... 225 
      _angle1 = mapFloat(v1, _valid_min, _valid_max, CROSSPOINT2-90.0, CROSSPOINT2);  //4
      _zone = 4;
    }else{ // 45 ... 135
      _angle1 = mapFloat(v1, _valid_max, _valid_min, CROSSPOINT1-90.0, CROSSPOINT1);  //0
      _zone = 0;
    }
  }

  if(useV2){  //use V2
    if(v2 > v1){ // 135 ... 225
      _angle2 = mapFloat(v2, _valid_min, _valid_max, CROSSPOINT1, CROSSPOINT1 + 90.0);  //2
      _zone = 2;
    }else{  // 315 ... 405 (45)
      _angle2 = mapFloat(v2, _valid_max, _valid_min, CROSSPOINT2, CROSSPOINT2 + 90.0);  //6
      if(_angle2 > 360.0) _angle2 = _angle2 - 360.0;
      _zone = 6;
    }
  }

  if(useV1 && useV2 || !useV1 && !useV2){ //blend
    if(v1 < 0.5 && v2 < 0.5){ // blend zone 0/1: 135
      _angle1 = mapFloat(v1, _valid_max, _valid_min, CROSSPOINT1-90.0, CROSSPOINT1);  //Zone 0
      _angle2 = mapFloat(v2, _valid_min, _valid_max, CROSSPOINT1, CROSSPOINT1 + 90.0); //Zone 2
      _zone = 1;
    }else if(v1 < 0.5 && v2 > 0.5){ //blend zone 1/2: 225
      _angle1 = mapFloat(v1, _valid_min, _valid_max, CROSSPOINT2-90.0, CROSSPOINT2);  // zone 2
      _angle2 = mapFloat(v2, _valid_min, _valid_max, CROSSPOINT1, CROSSPOINT1 + 90.0);  //zone 4
      _zone = 3;
    }else if(v1 > 0.5 && v2 > 0.5){ //blend zone 2/3: 315
      _angle1 = mapFloat(v1, _valid_min, _valid_max, CROSSPOINT2-90.0, CROSSPOINT2);  //zone 4
      _angle2 = mapFloat(v2, _valid_max, _valid_min, CROSSPOINT2, CROSSPOINT2 + 90.0);  // zone 6
      if(_angle2 > 360.0) _angle2 = _angle2 - 360.0;
      _zone = 5; 
    }else if(v1 > 0.5 && v2 < 0.5){ //blend zone 3/0: 405/45
      _angle1 = mapFloat(v1, _valid_max, _valid_min, CROSSPOINT1-90.0, CROSSPOINT1);  //zone 0
      _angle2 = mapFloat(v2, _valid_max, _valid_min, CROSSPOINT2, CROSSPOINT2 + 90.0);  // zone 6
      if(_angle2 > 360.0) _angle2 = _angle2 - 360.0;
      _zone = 7;
    }
  }

  switch(_zone){
    case 0:
      _angle = _angle1;
      break;
    case 2:
      _angle = _angle2;
      break;
    case 4:
      _angle = _angle1;
      break;
    case 6:
      _angle = _angle2;
      break;
    case 1: //blend
      _angle = (_angle1 + _angle2) / 2.0;
      break;
    case 3: //blend
      _angle = (_angle1 + _angle2) / 2.0;
    case 5: //blend
      _angle = (_angle1 + _angle2) / 2.0;
      break;
    case 7: //blend
      _angle = (_angle1 + _angle2) / 2.0;
      break;
    default: return -1; //invalid
      break;
  }

  // Serial.printf("%i\t%f\t%f\t",_zone, _angle1, _angle2);
  // Serial2.println(_angle);
  return _angle;
}




float Poti360::calibrateCrosspoint1(){
  float raw1 = oversample(_pin1, _samples);
  float raw2 = oversample(_pin2, _samples);
  float v1 = normalize(raw1, 0.0, ADC_MAX);
  float v2 = normalize(raw2, 0.0, ADC_MAX);

  float _diff = abs(v1-v2);
  if(v1 < 0.5 && v2 < 0.5) return -1;
  if(_diff < _lastDifference){
    _lastDifference = _diff;
    float _v = (v1 + v2) / 2.0;
    // Serial2.printf("V1:%f\tV1:%f\tV:%f\t%f\n", v1, v2, _v, _diff);
    Serial2.print("V1:");Serial2.print(v1, 7);Serial2.print("\tV2:");Serial2.print(v2, 7);Serial2.print("\tV:");Serial2.print(_v,7);Serial2.print("\tDiff:");Serial2.println(_diff, 7);
    return _v;
  }
  return -1;
}

float Poti360::calibrateCrosspoint2(){
  float raw1 = oversample(_pin1, _samples);
  float raw2 = oversample(_pin2, _samples);
  float v1 = normalize(raw1, 0.0, ADC_MAX);
  float v2 = normalize(raw2, 0.0, ADC_MAX);

  float _diff = abs(v1-v2);
  if(v1 > 0.5 && v2 > 0.5) return -1;
  if(_diff < _lastDifference){
    _lastDifference = _diff;
    float _v = (v1 + v2) / 2.0;
    Serial2.print("V1:");Serial2.print(v1, 7);Serial2.print("\tV2:");Serial2.print(v2, 7);Serial2.print("\tV:");Serial2.print(_v,7);Serial2.print("\tDiff:");Serial2.println(_diff, 7);
    return _v;
  }
  return -1;
}



int Poti360::needsSearch() {
    return 0; // default false
}




/**
 * @brief Rounds a float value to the given number of decimal places.
 *
 * @details Rounds a float value to the given number of decimal places.
 * The value is first multiplied by 10 to the power of decimals, then
 * the result is passed to the standard library round function and
 * finally divided by 10 to the power of decimals.
 *
 * @param value The float value to be rounded.
 * @param decimals The number of decimal places to round to.
 * @return The rounded float value.
 */
float Poti360::roundValue(float value) {
  return round(value * 10) / 10;
}

float Poti360::mapFloat(float x, float xLow, float xHigh, float yLow, float yHigh){
  return (x - xLow) * (yHigh - yLow) / (xHigh - xLow) + yLow;
}

float Poti360::normalize(float raw, float vmin, float vmax) {
  // Clipping und Normierung auf 0..1
  if (raw < vmin) raw = vmin;
  if (raw > vmax) raw = vmax;
  return (raw - vmin) / (vmax - vmin);
}

float Poti360::oversample(uint8_t pin, uint8_t samples){
  float sum = 0;
  for(int i = 0; i < samples; i++){
    sum += (float) analogRead(pin);
  }
  return sum / samples;
}

float Poti360::angleDiff(float a, float b) {
  float d = a - b;
  while (d > 180.0) d -= 360.0;
  while (d < -180.0) d += 360.0;
  return d;
}


