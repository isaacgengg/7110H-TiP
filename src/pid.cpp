#include "vex.h"
#include "pid.h"
#include <cmath>

PID::PID(double kP, double kD) {
m_kP=(kP), m_kD=(kD);
}

double PID::calculateErr(double ierror) {
  m_error = ierror;
  
  // //calculate delta time
  // double dT = m_timer.millis().convert(millisecond) - m_lastTime;
  // //abort if dt is too small
  // if(dT < m_minDt) return m+output;

  //calculate derivative
  m_derivative = (m_error - m_lastError);

  //calculate output
  m_output = (m_error * m_kP) + (m_derivative * m_kD);


  // if(m_output > 10) {
  // m_output = 10;
  // }
  
  // if(m_output < -10) {
  // m_output = -10;
  // }

  //save values
  // m_lastTime = m_timer.millis().convert(millisecond);
  m_lastError = m_error;
  
  return m_output;
}

double PID::calculate(double target, double current) {
  return calculateErr(target - current);
}

double PID::getError() {
  return m_error;
}

void PID::reset() {
  m_error = 0;
  m_lastError = 0;
  m_derivative = 0;
  m_output = 0;
}
