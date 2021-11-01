#ifndef KOBOT_motor_H
#define KOBOT_motor_H

#include <Arduino.h>
#include <Encoder.h>

#define TIMER_PRESCALAR 256

// corresponds to 3V where 255 -> 5V
#define MAX_DUTY_CYCLE 255

// 255 -> 5V, 1V -> 50
#define VOLTAGE_2_DUTY_CYCLE 51

// 2*pi/(gearRatio*encoderRes*4)
// 2*pi/(112*8*4)
// 900 count 
#define TICK_2_RAD  0.00698
#define REF_VAL_SLOPE 100
// 1200 count 
// #define TICK_2_RAD  0.00523

#define MAX_EFFORT 255

#define ERROR_WINDOW_TRESH 20


class KOBOT_motor 
{
  public:
    KOBOT_motor(
                const uint8_t directionPin1,   
                const uint8_t directionPin2,
                const uint8_t PWMPin); 

    void        setEncoder(Encoder& encoder);
    void        begin();
    void        resetPID();
    void        positionController();
    void        velocityController();
    void        OLVelocityController();

    void        runMotor(int16_t dutyCycle);
    void        stop();
    void        PIDController();
    void        feedForwardCompensation();
    void        measureAverageVelocity();
    void        checkErrorWindow();
    void        rampInput();
    int16_t     movingAverageFilter(int16_t currentVal);
    long        readEncoder();
    void        resetEncoder();

    int16_t     refVal;
    int16_t     targetVal;
    // int16_t dutyCycle to be able to repr. [-255, 255]
    int16_t     dutyCycle;
    int16_t     error, error_z1, error_z2;
    float       ffVal;
    uint8_t     mode;

    // PID Constants
    float       b_0,b_1,b_2;
    float       kp, ki, kd, f;
    float       controlEffort, controlEffort_z1;
    int16_t     measuredVal;

    Encoder*    myEnc;

  private:
    uint8_t     _directionPin1, _directionPin2, _PWMPin;
};
#endif