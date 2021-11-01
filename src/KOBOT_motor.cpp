#include <KOBOT_motor.h>

/* 
Visual Style Notes:
use space between binary operators
use space after comma
use space after line comment start
use docstring for every class method
 */

KOBOT_motor::KOBOT_motor(  
            const uint8_t directionPin1,   
            const uint8_t directionPin2,
            const uint8_t PWMPin)
{
  _directionPin1 = directionPin1;
  _directionPin2 = directionPin2;
  _PWMPin = PWMPin;
  _PWMPin = PWMPin;
}

/* 
carry out the required, variable and pin 
initializations
*/
void KOBOT_motor::begin()
{
  // set pin modes for the motor driver IC
  pinMode(_directionPin1, OUTPUT);
  pinMode(_directionPin2, OUTPUT);
  pinMode(_PWMPin, OUTPUT);
  
  // initate as low --> disabled
  digitalWrite(_directionPin1, LOW);
  digitalWrite(_directionPin2, LOW);

  // initate as stopped
  stop();

  error_z1 = 0;
  error_z2 = 0;
  error = 0;
  controlEffort = 0;
  controlEffort_z1 = 0;
  measuredVal = 0;
  dutyCycle = 0;
}

void KOBOT_motor::stop() 
{
  analogWrite(_PWMPin, 0);
  
  refVal = 0;
  measuredVal = 0;
  
  error = 0;
  error_z1 = 0;
  error_z2 = 0;

  controlEffort = 0;
  controlEffort_z1 = 0;
}

void KOBOT_motor::resetPID() 
{
  error = 0;
  error_z1 = 0;
  error_z2 = 0;

  // controlEffort = 0;
  // controlEffort_z1 = 0;
}

/* 
Sets the output values for the direction 
and PWM pins
*/
void KOBOT_motor::runMotor(int16_t dutyCycle)
{
  // CCW direction
  if (dutyCycle > 0)
  {
    digitalWrite(_directionPin1, HIGH);
    digitalWrite(_directionPin2, LOW);
  }
  // CW direction
  else if (dutyCycle < 0) 
  {
    digitalWrite(_directionPin1, LOW);
    digitalWrite(_directionPin2, HIGH);
    dutyCycle = -dutyCycle;
  }
  else 
  {
    stop();
    return;
  }
  // saturate the output (if >255)
  if (abs(dutyCycle) > MAX_DUTY_CYCLE)
  {
    dutyCycle = MAX_DUTY_CYCLE;
  }
  analogWrite(_PWMPin, dutyCycle);
}


void KOBOT_motor::PIDController()
{
  // Calculation of discrete controller constants
  b_0 = kp +  kd * f + ki / f;
  b_1 = -kp - 2 * kd * f;
  b_2 = kd * f;

  // control effort [OUTPUT/INPUT]
  controlEffort = (b_0 * error
    + b_1 * error_z1 + b_2 * error_z2)*TICK_2_RAD;
  // sum with the prev. value of the effort
  feedForwardCompensation();
  controlEffort += controlEffort_z1;

  // // clamp the output if necessary
  // if (abs(controlEffort) > MAX_EFFORT)
  // {
  //   if (controlEffort > 0)
  //   {
  //     controlEffort = MAX_EFFORT;
  //   }
  //   else if (controlEffort < 0)
  //   {
  //     controlEffort = -MAX_EFFORT;
  //   }
  //   // return without changing any values
  //   return;
  // }

  // control voltage that is one period bacward shifted
  controlEffort_z1 = controlEffort;
  // error that is 2 period backward shifted 
  error_z2 = error_z1;
  // error that is 1 period backward shifted 
  error_z1 = error;
}

/*
Position control loop : VOLTAGE/TICKS
calculates the output dutyCycle with the 
defined control law current refVal and 
current measuredVal at a constant sampling rate
*/
void KOBOT_motor::positionController() 
{
  // Calculate the position [TICKS]
  measuredVal = myEnc->read();
  // Calculate the position error
  error = (refVal - measuredVal);
  if (abs(error) < 8)
  {
    controlEffort = 0;
    controlEffort_z1 = 0;
  }
  // Feed the position error to calculate the control effort
  PIDController();
  dutyCycle = int(controlEffort * VOLTAGE_2_DUTY_CYCLE);
  // run motor with the calculated duty cycle
  runMotor(dutyCycle);
}

/*
Velocity control loop : VOLTAGE/TICKS_PER_SECOND
calculates the output dutyCycle with the defined 
control law current refVal and current measuredVal 
at a constant sampling rate
*/
void KOBOT_motor::velocityController()
{
  // feedForwardCompensation();

  // // limit the min. value for velocity
  // if (abs(refVal) < 500)
  // {
  //   stop();
  // }

  // Calculate the velocity of the wheel [TICKS/SEC]
  // measure velocity by downsampling

  measureAverageVelocity();
  // rampInput();
  // Calculate the velocity error 
  error = (refVal - measuredVal);
  // Serial.println(error);
  // Feed the velocity error to calculate the control effort
  PIDController();
  dutyCycle = int(controlEffort * VOLTAGE_2_DUTY_CYCLE);
  // run motor with the calculated duty cycle
  runMotor(dutyCycle);
}

/*
Velocity control loop : VOLTAGE/TICKS_PER_SECOND
calculates the output dutyCycle with the defined 
control law current refVal and current measuredVal 
at a constant sampling rate
*/
void KOBOT_motor::OLVelocityController()
{

  // limit the min. value for velocity
  if (abs(refVal) < 500)
  {
    stop();
  }
  // refVal is in terms of Ticks Per Second
  // map it to 8-bit dutyCycle
  dutyCycle = map(refVal, 6000, -6000, -255, 255);
  // run motor with the calculated duty cycle
  runMotor(dutyCycle);
}



/*
Turn off controller if error is in the desired window
*/
void KOBOT_motor::checkErrorWindow()
{
  if (abs(error) < ERROR_WINDOW_TRESH)
  {
    error = 0;
    controlEffort = controlEffort_z1;
  }
}

/*
Compensate for constant disturbance by
changing the controlEffort
*/
void KOBOT_motor::feedForwardCompensation()
{
  if (refVal > 0)
  {
    controlEffort += ffVal;
  } 
  else if (refVal < 0)
  {
    controlEffort -= ffVal;
  } 
}


/*
If step input slope is too high convert it to ramp
*/
void KOBOT_motor::rampInput()
{
  if (abs(targetVal - refVal) > REF_VAL_SLOPE)
  {
    if (targetVal - refVal > 0) 
    {
      refVal = refVal + REF_VAL_SLOPE;
    }
    else if (targetVal - refVal < 0) 
    {
      refVal = refVal - REF_VAL_SLOPE;
    }
  }
  else
  {
    refVal = targetVal;
  }
}

/*
Set encoder object from an external class
*/
void KOBOT_motor::setEncoder(Encoder& encoder)
{
  myEnc = &encoder;
}

long KOBOT_motor::readEncoder()
{
  return myEnc->read();
}

void KOBOT_motor::resetEncoder()
{
  myEnc->write(0);
}

/*
Measure velocity of the wheel in TICK_PER_S
by using average of numSamples measurements
*/
void KOBOT_motor::measureAverageVelocity()
{
  static long prevPos;
  static long prevTime;

  long currentPos = myEnc->read();
  int8_t posDiff = currentPos - prevPos;

  if (abs(posDiff)>=0)
  {
    // use micros for high velocity resoln.
    long currentTime = micros();
    long deltaT = currentTime - prevTime;
    // TICK_PER_SEC
    int16_t currentVal = int(1000000 * float(posDiff)/deltaT);
    // movingAverageFilter(currentVal);
    measuredVal = currentVal;
    prevPos = currentPos;
    prevTime = currentTime;
    measuredVal = movingAverageFilter(measuredVal);
  }
}


/*
Filter measured value by using a weighted sum of the 
current value and previous value
*/
int16_t KOBOT_motor::movingAverageFilter(int16_t currentVal)
{
  const float filterRatio = 0.2;
  static int16_t prevVal;

  measuredVal = int(float(prevVal) * (1.0-filterRatio) 
    + float(currentVal) * filterRatio);
  prevVal = measuredVal;
  return measuredVal;

}

