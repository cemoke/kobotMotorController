#include <Wire.h>
#include <KOBOT_motor.h>
// #define DEBUG

// Since this value is used in ISR
// define as volatile
volatile bool timerInterruptFlag = false;

KOBOT_motor motor(6, 7, 5);

// Use Interrupt pins for encoder ticks
Encoder myEnc(2, 3);

uint8_t processId = 0;
int8_t measured_vel = 0;

/**
  Returns float32 value read as 4 seperate bytes from
  the I2C bus
  @return float32 val read from I2C bus
*/
float parseFloatValues()
{
  // as first item number of bytes are sent
  // read that value
  uint8_t howMany = Wire.read();
  byte data[4];

  // collect all the bytes
  for (uint8_t i = 0; i < howMany; i++)
  {
    data[i] = Wire.read();
  }

  /*   
  A union is an "overlay" of variables,
  sharing the same memory cells.
  In this case it is used as a re-interpret-cast. 
  */
  union floatTag
  {
    byte b[4];
    float fVal;
  } ft;

  ft.b[0] = data[0];
  ft.b[1] = data[1];
  ft.b[2] = data[2];
  ft.b[3] = data[3];

  return ft.fVal;
}

/**
  Returns int16 value read as 2 seperate bytes from
  the I2C bus
  @return int16 val read from I2C bus
*/
int16_t parseInt16Values()
{
  // as first item, number of bytes are sent
  // read that value
  uint8_t howMany = Wire.read();
  byte data[2];
  // collect all the bytes
  for (uint8_t i = 0; i < howMany; i++)
  {
    data[i] = Wire.read();
  }

  /*   
  A union is an "overlay" of variables, 
  sharing the same memory cells.
  In this case it is used as a re-interpret-cast. 
  */
  union int16_tTag 
  {
    byte b[2];
    uint16_t intVal;
  } it;

  it.b[0] = data[0];
  it.b[1] = data[1];

  return it.intVal;
}

/**
  Write int32 value as 4 seperate bytes to
  the I2C bus
  @param int32 val to be sent to I2C bus
*/
void composeInt32Msg(int32_t longVal)
{
  byte msgArray[4];
  // seperate 32-bit value to 4 bytes
  msgArray[0] = (longVal >> 24) & 0xFF;
  msgArray[1] = (longVal >> 16) & 0xFF;
  msgArray[2] = (longVal >> 8) & 0xFF;
  msgArray[3] = (longVal)&0xFF;
  Wire.write(msgArray, 4);
}

/**
  Write int16 value as 2 seperate bytes to
  the I2C bus
  @param int16 val to be sent to I2C bus
*/
void composeInt16Msg(int16_t intVal)
{
  byte msgArray[2];
  // seperate 16-bit value to 2 bytes
  msgArray[0] = (intVal >> 8) & 0xFF;
  msgArray[1] = intVal & 0xFF;
  Wire.write(msgArray, 2);
}

void setTimerInterrupt(float samplingFreq)
{
  // disable interrupts
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  long compareMatchVal;
  // find compare match value
  // use 16 mHz from timer1
  compareMatchVal = 16000000 / (samplingFreq * TIMER_PRESCALAR) - 1;
  OCR1A = compareMatchVal;
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12);
  TIMSK1 |= (1 << OCIE1A);
  // enable interrupts
  sei();
}

/**
  Master performing write operation
  determine the variable which is going 
  to be written by the processId
  @param int howMany val to be sent to I2C bus
*/
void receiveEvent(int howMany)
{
  // what to do with the incoming msg is determined
  // by its first byte (register value)
  processId = Wire.read();
  switch (processId)
  {
    case 0:
    {
      int16_t tmp = parseInt16Values();
      if (tmp > 10000)
      {
        motor.refVal = 10000 - tmp;
      }
      else
      {
        motor.refVal = tmp;
      }
    #ifdef DEBUG
      Serial.println("ref : ");
      Serial.println(motor.refVal);
    #endif
      break;
    }
    case 1:
    {
      motor.kp = parseFloatValues();
    #ifdef DEBUG
      Serial.println("Kp : ");
      Serial.println(motor.kp);
    #endif
      break;
    }
    case 2:
    {
      motor.ki = parseFloatValues();
    #ifdef DEBUG
      Serial.println("Ki : ");
      Serial.println(motor.ki);
    #endif
      break;
    }
    case 3:
    {
      motor.kd = parseFloatValues();
    #ifdef DEBUG
      Serial.println("Kd : ");
      Serial.println(motor.kd);
    #endif
      break;
    }
    case 4:
    {
      motor.f = parseFloatValues();
      setTimerInterrupt(motor.f);
    #ifdef DEBUG
      Serial.println("f : ");
      Serial.println(motor.f);
    #endif
      break;
    }
    default:
      break;
  }
  // flush the i2c input buffer
  while (Wire.available() > 0)
  {
    processId = Wire.read();
  }
}

/**
  Master performing read operation
  determine the variable which is going 
  to be read by the processId
*/
void requestEvent()
{
  // what to do with the incoming msg is determined
  // by its first byte (register value)
  switch (processId)
  {
  case 255:
  {
    composeInt16Msg(motor.measuredVal);
    break;
  }
  case 254:
  {
    int16_t intVal = analogRead(A6);
    composeInt16Msg(intVal);
    break;
  }
  default:
    break;
  }
}

void setup()
{
  #ifdef DEBUG
    Serial.begin(115200);
  #endif
  // When facing camera the one on the left is 8
  Wire.begin(8);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  motor.begin();
  motor.setEncoder(myEnc);
  // set default vals
  motor.f = 100;
  motor.kp = 0.2;
  motor.ki = 5.0;
  motor.kd = 0.01;
  motor.mode = 0;
  #ifdef DEBUG
    // motor.refVal = 2000;
  #endif
  // set timer interrupt on
  setTimerInterrupt(motor.f);
}

void loop()
{
  // if interrupt flag is risen
  if (timerInterruptFlag)
  {
    #ifdef DEBUG
      // Serial.println("Measured Val : ");
      // Serial.println(motor.measuredVal);
      // Serial.println("Encoder Val : ");
      // Serial.println(motor.readEncoder());
    #endif
    // choose controller according to mode
    switch (motor.mode)
    {
    case 0:
    {
      motor.velocityController();
      break;
    }
    case 1:
    {
      motor.positionController();
      break;
    case 2:
    {
      motor.OLVelocityController();
      break;
    }
    }
    default:
      break;
    }
    // lower interrupt flag
    timerInterruptFlag = false;
  }
}

ISR(TIMER1_COMPA_vect)
{
  // rise interrupt flag
  timerInterruptFlag = true;
}