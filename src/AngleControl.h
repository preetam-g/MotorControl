#ifndef MOTORCONTROL_ANGLECONTROL_H
#define MOTORCONTROL_ANGLECONTROL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#include <Encoder.h>
#include "Motor.h"
#endif 

class AngleControl 
{

public:
    AngleControl(Motor& motor, Encoder& encoder, double& desAngle)
        : motor(motor), encoder(encoder), desired(desAngle) {}

    void setDesired(double desAngle) 
    {
        this->desired = desAngle;
    }

    double getAngle()
    {
        return this->currAngle;
    }

    void getPos()
    {
        return this->encoder.read();
    }

    void printErrors()
    {
        Serial.print(error);
        Serial.print(",");
        Serial.print(integral);
        Serial.print(",");
        Serial.print(derivative);
    }

    int16_t compute()
    {
        this->calculateAngle();
        this->pid();
        return this->pwmOut;
    }

    int16_t retain()
    {
        this->calculateAngle();
        return this->pwmOut;
    }


private:
    // motor and encoder
    Motor& motor;
    Encoder& encoder;

    // pid variables
    double error = 0, prevError = 0;
    double integral = 0;
    double derivative = 0;

    // angle variables
    double desired = 0;
    int16_t currAngle = 0;

    // output variable 
    int16_t pwmOut = 0;

    void calculateAngle()
    {
        
        int32_t currPos = this->encoder.read();
        
        int32_t temp = currPos * 360.0 / this->motor.getCpr();  
        this->currAngle = temp%360;

    }

    void pid()
    {

        this->error = this->desired - this->currAngle;
        this->integral += this->error;
        this->derivative = this->error - this->prevError;

        this->prevError = this->error;

        double pidOut = this->motor.getKp() * error + this->motor.getKd() * this->derivative + this->motor.getKi() * this->integral;

        this->pwmOut = constrain((int)pidOut, -255, 255);

    }


};


#endif