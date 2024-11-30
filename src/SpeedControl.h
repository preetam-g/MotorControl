#ifndef MOTORCONTROL_SPEEDCONTROL_H
#define MOTORCONTROL_SPEEDCONTROL_H

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#include <Encoder.h>
#include "Motor.h"
#endif 

class SpeedControl
{

public:
    SpeedControl(Motor& motor, Encoder& encoder, double& desRpm, uint8_t& rpm_delay)
        : motor(motor), encoder(encoder) 
    {
        this->desired = desRpm;
        this->rpm_delay = rpm_delay;
    }

    void setRpm(double val)
    {
        this->desired = val;
    }

    int16_t compute()
    {
        this->calculateRpm();
        this->pid();
        return this->pwmOut;
    }

    /**
     * Calculates rpm but does not update pwm Output, can be used when rpm is within required threshold of the desired rpm
     */
    int16_t retain()
    {
        this->calculateRpm();
        return this->pwmOut;
    }

    int32_t getPos()
    {
        return this->encoder.read();
    }

    double getRpm()
    {
        return this->currRpm;
    }

    void setStartTime()
    {
        this->prevTime = millis();
    }

    void printErrors()
    {
        Serial.print(error);
        Serial.print(",");
        Serial.print(integral);
        Serial.print(",");
        Serial.print(derivative);
    }

private:

    // Motor and Encoder references
    Motor& motor;
    Encoder& encoder;

    // RPM calculation variables
    double desired = 0;
    double currRpm = 0;
    double prevRpm = 0;

    uint32_t prevTime = 0; 
    int32_t prevPos = 0;

    uint8_t rpm_delay = 0;

    // PID variables
    double error = 0, prevError = 0;
    double integral = 0;
    double derivative = 0;

    // Output variable
    int16_t pwmOut = 0;

    void calculateRpm()
    {

        uint32_t currTime = millis();
        uint32_t deltaTime = currTime - prevTime;

        if (deltaTime > rpm_delay)
        {

            int32_t currPos = this->encoder.read();
            int32_t deltaPos = currPos - prevPos;

            this->currRpm = (double)(60000 * deltaPos / this->motor.getCpr() / deltaTime);

            // 2 point moving average
            this->currRpm = (this->currRpm + this->prevRpm)/2;
            this->prevRpm = this->currRpm;


            this->prevPos = currPos;
            this->prevTime = currTime;

        }

    }

    void pid()
    {

        this->error = this->desired - this->currRpm;
        this->integral += this->error;
        this->derivative = this->error - this->prevError;

        this->prevError = this->error;

        double pidOut = this->motor.getKp() * error + this->motor.getKd() * this->derivative + this->motor.getKi() * this->integral;

        this->pwmOut = constrain((int)pidOut, -255, 255);

    }

};

#endif
