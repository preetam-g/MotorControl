#ifndef MOTORCONTROL_MOTOR_H
#define MOTORCONTROL_MOTOR_H

class Motor
{

private:    
    int cpr; // counts per cycle of output shaft
    double kp = 0, ki = 0, kd = 0;

public:

    Motor(int output_shaft_cpr, double kp = 0, double ki = 0, double kd = 0) 
    : cpr(output_shaft_cpr), kp(kp), ki(ki), kd(kd) {}

    int getCpr() {return this->cpr;}
    double getKp() {return this->kp;}
    double getKi() {return this->ki;}
    double getKd() {return this->kd;}
    
};

#endif