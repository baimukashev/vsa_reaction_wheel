#ifndef _PID_SOURCE_
#define _PID_SOURCE_

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

class PIDImpl
{
    public:
        PIDImpl( double dt, double max, double min, double offc,double Kp, double Kd, double Ki );
        PIDImpl( double dt, double max, double min, double offc,double constant,double Kp, double Kd, double Ki );

    ~PIDImpl();
        double calculate( double setpoint, double pv );

    private:
        double _offc;
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
        double _constant = 0;
        double _vel[5] = {0};
};


PID::PID( double dt, double max, double min, double offc, double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,offc,Kp,Kd,Ki);
}
PID::PID( double dt, double max, double min, double offc,double constant ,double Kp, double Kd, double Ki )
{
    pimpl = new PIDImpl(dt,max,min,offc,constant,Kp,Kd,Ki);
}
double PID::calculate( double setpoint, double pv )
{
    return pimpl->calculate(setpoint,pv);
}
PID::~PID() 
{
    delete pimpl;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/**
 * Implementation
 */
PIDImpl::PIDImpl( double dt, double max, double min,double offc,double Kp, double Kd, double Ki ) :
    _offc(offc),
    _dt(dt),
    _max(max),
    _min(min),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}
PIDImpl::PIDImpl( double dt, double max, double min,double offc,double constant,double Kp, double Kd, double Ki ) :
        _offc(offc),
        _dt(dt),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0),
        _integral(0),
        _constant(constant)
{
}
double PIDImpl::calculate( double setpoint, double pv )
{
    
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    //average filer
    double average=derivative/5;
    for(int i=0;i<4;++i){
        _vel[i] = _vel[i+1];
        average+=_vel[i+1]/5;
    }
    _vel[4] = derivative;
    double Dout = _Kd * average;

    // Calculate total output
    double output = Pout + Iout + Dout;
    output =output + _offc*sgn(output) +_constant;
    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl()
{
}

#endif