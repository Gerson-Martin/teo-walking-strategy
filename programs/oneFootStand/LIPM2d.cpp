/*
 * 2dLIPM.cpp
 *
 *  Created on: 03/12/2015
 *      Author: teo
 */

#include "global.h"
#include "LIPM2d.h"

LIPM2d::LIPM2d()
{

    // Inicializacion variables
    _ang_ref = 0.0; // inintialization of the inputs
    _zmp_ft = 0;
    //_zmp_ref = 0.0;

    _a = 0.834; // quadratic error equacion from exp1 (testCap53)
    _b = 1.024;
    _c = -0.0004;

    _u = 0.0; // inintialization of the outputs
    _u_ref = 0.0;
    y = 0.0;

    _x1[0] = 0.0;  // inintialization of the space state variable
    _x1[1] = 0.0;
    _x2[0] = 0.0;
    _x2[1] = 0.0;

    _z[0] = 0.0;
    _z[1] = 0.0;

    pre_y = 0.0; // others options (not used)
    dy = 0.0;

    PIDout = 0.0; // PID options (not used)
    Pout = 0.0;
    Iout = 0.0;
    Dout = 0.0;

    // initialization damping variables
    _Wn = 0.0; // natural frecuency
    _chi = 1; // damping coefficient
    _ka = 9.91; // spring
    _ka_const = 9.9575;
    _ba = 38.6844; // shock
}


LIPM2d::~LIPM2d(){
}

float LIPM2d::model(float ft, float ref)     /** STATE FEEDBACK WITH DAMPING (ka, ba) **/
{
    // Discrete-time Space State Model evaluation

    _zmp_error =  ref - ft;

    _x1[0] = _x1[1];
    _x2[0] = _x2[1];

    _ka_const = 0.25 * ref + 9.95;
    _ang_ref = (ref*(-G))/ (L*(_ka_const-G)) ;

    //_ang_ref = -(asin(ref/L)*180/G); // relacion entre zmp_ref y ang_ref

    _zmp_ft = _a*ref*ref + _b*ref + _c; // equation from exp1 (stationary error)

    if (_ang_ref==0 || _zmp_ft<0.00000000001){ // calculation of ka in steady state
        _ka = 9.95;
    }
    else    {
        _ka = ((_zmp_ft*(-1)*G/L)/_ang_ref)+G;
    }

    _Wn = sqrt((_ka-G))/L; // based on the general form of a 2nd order transfer function
    _ba = 2*_chi*_Wn*M*L; // calculation of ba related to the previous line and chi = 1 (damping coefficient);

    // Dynamical-LIPM represented in space state
    _A[0][0] = 0.0;
    _A[0][1] = 1.0;
    _A[1][0] = -((_ka-G)/L);
    _A[1][1] = -(_ba/(M*L));

    _B[0][0] = 0.0;
    _B[1][0] = 1.0;

    _C[0] = -G/(L*L);
    _C[1] = 0.0;

    _D = 0.0;


    _u = _zmp_error; //bucle abierto. nuestra entrada es directamente la refencia de ZMP que queramos.
    y = _C[0]*_x1[0] + _C[1]*_x2[0] + _D*_u;
    //dy = (y - pre_y) / _T; // velocity

    _x1[1] = _A[0][0]*_x1[0] + _A[0][1]*_x2[0] + _B[0][0]*_u;
    _x2[1] = _A[1][0]*_x1[0] + _A[1][1]*_x2[0] + _B[1][0]*_u;

    ang_error_out = y; // only for a intuitive name fot the output
    return ang_error_out;

}
