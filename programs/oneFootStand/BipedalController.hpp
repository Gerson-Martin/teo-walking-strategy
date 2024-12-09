#ifndef __BIPEDAL_CONTROLLER_HPP__
#define __BIPEDAL_CONTROLLER_HPP__
#include <vector>


#include <kdl/frames.hpp>

#include <fcontrol.h>

#include <kdl/utilities/utility.h> // KDL::deg2rad

class bipedalController
{
PIDBlock *PID_X_DS,*PID_Y_DS,*PID_X_SS,*PID_Y_SS;
public:
    bipedalController();
    void setControl(string mode, KDL::Vector PIDvalues, double dts);
    void setControl(string mode, KDL::Vector PIDvalues_X, KDL::Vector PIDvalues_Y, double dts);
    KDL::Vector getOutput(string mode, KDL::Vector input);

};



#endif
