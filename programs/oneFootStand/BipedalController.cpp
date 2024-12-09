#include "BipedalController.hpp"

bipedalController::bipedalController() {

}

void bipedalController::setControl(string mode, KDL::Vector PIDvalues,double dts)
{
    if(mode=="DS"){
        PID_X_DS=new PIDBlock(PIDvalues[0],PIDvalues[1],PIDvalues[2],dts);
        PID_Y_DS=new PIDBlock(PIDvalues[0],PIDvalues[1],PIDvalues[2],dts);
    }
    else if(mode=="SS"){
        PID_X_SS=new PIDBlock(PIDvalues[0],PIDvalues[1],PIDvalues[2],dts);
        PID_Y_SS=new PIDBlock(PIDvalues[0],PIDvalues[1],PIDvalues[2],dts);
    }else{}
}

void bipedalController::setControl(string mode, KDL::Vector PIDvalues_X, KDL::Vector PIDvalues_Y,double dts)
{
    if(mode=="DS"){
        PID_X_DS=new PIDBlock(PIDvalues_X[0],PIDvalues_X[1],PIDvalues_X[2],dts);
        PID_Y_DS=new PIDBlock(PIDvalues_Y[0],PIDvalues_Y[1],PIDvalues_Y[2],dts);
    }
    else if(mode=="SS"){
        PID_X_SS=new PIDBlock(PIDvalues_X[0],PIDvalues_X[1],PIDvalues_X[2],dts);
        PID_Y_SS=new PIDBlock(PIDvalues_Y[0],PIDvalues_Y[1],PIDvalues_Y[2],dts);
    }else{}
}

KDL::Vector bipedalController::getOutput(string mode, KDL::Vector input)
{
    KDL::Vector output;
    if(mode=="DS"){
        output[0]=PID_X_DS->OutputUpdate(input[0]);
        output[1]=PID_Y_DS->OutputUpdate(input[1]);
        output[2]=0.0;
    }
    else if(mode=="SS"){
        output[0]=PID_X_SS->OutputUpdate(input[0]);
        output[1]=PID_Y_SS->OutputUpdate(input[1]);
        output[2]=0.0;
    }else{
        output[0]=0.0;
        output[1]=0.0;
        output[2]=0.0;
    }
    return output;
}
