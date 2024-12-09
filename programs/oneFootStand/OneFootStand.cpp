// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "OneFootStand.hpp"
#include "utils.h"
#include <cmath> // std::abs, std::copysign

#include <algorithm> // std::copy, std::max, std::min
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>
#include <unistd.h>
#include <kdl/utilities/utility.h> // KDL::deg2rad
#include <KdlVectorConverter.hpp> // TODO: unused
#include <Eigen/Dense>


using namespace roboticslab;

bool OneFootStand::configure(yarp::os::ResourceFinder & rf)
{
    system("pkill gnuplot_qt");
//    yCDebug(OFS) << "Config:" << rf.toString();
    period = rf.check("periodMs", yarp::os::Value(0), "period [ms]").asInt32() * 0.001;
    if (period <= 0)
    {
        yCError(OFS) << "Missing or invalid period parameter:" << static_cast<int>(period * 1000) << "(ms)";
        return false;
    }

    walkingRobot robot;
    auto robotRemote = rf.find("robotRemote").asString();
    auto sensorRemote = rf.find("sensorRemote").asString();


    robot.initWalkingRobot(robotRemote,DEFAULT_LOCAL_PREFIX);
    robot.initSensor(sensorRemote,period);
    data.open(NOMBRE_ARCHIVO2, std::fstream::out);
    data1.open(NOMBRE_ARCHIVO, std::fstream::out);
    data2.open(NOMBRE_ARCHIVO3, std::fstream::out);
    data << "pos x" << DELIMITADOR <<"pos y"  << DELIMITADOR << "pos z"  << DELIMITADOR
            << "turn x" << DELIMITADOR <<"turn y"  << DELIMITADOR << "turn z"  << DELIMITADOR
            << "ZMPx"  << DELIMITADOR <<"ZMPy" << DELIMITADOR << "ZMPz" << DELIMITADOR
            << "targetZMPx"  << DELIMITADOR <<"targetZMPy" << DELIMITADOR << "targetZMPz" <<"\n";
    data1<< "time" << DELIMITADOR <<"RL_x"  <<DELIMITADOR<<"LL_x" << DELIMITADOR <<"zmp_x" << DELIMITADOR << "zmp_y" << DELIMITADOR << "zmp_z"<<"\n";
    data2 << "time" << DELIMITADOR <<"ZMP X TARGET"  << DELIMITADOR << "ZMPX"  << DELIMITADOR
          << "ZMPY" << DELIMITADOR <<"ZMPZ"<<"\n";


    SystemBlock sys(vector<double>{ 0.086760305974496 , -0.098958263005331  ,-0.174195481119144,   0.304401833644137,  -0.117741684003736},
     vector<double>{-0.264939789653879 ,  2.005721776627335,  -5.394085289985741,   6.829900630666893,  -4.176252606448626,   1.000000000000000});


    double b=0.9672;
    double dts=0.02;
    imc=calculateIMC_Controller(sys,b);
//mlp vs  Kolmogorov-Arnold Networks
    reference=new IPlot(period, "Reference signal vs time ", "Time (s)", "Control signal ");
    pVt=new IPlot(period,"Position vs time ", "Time (s)", "Position signal ZMP (m) ");
    err=new IPlot(period,"Error vs time ", "Time (s)", "Error (m)");
    err_model=new IPlot(period,"Error of model vs time ", "Time (s)", "Error (m)");
    control=new IPlot(period,"control Signal vs time ", "Time (s)", "input(m)");
    sistema=new IPlot(period,"sistema vs time ", "Time (s)", "input(m)");
    int numOrder=sys.GetNumOrder(),denOrder=sys.GetDenOrder();
    //ref=VectorXd::Zeros(SIMULATION_TIME/period);

 /////////////////////////////agragado para probar trayectroias
//step with slope
    double tiempoIni=9;//10
    for (double var = 0; var < tiempoIni; var+=dts) {

        trajectoria.push_back(0);
    }
    for (double var = 0; var < 1.0; var+=dts) {
        trajectoria.push_back(TARGET_ZMP*var);
    }
    for (double var = 0; var < 30; var+=dts) {
    trajectoria.push_back(trajectoria.back());
    }

//trayectoria
//    for (double var = 0; var <= 10.0; var+=dts) {
//        trajectoria.push_back(0);
//    }
//    for (double var = 0; var <= 20.0; var+=dts) {
//        if(var<=10)
//        trajectoria.push_back(TARGET_ZMP/10*var);
//        else
//            trajectoria.push_back(TARGET_ZMP+(TARGET_ZMP/10*(10-var)));
//    }
//    for (double var = 0; var <= 20.0; var+=dts) {
//        if(var<=10)
//        trajectoria.push_back(TARGET_ZMP/10*var);
//        else
//            trajectoria.push_back(TARGET_ZMP+(TARGET_ZMP/10*(10-var)));
//    }
//    for (double var = 0; var < 20; var+=dts) {
//        trajectoria.push_back(trajectoria.back());
//    }


    for (int var = 0; var <=trajectoria.size(); var+=1) {

        reference->pushBack(trajectoria[var]);
    }
reference->Plot();

    double mag,phi;

    sys.GetMagnitudeAndPhase(dts,1,mag,phi);

//    FPDTuner tuner(60,1);
//    tuner.TuneIsom(sys,controller);
    controller.ParameterUpdate(vector<double>{0,1,-1.4});
    controller.PrintParameters();


    double kpx=0;//0.0258;//-0.009815;//7.1;//3.5817;//2.887;//0.4753;//0.4625;//1.6984;
    double kix=0.005;//0;//-0.07818;//0.0;//1.3198;//0.0459;//0.0;//0.2149;
    double kdx=0;//0.00149;//-0.0003081;//0.4658;//0.0;//0.0;//0.1195;
    double ts=0.02;

    PIDBlock pidx(kpx,kix,kdx,ts);


    PIDx=pidx;

    double kpy=-1.333;

    double kiy=-0.0082545;
    double kdy=-0.10837;

//    std::vector<double> num={-0.5128,0.5128},den={-0.9897,1};
     std::vector<double> num={-0.5128,0.5128},den={-0.9897,1};
    std::vector<double> num2={-1,1},den2={-1.68,1.7};
    SystemBlock f(num,den),f2(num2,den2);
    filtro=f;
    filtro2=f2;
    PIDBlock pidY(kpy,kiy,kdy,ts);
    PIDy=pidY;
    leftInitial.resize(6);
    rightInitial.resize(6);
    robot.enc_Left->getEncoders(leftInitial.data());
    robot.enc_Right->getEncoders(rightInitial.data());
    l=leftInitial[4];r=rightInitial[4];
    this->generate_input();
    index=0;
    KDL::Vector Pzmp;
    robot.getZMP(Pzmp);
    initZMP=Pzmp[0];
    //SISTEMA IDENTIFICADO PARA ZMP 0.05
    MatrixXd Am(5,5),Bm(5,1),Cm(1,5),Dm(1,1);

    Am <<     4.176252606448626,  -6.829900630666893,   5.394085289985741,  -2.005721776627335,   0.264939789653879
              ,1.0000    ,     0      ,   0    ,     0         ,0
                   ,0    ,1.0000     ,    0     ,    0         ,0
                   ,0    ,     0    ,1.0000     ,    0         ,0
                   ,0    ,     0         ,0    ,1.0000         ,0;
    Bm <<
          1,
          0,
          0,
          0,
          0;

  Cm << -0.117741684003736,   0.304401833644137,  -0.174195481119144,  -0.098958263005331,   0.086760305974496;
  Dm << 0;

  StateSpace ss(Am,Bm,Cm,Dm,dts);

//   pVt();
   mpc.setSystem(ss);
   mpc.setParam(80,40,1,1);
   first = yarp::os::Time::now();
   return yarp::os::PeriodicThread::setPeriod(period) && yarp::os::PeriodicThread::start();

}

////bool a=true;
//KDL::Vector error;
//double timeLimit=5.0;

//double fmPos=0,jj=0;
//double fmTarget=TARGET_ZMP;
//double actualError=0,actualControl=0;
//double error_model=0;

void OneFootStand::run()
{
//    MPCscheme();
LIPM2d_system();
//    IMCscheme();
//    saveIdentificationData();
//    controlScheme();
}

void OneFootStand::LIPM2d_system()
{
    double now = yarp::os::Time::now()-first;
    yCDebug(OFS)<<now;
    KDL::Vector Pzmp;
    robot.getZMP(Pzmp);
    double zmp_norm=Pzmp[0]-initZMP;

    //Control loop
    if( now<SIMULATION_TIME)
    {
       model_Ju.model(zmp_norm,trajectoria[jj]);
       double ka = 0.25 * trajectoria[jj] + 9.95; // dudo entre zmp_ref o Xzmp_ft
       double _ang_ref = (trajectoria[jj]*(-g))/ (L*(ka-g));
       double _ang_out =  model_Ju.ang_error_out + _ang_ref;
       std::vector<double> a=leftInitial;
       std::vector<double> b=rightInitial;
       a[4]=leftInitial[4]+_ang_out;
       b[4]=rightInitial[4]+_ang_out;
       jj++;
       robot.moviLeftJoint(a);
       robot.moviRightJoint(b);

       reference->pushBack(trajectoria[jj]);
       pVt->pushBack(zmp_norm);
       control->pushBack(_ang_out);
    }
    else{
        reference->PlotAndSave("limp2d/3/Reference"+std::to_string(TARGET_ZMP));
        pVt->PlotAndSave("limp2d/3/Response_"+std::to_string(TARGET_ZMP));
        control->PlotAndSave("limp2d/3/Control"+std::to_string(TARGET_ZMP));
        askToStop();
    }
}
