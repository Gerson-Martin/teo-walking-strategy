// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "OneFootStand.hpp"
#include "OneFootStand.hpp"
#include "OneFootStand.hpp"
#include "OneFootStand.hpp"
#include "OneFootStand.hpp"

#include <cmath> // std::abs, std::copysign

#include <algorithm> // std::copy, std::max, std::min
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/os/SystemClock.h>
#include <unistd.h>
#include <kdl/utilities/utility.h> // KDL::deg2rad
#include <KdlVectorConverter.hpp> // TODO: unused



using namespace roboticslab;
const double TARGET_ZMP=0.03;
const std::string NOMBRE_ARCHIVO = "/home/gerson/MATLAB-Drive/IdSystemDS_x_PE_"+std::to_string(TARGET_ZMP)+".csv";
const char DELIMITADOR =' ';
const double SIMULATION_TIME=15;
double first;
std::ofstream data1,data2;
const std::string NOMBRE_ARCHIVO2 = "/home/gerson/MATLAB-Drive/Control.txt";
const std::string NOMBRE_ARCHIVO3 = "/home/gerson/MATLAB-Drive/juanmi_"+std::to_string(TARGET_ZMP)+".txt";
const char DELIMITADOR2 =' ';
double l,r;
double initZMP=0;


namespace
{
    YARP_LOG_COMPONENT(OFS, "rl.OneFootStand")
}

constexpr auto DEFAULT_LOCAL_PREFIX = "/oneFootStand";
constexpr auto DEFAULT_IK_STEP = 0.001; // [m]
constexpr auto DEFAULT_MAX_SPEED = 0.01; // [m/s]
constexpr auto DEFAULT_MAX_ACCELERATION = 0.1; // [m/s^2]
constexpr auto REF_FRAME = ICartesianSolver::TCP_FRAME;

bool OneFootStand::configure(yarp::os::ResourceFinder & rf)
{

//    yCDebug(OFS) << "Config:" << rf.toString();

    period = rf.check("periodMs", yarp::os::Value(0), "period [ms]").asInt32() * 0.001;
    if (period <= 0)
    {
        yCError(OFS) << "Missing or invalid period parameter:" << static_cast<int>(period * 1000) << "(ms)";
        return false;
    }


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
//    data1<< "time" << DELIMITADOR <<"RL_x"  << DELIMITADOR << "RL_y"  << DELIMITADOR
//            << "RL_z" << DELIMITADOR <<"LL_x"  << DELIMITADOR << "LL_y"  << DELIMITADOR
//            << "LL_z"  << DELIMITADOR <<"zmp_x" << DELIMITADOR << "zmp_y" << DELIMITADOR << "zmp_z"<<"\n";
    data1<< "time" << DELIMITADOR <<"RL_x"  <<DELIMITADOR<<"LL_x" << DELIMITADOR <<"zmp_x" << DELIMITADOR << "zmp_y" << DELIMITADOR << "zmp_z"<<"\n";
    data2 << "time" << DELIMITADOR <<"ZMP X TARGET"  << DELIMITADOR << "ZMPX"  << DELIMITADOR
          << "ZMPY" << DELIMITADOR <<"ZMPZ"<<"\n";

//    sys=SystemBlock(vector<double>{ 0 ,    0 ,  0.008337141952312  ,-0.015779442449882  , 0.007493362739673        ,0},vector<double>{-0.813867030581406 ,  4.20293861541632,  -8.717849112092322,   9.081650142885303,  -4.752787415097272 ,  1.000000000000000});

//    SystemBlock sys(vector<double>{ 0.061794582093025,  -0.049153142746674,  -0.121961799425836,   0.100005120044723 ,  0.060310156915134 , -0.050709037715726},
//     vector<double>{-0.210755629246432  ,1.788619698539252 ,-5.065443396032936  ,6.607164247952522, -4.119215426031505 ,1.000000000000000});
    SystemBlock sys(vector<double>{ 0.0235149016848827, -0.0362969537779307, -0.0181615893732595,0.0529282819062423,-0.0219013799863369},
     vector<double>{-0.743653297812389,3.92954164147509,-8.3155221538982,8.81657986617873 ,-4.68683215649146 ,1});
//    SystemBlock sys(vector<double>{ 0.086760305974496 , -0.098958263005331  ,-0.174195481119144,   0.304401833644137,  -0.117741684003736},
//     vector<double>{-0.264939789653879 ,  2.005721776627335,  -5.394085289985741,   6.829900630666893,  -4.176252606448626,   1.000000000000000});

    double a=0.6;
    double b=1;
    double dts=0.02;
    imc=calculateIMC_Controller(sys,b);

    cs=new IPlot(period, "Control signal vs time ", "Time (s)", "Control signal ");
    pVt=new IPlot(period,"Position vs time ", "Time (s)", "Position signal ZMP (m) ");
    err=new IPlot(period,"Error vs time ", "Time (s)", "Error (rad)");
    int numOrder=sys.GetNumOrder(),denOrder=sys.GetDenOrder();
 /////////////////////////////agragado para probar trayectroias

    for (double var = 0; var <= 20.0; var+=dts) {
        if(var<=10)
        trajectoria.push_back(TARGET_ZMP/10*var);
        else
            trajectoria.push_back(TARGET_ZMP+(TARGET_ZMP/10*(10-var)));
//        err_plot.pushBack(trajectoria.back());
    }

    ///////////////////////////////////////////////////



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
//    KDL::Vector Pzmp;
//    robot.getZMP(Pzmp);
//    initZMP=Pzmp[0];
    first = yarp::os::Time::now();
    return yarp::os::PeriodicThread::setPeriod(period) && yarp::os::PeriodicThread::start();
}

bool a=true;
KDL::Vector error;
double timeLimit=5.0;






double fmPos=0;
double fmTarget=TARGET_ZMP;
double actualError,actualControl;
double error_model=0;





void OneFootStand::run()
{
    IMCscheme();
//    saveIdentificationData();
//    controlScheme();
}


void OneFootStand::IMCscheme(){

    double now = yarp::os::Time::now()-first;
    yCDebug(OFS)<<now;
    KDL::Vector Pzmp;
    robot.getZMP(Pzmp);
    //Control loop
    if( now<SIMULATION_TIME)
    {
       actualError=fmTarget-error_model;
       actualControl=actualError>imc;
       fmPos=actualControl>sys;
       error_model=(Pzmp[0])-fmPos;
       std::vector<double> a=leftInitial;
       std::vector<double> b=rightInitial;
       a[4]=leftInitial[4]+zmptoangle(actualControl);
       b[4]=rightInitial[4]+zmptoangle(actualControl);
        //fmPos=sys.OutputUpdate(actualControl);

       robot.moviLeftJoint(a);
       robot.moviRightJoint(b);


        err->pushBack(actualError);
        cs->pushBack(actualControl);
        pVt->pushBack(Pzmp[0]);
    }
    else{
        cs->PlotAndSave("control");
        pVt->PlotAndSave("sistema");
        close();
    }
}
void OneFootStand::controlScheme(){
    //control a fake motor
   double now = yarp::os::Time::now()-first;
   yCDebug(OFS)<<now;
   KDL::Vector Pzmp;
   robot.getZMP(Pzmp);

   //Control loop
   if( now<SIMULATION_TIME)
   {
      actualError=fmTarget-Pzmp[0];
      actualControl=controller.OutputUpdate(actualError);
//      actualControl=PIDx.UpdateControl(actualError);
      //CAMBIAR NOMBRE
      std::vector<double> a=leftInitial;
      std::vector<double> b=leftInitial;
      a[4]=leftInitial[4]+zmptoangle(actualControl);
      b[4]=rightInitial[4]+zmptoangle(actualControl);
       //fmPos=sys.OutputUpdate(actualControl);

      robot.moviLeftJoint(a);
      robot.moviRightJoint(b);


       err->pushBack(actualError);
       cs->pushBack(actualControl);
       pVt->pushBack(Pzmp[0]);
   }
   else{
       cs->PlotAndSave("control");
       pVt->PlotAndSave("sistema");
       close();
//       interruptModule();
   }
}
void OneFootStand::generate_input(){

    double target_theta=-asin(TARGET_ZMP/(L))*180/3.14159;//target take of the initial position
    double delta=target_theta/50;
    double target=target_theta+l ;//global target
    double current_theta=l;//starting in initial position
    double diff=0;
    double PE=0;
    generatedInput.push_back(current_theta);
    for(int i=0;i<=SIMULATION_TIME/period;i++){
        PE=target_theta/1000*((rand() % 10 + 1)-5);
        if((current_theta)>(target)){
            diff=(target-current_theta);
            if(abs(diff)>=abs(delta)){//si la diferecnia es menor que el error.
                current_theta+=delta;
            }else{//si la diferecnia es mayor que el error, sumamos el error
                current_theta+=diff;
            }
        }
        yCDebug(OFS)<<current_theta<<"  "<<target;
        generatedInput.push_back(current_theta+PE);
    }

}


void OneFootStand::saveIdentificationData(){
    double now = yarp::os::Time::now()-first;
//    yCDebug(OFS)<<now;
    KDL::Vector Pzmp;
    robot.getZMP(Pzmp);
    rightInitial[4]=generatedInput[index];
    leftInitial[4]=generatedInput[index];
    index++;
    if( now<SIMULATION_TIME){
        robot.moviRightJoint(rightInitial);
        robot.moviLeftJoint(leftInitial);
        data1 <<now<< DELIMITADOR2<<rightInitial[4]<<DELIMITADOR2<<leftInitial[4]
              <<DELIMITADOR2<<Pzmp[0]<<DELIMITADOR2<<Pzmp[1]<<DELIMITADOR2<<Pzmp[2]<<"\n";
    }
    else{
        close();
    }
}

void OneFootStand::printAndSave(std::vector<double> vd, KDL::Vector Pzmp, KDL::Vector target_ZMP,KDL::Vector controlSignal,std::vector<double> vd2){

    yCDebug(OFS) << "ZMP : ["<<Pzmp[0]<<" ; "<<Pzmp[1]<<" ; "<<Pzmp[2]<<"]";
    yCDebug(OFS) << "Error : ["<<error[0]<<" ; "<<error[1]<<" ; "<<error[2]<<"]";
    yCDebug(OFS) << "CONTROL : ["<<controlSignal[0]<<" ; "<<controlSignal[1]<<" ; "<<controlSignal[2]<<"]";
    yCDebug(OFS) << "Posicion Siguiente : ["<<vd[0]<<" ; " <<vd[1]<<" ; "<<vd[2]<<" ; "<<vd[3]<<" ; "<<vd[4]<<" ; "<<vd[5]<<"]";
    yCDebug(OFS) << "POsicion Actual : [" <<vd2[0]<<" ; " <<vd2[1]<<" ; "<<vd2[2]<<" ; "<<vd2[3]<<" ; "<<vd2[4]<<" ; "<<vd2[5]<<"]";
    System dato;
    dato.input.x=vd[0];
    dato.input.y=vd[1];
    dato.input.z=vd[2];
    dato.input.gx=vd[3];
    dato.input.gy=vd[4];
    dato.input.gz=vd[5];
    dato.output.x=Pzmp[0];
    dato.output.y=Pzmp[1];
    dato.output.z=Pzmp[2];
    dato.target.x=target_ZMP[0];
    dato.target.y=target_ZMP[1];
    dato.target.z=target_ZMP[2];
    saveData(dato);
}
void OneFootStand::printAndSave(double angle,double x, KDL::Vector Pzmp, KDL::Vector target_ZMP){

    data1 <<angle<< DELIMITADOR2<<x<< DELIMITADOR2<<Pzmp[0]<<DELIMITADOR2<<Pzmp[1]<<DELIMITADOR2<<Pzmp[2]<<DELIMITADOR2
         <<target_ZMP[0]<<DELIMITADOR2<<target_ZMP[1]<<DELIMITADOR2<<target_ZMP[2]<<"\n";
}

void OneFootStand::printAndSave(double time, KDL::Vector Pzmp, KDL::Vector target_ZMP){

    data2 <<time<< DELIMITADOR2<<target_ZMP[0]<<DELIMITADOR2<<target_ZMP[1]<<DELIMITADOR2<<target_ZMP[2]
          << DELIMITADOR2<<Pzmp[0]<<DELIMITADOR2<<Pzmp[1]<<DELIMITADOR2<<Pzmp[2]<<"\n";
}

bool OneFootStand::updateModule()
{
    return yarp::os::PeriodicThread::isRunning();
}

bool OneFootStand::interruptModule()
{
    data.close();
    data1.close();
    robot.close();
    yarp::os::PeriodicThread::stop();
    return 0 ;
}

double OneFootStand::getPeriod()
{
    return 1.0; // [s]
}
void OneFootStand::EMA_Exponential_Filter(KDL::Vector &data,double alpha){
    data=alpha*data+(1-alpha)*datoAnterior;
}
bool OneFootStand::close()
{
    data.close();
    data1.close();
    robot.close();
    askToStop();
    return true;
}


void OneFootStand::saveData(System system)
{
        data << system.input.x << DELIMITADOR <<system.input.y << DELIMITADOR << system.input.z << DELIMITADOR
             << system.input.gx << DELIMITADOR <<system.input.gy << DELIMITADOR << system.input.gz << DELIMITADOR
             << system.output.x << DELIMITADOR <<system.output.y << DELIMITADOR << system.output.z << DELIMITADOR
             << system.target.x << DELIMITADOR <<system.target.y << DELIMITADOR << system.target.z << "\n";
}
double OneFootStand::zmptoangle(double zmp){
    return -asin(zmp/(L))*180/3.14159;
}

/////////////////////////////////////////////////IMC/////////////////////////////

SystemBlock OneFootStand::calculateIMC_Controller(SystemBlock& sys, double &b){
    //choose the stable  system of the indetified sistem
    std::vector<double> numSys,denSys;
    sys.GetZTransferFunction(numSys,denSys);
    roots zeros=get_nth_root(numSys);
    roots zeros_estables;
    //cout<<"sistema real 3"<<endl;
    for (int i = 0; i < zeros.size(); ++i) {
        if(std::norm(zeros[i])<=1){
            zeros_estables.push_back(zeros[i]);
        }
    }
    //////Hallamos G_estable y la invertimos
    std::vector<double> num_G_est;
    if(!zeros_estables.empty())
        num_G_est=calculatePolynomial(zeros_estables);
    else
        num_G_est=numSys;
    SystemBlock G_inv_estable(denSys,num_G_est);

    //Hallamos el filtro que compensa los zeros no estables y el comportamiento deseado
    double n=sys.GetNumOrder()-zeros_estables.size()+1;
    std::vector<double> numFilter,denFilter(n+1);
    roots aux(n,std::complex<double>(b));
    denFilter=calculatePolynomial(aux);
    //para hallar el numerador del filtro, compensaremos la ganacia del sistema total sys+control
    double gainFilter=1/std::accumulate(denFilter.begin(),denFilter.end(),0.0);
    double gainStatic=1/G_inv_estable.GetGain()*1/sys.GetGain()*1/gainFilter;
    numFilter.push_back(gainStatic);
    SystemBlock filter(numFilter,denFilter);
    //calculamos el sistema final IMC
    SystemBlock IMC=G_inv_estable*filter;
    return IMC;
}
std::vector<double> OneFootStand::calculatePolynomial(roots r){
    roots respuesta{-r[0],1};
    for (int i = 1; i < r.size(); ++i) {
        respuesta=conv(respuesta,roots{-r[i],1});
    }
    std::vector<double> polynomial;
    for (int i = 0; i < respuesta.size(); ++i) {
        polynomial.push_back(respuesta[i].real());
    }
    return polynomial;
}

OneFootStand::roots OneFootStand::get_nth_root(std::vector<double> &polynomial){
    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    VectorXd v1= Eigen::Map<Eigen::VectorXd,Eigen::Unaligned>(polynomial.data(),polynomial.size());
    solver.compute(v1);
    const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType &r =solver.roots();
    roots zeros(&r[0],r.data()+r.rows());
    return zeros;
}


template<typename T>
std::vector<T> OneFootStand::conv(const std::vector<T> &f, const std::vector<T> &g)
{
    int const nf = f.size();
    int const ng = g.size();
    int const n  = nf + ng - 1;
    std::vector<T> out(n, T());
    for(auto i(0); i < n; ++i) {
      int const jmn = (i >= ng - 1)? i - (ng - 1) : 0;
      int const jmx = (i <  nf - 1)? i            : nf - 1;
      for(auto j(jmn); j <= jmx; ++j) {
        out[i] += (f[j] * g[i - j]);
      }
    }
    return out;
}
