
#include "OneFootStand.hpp"
using namespace roboticslab;
const double TARGET_ZMP=0.03;
const std::string NOMBRE_ARCHIVO = "/home/gerson/MATLAB-Drive/IdSystemDS_x_Gerson_"+std::to_string(TARGET_ZMP)+".csv";
const char DELIMITADOR =' ';
const double SIMULATION_TIME=60;
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

KDL::Vector error;
double timeLimit=5.0;

double fmPos=0,jj=0;
double fmTarget=TARGET_ZMP;
double actualError=0,actualControl=0;
double error_model=0;

void OneFootStand::MPCscheme(){

    double now = yarp::os::Time::now()-first;
    yCDebug(OFS)<<now;
    KDL::Vector Pzmp;
    robot.getZMP(Pzmp);
    double zmp_norm=Pzmp[0]-initZMP;
    //Control loop
    if( now<SIMULATION_TIME)
    {
       actualControl=mpc.updateControl(zmp_norm,ref.segment(jj,80));

       jj++;
       std::vector<double> a=leftInitial;
       std::vector<double> b=rightInitial;
       a[4]=leftInitial[4]+zmptoangle(actualControl);
       b[4]=rightInitial[4]+zmptoangle(actualControl);
       robot.moviLeftJoint(a);
       robot.moviRightJoint(b);
        err->pushBack(zmp_norm-ref[jj]);
        reference->pushBack(ref[jj]);
        pVt->pushBack(zmp_norm);
        control->pushBack(actualControl);
    }
    else{
        reference->PlotAndSave("MPC/3/ReferenceMPC_"+std::to_string(TARGET_ZMP));
        pVt->PlotAndSave("MPC/3/ResponseMPC_"+std::to_string(TARGET_ZMP));
        err->PlotAndSave("MPC/3/errorMPC_"+std::to_string(TARGET_ZMP));
        control->PlotAndSave("MPC/3/ControlMPC_"+std::to_string(TARGET_ZMP));
        askToStop();
//        close();
    }
}

//CONTROLADOR DE IMC
void OneFootStand::IMCscheme(){

    double now = yarp::os::Time::now()-first;
    yCDebug(OFS)<<now;
    KDL::Vector Pzmp;
    robot.getZMP(Pzmp);
    double zmp_norm=Pzmp[0]-initZMP;
    //Control loop
    if( now<SIMULATION_TIME)
    {
        if(jj<=(trajectoria.size()-1)){
            actualError=trajectoria[jj]-error_model;

        }
        else
            actualError=trajectoria[trajectoria.size()-1]-error_model;
       actualControl=actualError>imc;
       std::vector<double> a=leftInitial;
       std::vector<double> b=rightInitial;
       a[4]=leftInitial[4]+zmptoangle(actualControl);
       b[4]=rightInitial[4]+zmptoangle(actualControl);
       jj++;
       fmPos=actualControl>sys;
       error_model=(zmp_norm)-fmPos;

       robot.moviLeftJoint(a);
       robot.moviRightJoint(b);

       err->pushBack(actualError);
       err_model->pushBack(error_model);
       reference->pushBack(trajectoria[jj]);
       pVt->pushBack(zmp_norm);
       control->pushBack(actualControl);
       sistema->pushBack(fmPos);
       pVt->pushBack(Pzmp[0]-initZMP);
    }
    else{
        reference->PlotAndSave("IMC/3/ReferenceIMC_"+std::to_string(TARGET_ZMP));
        pVt->PlotAndSave("IMC/3/ResponseIMC_"+std::to_string(TARGET_ZMP));
        err->PlotAndSave("IMC/3/errorIMC_"+std::to_string(TARGET_ZMP));
        sistema->PlotAndSave("IMC/3/simuladoIMC_"+std::to_string(TARGET_ZMP));
        err_model->PlotAndSave("IMC/3/errorIMC_"+std::to_string(TARGET_ZMP));
        control->PlotAndSave("IMC/3/ControlIMC_"+std::to_string(TARGET_ZMP));
        askToStop();
    }
}
//controlador de PID

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
       control->pushBack(actualControl);
       pVt->pushBack(Pzmp[0]);
   }
   else{
       control->PlotAndSave("control0");
       pVt->PlotAndSave("sistema0");
       askToStop();
//       interruptModule();
   }
}

//genera la entrada de referencia
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
        pVt->pushBack(Pzmp[0]);
        data1 <<now<< DELIMITADOR2<<rightInitial[4]<<DELIMITADOR2<<leftInitial[4]
              <<DELIMITADOR2<<Pzmp[0]<<DELIMITADOR2<<Pzmp[1]<<DELIMITADOR2<<Pzmp[2]<<"\n";
    }
    else{
        pVt->Plot();
        askToStop();
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
    yCDebug(OFS)<<"interruptModule";
    data.close();
    data1.close();
    robot.close();
//    yarp::os::PeriodicThread::stop();
    return true;
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
    yCDebug(OFS)<<"close()";
    data.close();
    data1.close();
    robot.close();
    delete reference;
    delete pVt;
    delete err;
    delete err_model;
    delete control;
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
