// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

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
const std::string NOMBRE_ARCHIVO = "/home/gerson/MATLAB-Drive/IdSystemDS.csv";
const char DELIMITADOR =' ';
double first;
std::ofstream data1,data2;
const double TARGET_ZMP=0.01;
const std::string NOMBRE_ARCHIVO2 = "/home/gerson/MATLAB-Drive/Control.txt";
const std::string NOMBRE_ARCHIVO3 = "/home/gerson/MATLAB-Drive/juanmi_"+std::to_string(TARGET_ZMP)+".txt";
const char DELIMITADOR2 =' ';


namespace
{
    YARP_LOG_COMPONENT(OFS, "rl.OneFootStand")

    bool parseFrameRotation(const yarp::os::Value & value, const std::string & name, KDL::Rotation & rot)
    {
        if (!value.isNull())
        {
            if (!value.isList() || value.asList()->size() != 3)
            {
                yCError(OFS) << "Parameter" << name << "must be a list of 3 doubles";
                return false;
            }

            yCInfo(OFS) << name << "RPY [deg]:" << value.toString();

            auto roll = value.asList()->get(0).asFloat64() * KDL::deg2rad;
            auto pitch = value.asList()->get(1).asFloat64() * KDL::deg2rad;
            auto yaw = value.asList()->get(2).asFloat64() * KDL::deg2rad;

            // sequence (old axes): 1. R_x(roll), 2. R_y(pitch), 3. R_z(yaw)
            rot = KDL::Rotation::RPY(roll, pitch, yaw);
        }
        else
        {
            yCInfo(OFS) << "Using no" << name;
            rot = KDL::Rotation::Identity();
        }

        return true;
    }
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

//    ikStep = rf.check("ikStep", yarp::os::Value(DEFAULT_IK_STEP), "IK step [m]").asFloat64();

//    if (ikStep <= 0.0)
//    {
//        yCError(OFS) << "Invalid IK step parameter:" << ikStep << "(m)";
//        return false;
//    }

//    maxSpeed = rf.check("maxSpeed", yarp::os::Value(DEFAULT_MAX_SPEED), "max speed [m/s]").asFloat64();

//    if (maxSpeed <= 0.0)
//    {
//        yCError(OFS) << "Invalid max speed parameter:" << maxSpeed << "(m/s)";
//        return false;
//    }

//    maxAcceleration = rf.check("maxAcceleration", yarp::os::Value(DEFAULT_MAX_ACCELERATION), "max acceleration [m/s^2]").asFloat64();

//    if (maxAcceleration <= 0.0)
//    {
//        yCError(OFS) << "Invalid max acceleration parameter:" << maxAcceleration << "(m/s^2)";
//        return false;
//    }

//    dryRun = rf.check("dryRun", "process sensor loops, but don't send motion command");

//    if (dryRun)
//    {
//        yCInfo(OFS) << "Dry run mode enabled, robot will perform no motion";
//    }

//    auto sensorFrameRPY = rf.check("sensorFrameRPY", yarp::os::Value::getNullValue(), "sensor frame RPY rotation regarding TCP frame [deg]");

//    if (!parseFrameRotation(sensorFrameRPY, "sensor frame", R_N_sensor))
//    {
//        return false;
//    }

//    auto soleFrameRPY = rf.check("soleFrameRPY", yarp::os::Value::getNullValue(), "sole frame RPY rotation regarding TCP frame [deg]");

//    if (!parseFrameRotation(soleFrameRPY, "sole frame", R_N_sole))
//    {
//        return false;
//    }

    auto localPrefix = rf.check("local", yarp::os::Value(DEFAULT_LOCAL_PREFIX), "local port prefix").asString();

//    // ----- sensor device -----

//    if (!rf.check("sensorName", "remote FT sensor name to connect to via MAS client"))
//    {
//        yCError(OFS) << "Missing parameter: sensorName";
//        return false;
//    }

//    auto sensorName = rf.find("sensorName").asString();

//    if (!rf.check("sensorRemote", "remote FT sensor port to connect to via MAS client"))
//    {
//        yCError(OFS) << "Missing parameter: sensorRemote";
//        return false;
//    }
    auto robotRemote = rf.find("robotRemote").asString();
    auto sensorRemote = rf.find("sensorRemote").asString();

//    yarp::os::Property sensorOptions {
//        {"device", yarp::os::Value("multipleanalogsensorsclient")},
//        {"remote", yarp::os::Value(sensorRemote)},
//        {"local", yarp::os::Value(localPrefix + sensorRemote)},
//        {"timeout", yarp::os::Value(period * 0.5)}
//    };

//    if (!sensorDevice.open(sensorOptions))
//    {
//        yCError(OFS) << "Failed to open sensor device";
//        return false;
//    }

//    if (!sensorDevice.view(sensor))
//    {
//        yCError(OFS) << "Failed to view sensor interface";
//        return false;
//    }

//    sensorIndex = -1;

//    for (auto i = 0; i < sensor->getNrOfSixAxisForceTorqueSensors(); i++)
//    {
//        std::string temp;

//        if (sensor->getSixAxisForceTorqueSensorName(i, temp) && temp == sensorName)
//        {
//            sensorIndex = i;
//            break;
//        }
//    }

//    if (sensorIndex == -1)
//    {
//        yCError(OFS) << "Failed to find sensor with name" << sensorName;
//        return false;
//    }

//    int retry = 0;

//    while (sensor->getSixAxisForceTorqueSensorStatus(sensorIndex) != yarp::dev::MAS_OK)
//    {
//        if (++retry == 10)
//        {
//            yCError(OFS) << "Failed to get first sensor read";
//            return false;
//        }

//        yarp::os::SystemClock::delaySystem(0.1);
//    }

//    // ----- robot device -----

//    if (!rf.check("robotRemote", "remote robot port to connect to"))
//    {
//        yCError(OFS) << "Missing parameter: robotRemote";
//        return false;
//    }

//    auto robotRemote = rf.find("robotRemote").asString();

//    yarp::os::Property robotOptions {
//        {"device", yarp::os::Value("remote_controlboard")},
//        {"remote", yarp::os::Value(robotRemote)},
//        {"local", yarp::os::Value(localPrefix + robotRemote)}
//    };

//    if (!robotDevice.open(robotOptions))
//    {
//        yCError(OFS) << "Failed to open robot device";
//        return false;
//    }



//    if (!robotDevice.view(limits) || !robotDevice.view(mode) || !robotDevice.view(enc) || !robotDevice.view(posd))
//    {
//        yCError(OFS) << "Failed to view robot control interfaces";
//        return false;
//    }

//    int numJoints;
//    enc->getAxes(&numJoints);
//    previousJointPose.resize(numJoints);

//    retry = 0;

//    while (!enc->getEncoders(previousJointPose.data()))
//    {
//        if (++retry == 10)
//        {
//            yCError(OFS) << "Failed to get initial joint pose";
//            return false;
//        }

//        yarp::os::SystemClock::delaySystem(0.1);
//    }

//    yCInfo(OFS) << "Initial joint pose:" << previousJointPose;

//    yarp::os::Bottle bMin, bMax;

//    for (int joint = 0; joint < numJoints; joint++)
//    {
//        double qMin, qMax;

//        if (!limits->getLimits(joint, &qMin, &qMax))
//        {
//            yCError(OFS) << "Unable to retrieve position limits for joint" << joint;
//            return false;
//        }

//        bMin.addFloat64(qMin);
//        bMax.addFloat64(qMax);
//    }

//    yCInfo(OFS) << "Joint position limits:" << bMin.toString() << bMax.toString();

//    if (!mode->setControlModes(std::vector(numJoints, VOCAB_CM_POSITION_DIRECT).data()))
//    {
//        yCError(OFS) << "Failed to set posd control mode";
//        return false;
//    }

//    // ----- solver device -----

//   if (!rf.check("kinematics", "description file for kinematics"))
//   {
//       yCError(OFS) << "Missing parameter: kinematics";
//       return false;
//   }

//   yarp::os::Property solverOptions {
//       {"device", yarp::os::Value("KdlSolver")},
//       {"kinematics", rf.find("kinematics")},
//       {"ikPos", yarp::os::Value("st")},
//       {"invKinStrategy", yarp::os::Value("humanoidGait")},
//       {"quiet", yarp::os::Value::getNullValue()}
//   };

//   solverOptions.put("mins", yarp::os::Value::makeList(bMin.toString().c_str()));
//   solverOptions.put("maxs", yarp::os::Value::makeList(bMax.toString().c_str()));

//   if (!solverDevice.open(solverOptions))
//   {
//       yCError(OFS) << "Failed to open solver device";
//       return false;
//   }

//   if (!solverDevice.view(solver))
//   {
//       yCError(OFS) << "Failed to view solver interface";
//       return false;
//   }



//    // ----- ZMP publisher port -----

//    if (!zmpPort.open(localPrefix + "/zmp:o"))
//    {
//        yCError(OFS) << "Failed to open ZMP publisher port" << zmpPort.getName();
//        return false;
//    }
    robot.initWalkingRobot(robotRemote,localPrefix);
    robot.initSensor(sensorRemote,period);
    data.open(NOMBRE_ARCHIVO2, std::fstream::out);
    data1.open(NOMBRE_ARCHIVO, std::fstream::out);
    data2.open(NOMBRE_ARCHIVO3, std::fstream::out);
    data << "pos x" << DELIMITADOR <<"pos y"  << DELIMITADOR << "pos z"  << DELIMITADOR
            << "turn x" << DELIMITADOR <<"turn y"  << DELIMITADOR << "turn z"  << DELIMITADOR
            << "ZMPx"  << DELIMITADOR <<"ZMPy" << DELIMITADOR << "ZMPz" << DELIMITADOR
            << "targetZMPx"  << DELIMITADOR <<"targetZMPy" << DELIMITADOR << "targetZMPz" <<"\n";
    data1<< "time" << DELIMITADOR <<"RL_x"  << DELIMITADOR << "RL_y"  << DELIMITADOR
            << "RL_z" << DELIMITADOR <<"LL_x"  << DELIMITADOR << "LL_y"  << DELIMITADOR
            << "LL_z"  << DELIMITADOR <<"zmp_x" << DELIMITADOR << "zmp_y" << DELIMITADOR << "zmp_z"<<"\n";
    data2 << "time" << DELIMITADOR <<"ZMP X TARGET"  << DELIMITADOR << "ZMPX"  << DELIMITADOR
          << "ZMPY" << DELIMITADOR <<"ZMPZ"<<"\n";
    first = yarp::os::Time::now();

    double kpx=0.0258;//-0.009815;//7.1;//3.5817;//2.887;//0.4753;//0.4625;//1.6984;
    double kix=0;//-0.07818;//0.0;//1.3198;//0.0459;//0.0;//0.2149;
    double kdx=0.00149;//-0.0003081;//0.4658;//0.0;//0.0;//0.1195;
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
    robot.stat_Left(leftInitial);
    robot.stat_Right(rightInitial);
    return yarp::os::PeriodicThread::setPeriod(period) && yarp::os::PeriodicThread::start();
}







bool a=true;
KDL::Vector error;
double timeLimit=5.0;
double deltax=0.000;
double deltay=0.001;
KDL::Vector target_ZMP;



void OneFootStand::run()
{

//    KDL::Wrench wrench_N; // expressed in TCP frame
//    double H=2.5*1/100;
//    double Px_zmp=0;
//    double Pz_zmp= (wrench_N.torque.data[1]+wrench_N.force.data[2]*H)/wrench_N.force.data[0];
//    double Py_zmp= -(wrench_N.torque.data[2]-wrench_N.force.data[1]*H)/wrench_N.force.data[0];
      KDL::Vector Pzmp;
      robot.getZMP(Pzmp);
      EMA_Exponential_Filter(Pzmp,0.7);

////    EMA_Exponential_Filter(Pzmp,0.3);
      leftInitial[0]+=deltax;
      leftInitial[1]+=deltay;
      rightInitial[0]+=deltax;
      rightInitial[1]+=deltay;
//    yCDebug(OFS)<<Px_zmp<<Py_zmp<<Pz_zmp;

    double now = yarp::os::Time::now()-first;
    if( now<1.0){
        robot.moviRightPos(rightInitial);
        robot.moviLeftPos(leftInitial);
        data1 <<now<< DELIMITADOR2<<rightInitial[0]<<DELIMITADOR2<<rightInitial[1]<<DELIMITADOR2<<rightInitial[2]
             <<DELIMITADOR2<<leftInitial[0]<<DELIMITADOR2<<leftInitial[1]<<DELIMITADOR2<<leftInitial[2]
              <<DELIMITADOR2<<Pzmp[0]<<DELIMITADOR2<<Pzmp[1]<<DELIMITADOR2<<Pzmp[2]<<"\n";
        yCDebug(OFS)<<now;
    }
    else{
        close();
    }
//    if (!solver->fwdKin(previousJointPose,leftInitial)){
//        yCDebug(OFS) << "Error solver";
//    }

//    if(a){
////        yCDebug(OFS)<<"zzzzzzzzzzzzzzzzzzzzzzzzzz";
////        target_ZMP[1]=target_ZMP[1]-0.01/5*0.02;// *copysign(1.0,Pzmp[1]);
////        target_ZMP[2]=target_ZMP[2]-0.01/5*0.02;// *copysign(1.0,Pzmp[2]);
////        target_ZMP[0]=0;
//       target_ZMP[1]=TARGET_ZMP;// *copysign(1.0,Pzmp[1]);
//       target_ZMP[2]=0.0;// *copysign(1.0,Pzmp[2]);
//       target_ZMP[0]=0.0;

//       target_ZMP=Pzmp/Pzmp.Norm();
////       target_ZMP = Pzmp - target_ZMP;
////       yCDebug(OFS)<<"------------------"<<target_ZMP[0]<<target_ZMP[1]<<target_ZMP[2]<<copysign(1.0,Pzmp[2]);
//       a=false;
//    }



//    double ep=0.002;
//    double now = yarp::os::Time::now()-first;
//    KDL::Vector  controlSignal;
//    error=(target_ZMP-Pzmp);
//    std::vector<double> nextPosition;
//   if( now<15.0/*&& sqrt(error[2]*error[2])>=ep*/){
//    double ts=0.02;
//    double velMax=3;//[º/s]
//    double velMax2=0.1/1;//[m/s]
//    double grados_en_Ts=velMax*ts;//[º]
//    double metros_en_Ts=velMax2*ts;
////    double conDegToRad=3.14/180;
//    double limitDegreeMaxTs=grados_en_Ts;//0.0174/50.0; //0.9*ts*velMax*conDegToRad;
//    double limitMetrosMaxTs=metros_en_Ts;
//    double convDegreeToZMP=2.0;//1.0/2.0;//1.0/2.0;//4.0*0.7;//4.0/2;//[grados de giro del robot/m de ZMP]
//    //x_pie=z_base
//    //y_pie=-y_base
//    //z_pie=x_base;

//    //en el pie el zmp esta en el plano z-y
//    //si el angulo es positivo en pierna izquierda debe ser opuesto para pierna izquierda
////    PID.AntiWindup(-limit,limit);
//    controlSignal[0]=-error[2]/100;//0.05*error[2]+0.002*(error[2]>filtro);
//   controlSignal[1]=-error[1]*convDegreeToZMP; //-error[1]*convDegreeToZMP;//*copysign(1.0,leftInitial[1]);
//// yCDebug(OFS)<<copysign(1.0,leftInitial[1]);//<<leftInitial[0]<<leftInitial[1]<<leftInitial[2];
//    if (controlSignal[1]>=limitDegreeMaxTs){
//                yCDebug(OFS) << "Error Saturación";
//            controlSignal[1]=limitDegreeMaxTs;
//        }
//        else if (controlSignal[1]<=-limitDegreeMaxTs){
//             yCDebug(OFS) << "Error Saturación 2";
//            controlSignal[1]= -limitDegreeMaxTs;
//        }
//        if (controlSignal[0]>=limitMetrosMaxTs){
//            yCDebug(OFS) << "Error Saturación 3";
//            controlSignal[0]=limitMetrosMaxTs;
//        }
//        else if (controlSignal[0]<=-limitMetrosMaxTs){
//             yCDebug(OFS) << "Error Saturación 4";
//            controlSignal[0]= -limitMetrosMaxTs;
//        }



//    //leftInitial[0]-=controlSignal[0];
//    if(!solver->invKin(leftInitial, previousJointPose, nextPosition))
//    {
//         yCDebug(OFS) <<"Error inverse kinematic ";
//    }





//        //     yCDebug(OFS)<<nextPosition[0]<<nextPosition[1]<<nextPosition[2]<<nextPosition[3]<<nextPosition[4]<<nextPosition[nextPosition.size()-1];
////    nextPosition[0]=0.0;
//    //y en el tcp es -y en la base
//    nextPosition[5]-=controlSignal[1];// *copysign(1.0,leftInitial[1]);
//    //derecha positivo
////    yCDebug(OFS)<<nextPosition[0]<<nextPosition[1]<<nextPosition[2]<<nextPosition[3]<<nextPosition[4]<<nextPosition[nextPosition.size()-1];


//    if(checkLimits(nextPosition)){
////        posd->setPositions(nextPosition.data());
//    }
//    else{
//        yCDebug(OFS) << "Valores articulares fuera de rango";
//    }
//    printAndSave(nextPosition,Pzmp,target_ZMP,controlSignal,previousJointPose);
//    printAndSave(controlSignal[1],controlSignal[0],Pzmp,target_ZMP);
//    printAndSave(now, Pzmp,target_ZMP);

//   }
//   else
//   {
//            yCDebug(OFS) << "Fin del programa";close();
//   }



//    if(!enc->getEncoders(previousJointPose.data()))
//    {
//         yCError(OFS) << "Failed to get initial joint pose";
//    }

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
    yarp::os::PeriodicThread::stop();
    return 0 ;//|| iCartesianControl->stopControl();
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
    return true;
}


void OneFootStand::saveData(System system)
{
        data << system.input.x << DELIMITADOR <<system.input.y << DELIMITADOR << system.input.z << DELIMITADOR
             << system.input.gx << DELIMITADOR <<system.input.gy << DELIMITADOR << system.input.gz << DELIMITADOR
             << system.output.x << DELIMITADOR <<system.output.y << DELIMITADOR << system.output.z << DELIMITADOR
             << system.target.x << DELIMITADOR <<system.target.y << DELIMITADOR << system.target.z << "\n";


}

