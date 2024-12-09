// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup gait-experiments-programs
 * @defgroup stableGait stableGait
 * @brief An attempt to perform gait on a humanoid robot with simple, non-dynamic stability assumptions.
 *
 * <b>Building</b>
 *
\verbatim
mkdir build; cd build; cmake ..
make -j$(nproc)
\endverbatim
 *
 * <b>Running example with teoSim</b>
 * First we must run a YARP name server if it is not running in our current namespace:
 *
\verbatim
[on terminal 1] yarp server
\endverbatim
 *
 * The following is an example for the simulated robot's legs:
 *
\verbatim
[on terminal 2] teoSim
[on terminal 3] yarpdev --device BasicCartesianControl --name /teoSim/leftLeg/CartesianControl --kinematics teo-leftLeg.ini --local /BasicCartesianControl/teoSim/leftLeg --remote /teoSim/leftLeg --ik st --invKinStrategy humanoidGait
[on terminal 4] yarpdev --device BasicCartesianControl --name /teoSim/rightLeg/CartesianControl --kinematics teo-rightLeg.ini --local /BasicCartesianControl/teoSim/rightLeg --remote /teoSim/rightLeg --ik st --invKinStrategy humanoidGait
[on terminal 5] stableGait --distance 1.0 --dry
\endverbatim
 */

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <yarp/os/LogStream.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/Timer.h>

#include <yarp/dev/PolyDriver.h>

#include <kdl/frames.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/utilities/error.h>

#include <ICartesianControl.h>
#include <KdlVectorConverter.hpp>

#include "GaitSpecs.hpp"
#include "StepGenerator.hpp"
#include "LIPMTrajectoryGenerator.hpp"
#include "TrajectoryGenerator.hpp"
#include "TargetBuilder.hpp"
#include "fcontrol.h"
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/path_circle.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include "walkingRobot.hpp"
#include <unistd.h>

constexpr auto DEFAULT_TRAJ_VEL = 0.175;     // [m/s]
constexpr auto DEFAULT_TRAJ_ACC = 5.0;       // [m/s^2]
constexpr auto DEFAULT_CMD_PERIOD = 0.02;    // [s]
constexpr auto DEFAULT_FOOT_LENGTH = 0.245;  // [m]
constexpr auto DEFAULT_FOOT_WIDTH = 0.12;    // [m]
constexpr auto DEFAULT_FOOT_LIFT = 0.01;    // [m]
constexpr auto DEFAULT_GAIT_SEP = 0.125;      // [m]
constexpr auto DEFAULT_GAIT_SQUAT= 0.82;     // [m]
constexpr auto DEFAULT_GAIT_STEP=0.04;      // [m]
constexpr auto DEFAULT_LOCAL_PREFIX = "/oneFootStand";

//constexpr auto DEFAULT_TRAJ_VEL = 0.175;     // [m/s]
//constexpr auto DEFAULT_TRAJ_ACC = 5.0;       // [m/s^2]
//constexpr auto DEFAULT_CMD_PERIOD = 0.02;    // [s]
//constexpr auto DEFAULT_FOOT_LENGTH = 0.245;  // [m]
//constexpr auto DEFAULT_FOOT_WIDTH = 0.12;    // [m]
//constexpr auto DEFAULT_FOOT_LIFT = 0.01;    // [m]
//constexpr auto DEFAULT_GAIT_SEP = 0.125;      // [m]
//constexpr auto DEFAULT_GAIT_SQUAT= 0.82;     // [m]
//constexpr auto DEFAULT_GAIT_STEP=0.05;      // [m]
//constexpr auto DEFAULT_LOCAL_PREFIX = "/oneFootStand";

const std::string NOMBRE_ARCHIVO_RIGHT = "/home/gerson/MATLAB-Drive/trayectoria_pabloLR.txt";
const std::string NOMBRE_ARCHIVO_LEFT = "/home/gerson/MATLAB-Drive/trayectoria_pabloLL.txt";


const std::string NOMBRE_ARCHIVO = "/home/gerson/MATLAB-Drive/Caminata/TrajectoriaGeneradaGlobal.txt";
const std::string NOMBRE_ARCHIVO2 = "/home/gerson/MATLAB-Drive/Caminata/TrajectoriaGeneradaLocal.txt";
const std::string NOMBRE_ARCHIVO3 = "/home/gerson/MATLAB-Drive/Caminata/TrayectoriaArticulares.txt";
const std::string NOMBRE_ARCHIVO4= "/home/gerson/MATLAB-Drive/Caminata/TrayectoriaVel.txt";
const std::string DELIMITADOR =" ";

std::ifstream trajR,trajL;

int main(int argc, char * argv[])
{
    // Find YARP network.

    yarp::os::Network yarp;
    trajL.open(NOMBRE_ARCHIVO_LEFT);
    if (!trajL.is_open()) {
        std::cerr << "No se pudo abrir el archivo" << std::endl;
        return 1;
    }
    trajR.open(NOMBRE_ARCHIVO_RIGHT);
    if (!trajR.is_open()) {
        std::cerr << "No se pudo abrir el archivo" << std::endl;
        return 1;
    }

    if (!yarp::os::Network::checkNetwork())
    {
        yError() << "Please start a yarp name server first";
        return 1;
    }

    // Parse arguments.

    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);

//    yDebug() << rf.toString();

    double distance = rf.check("distance", yarp::os::Value(0.25), "distance to travel [m]").asFloat64();
    double trajVel = rf.check("vel", yarp::os::Value(DEFAULT_TRAJ_VEL), "velocity [m/s]").asFloat64();
    double trajAcc = rf.check("acc", yarp::os::Value(DEFAULT_TRAJ_ACC), "acceleration [m/s^2]").asFloat64();
    double period = rf.check("period", yarp::os::Value(DEFAULT_CMD_PERIOD), "command period [s]").asFloat64();
    double footLength = rf.check("length", yarp::os::Value(DEFAULT_FOOT_LENGTH), "foot length [m]").asFloat64();
    double footWidth = rf.check("width", yarp::os::Value(DEFAULT_FOOT_WIDTH), "foot width [m]").asFloat64();
    double footLift = rf.check("lift", yarp::os::Value(DEFAULT_FOOT_LIFT), "lift [m]").asFloat64();
    double gaitSep = rf.check("sep", yarp::os::Value(DEFAULT_GAIT_SEP), "foot separation [m]").asFloat64();
    double gaitStep =rf.check("step", yarp::os::Value(DEFAULT_GAIT_STEP), "lenght Step [m]").asFloat64();
    double gaitSquat = rf.check("sep", yarp::os::Value(DEFAULT_GAIT_SQUAT), "Squat hight of Base robot[m]").asFloat64();

    bool dryRun = rf.check("dry", "dry run");


    auto robotRemote = rf.find("robotRemote").asString();
    auto sensorRemote = rf.find("sensorRemote").asString();
    auto sensorIMURemote = rf.find("sensorIMURemote").asString();
    walkingRobot *robot;
    robot=new walkingRobot();
    robot->initWalkingRobot(robotRemote,DEFAULT_LOCAL_PREFIX);
//    robot->initSensor(sensorRemote,period);
//    robot->initIMU(sensorIMURemote,period);

    std::vector<double> leftInitial,rightInitial;
    robot->stat_Left(leftInitial);
    robot->stat_Right(rightInitial);
    leftInitial[2] += KDL::epsilon; // initial pose is hard to attain
    rightInitial[2] += KDL::epsilon; // initial pose is hard to attain
    TargetBuilder::Targets pointsLeft,pointsRight;


    double paso=0.85;
    double offset=0.12;
    std::string linea1;
    while (std::getline(trajL, linea1)) {  // Leer cada línea del archivo
        std::istringstream stream(linea1);   // Convertir la línea en un stream de entrada
        std::vector<double> vectorFila;        // Vector para almacenar los valores de la línea actual
        double numero;

//        while (stream >> numero) {  // Leer cada número de la línea
//            vectorFila.push_back(numero);   // Agregar el número al vector de la línea
//        }
        stream >> numero; // Leer cada número de la línea
        vectorFila.push_back(numero*paso);
        stream >> numero; // Leer cada número de la línea
        vectorFila.push_back(numero);
        stream >> numero; // Leer cada número de la línea
        vectorFila.push_back(numero+offset);

        vectorFila.push_back(leftInitial[3]);
        vectorFila.push_back(leftInitial[4]);
        vectorFila.push_back(leftInitial[5]);
        pointsLeft.push_back(vectorFila);  // Agregar el vector de la línea al vector de vectores
    }

    trajL.close();  // Cerrar el archivo
    std::string linea2;

    while (std::getline(trajR, linea2)) {  // Leer cada línea del archivo
        std::istringstream stream(linea2);   // Convertir la línea en un stream de entrada
        std::vector<double> vectorFila;        // Vector para almacenar los valores de la línea actual
        double numero;

        stream >> numero; // Leer cada número de la línea
        vectorFila.push_back(numero*paso);
        stream >> numero; // Leer cada número de la línea
        vectorFila.push_back(numero);
        stream >> numero; // Leer cada número de la línea
        vectorFila.push_back(numero+offset);


        stream >> numero; // Leer cada número de la línea
        vectorFila.push_back(rightInitial[3]);
        stream >> numero; // Leer cada número de la línea
        vectorFila.push_back(rightInitial[4]);
        stream >> numero; // Leer cada número de la línea
        vectorFila.push_back(rightInitial[5]);


        pointsRight.push_back(vectorFila);  // Agregar el vector de la línea al vector de vectores

    }
    trajR.close();  // Cerrar el archivo

    cout<<" left intial points "<<leftInitial[0]<<" "<<leftInitial[1]<<" "<<leftInitial[2]<<" "<<leftInitial[3]<<" "<<leftInitial[4]<<" "<<leftInitial[5]<<endl;
    cout<<" right intial points "<<rightInitial[0]<<" "<<rightInitial[1]<<" "<<rightInitial[2]<<" "<<rightInitial[3]<<" "<<rightInitial[4]<<" "<<rightInitial[5]<<endl;


    cout<<" left target points "<<pointsLeft[0][0]<<" "<<pointsLeft[0][1]<<" "<<pointsLeft[0][2]<<" "<<pointsLeft[0][3]<<" "<<pointsLeft[0][4]<<" "<<pointsLeft[0][5]<<endl;
    cout<<" right target points "<<pointsRight[0][0]<<" "<<pointsRight[0][1]<<" "<<pointsRight[0][2]<<" "<<pointsRight[0][3]<<" "<<pointsRight[0][4]<<" "<<pointsRight[0][5]<<endl;



      std::vector<std::vector<double>> traj_ini_left=robot->movlLeftPos(pointsLeft[0],period*10e4);
      std::vector<std::vector<double>> traj_ini_right=robot->movlRightPos(pointsRight[0],period*10e4);




//      for (int i = 0; i < traj_ini_left.size(); i++)
//      {
//          robot->moviLeftPos(traj_ini_left[i]);
//          robot->moviRightPos(traj_ini_right[i]);
//          usleep(period*10e4);
//      }
//      for (int i =0;i<=10;i++){
//          for (int i = 0; i < pointsLeft.size(); i++)
//          {
//              robot->moviLeftPos(pointsLeft[i]);
//              robot->moviRightPos(pointsRight[i]);
//              usleep(period*10e4);
//          }
//      }

    if (distance <= 0.0)
    {
        yError() << "Illegal argument: '--distance' must be greater than '0', was:" << distance;
        return 1;
    }

    if (trajVel <= 0.0)
    {
        yError() << "Illegal argument: '--vel' must be greater than '0', was:" << trajVel;
        return 1;
    }

    if (trajAcc <= 0.0)
    {
        yError() << "Illegal argument: '--acc' must be greater than '0', was:" << trajAcc;
        return 1;
    }

    if (period <= 0.0)
    {
        yError() << "Illegal argument: '--period' must be greater than '0', was:" << period;
        return 1;
    }

    if (footLength <= 0.0)
    {
        yError() << "Illegal argument: '--length' must be greater than '0', was:" << footLength;
        return 1;
    }

    if (footWidth <= 0.0)
    {
        yError() << "Illegal argument: '--width' must be greater than '0', was:" << footWidth;
        return 1;
    }


        if (gaitStep <= 0.0)
        {
            yError() << "Illegal argument: '--step' must be greater than '0', was:" << gaitStep;
            return 1;
        }
        if (gaitSquat <= 0.0)
        {
            yError() << "Illegal argument: '--squat' must be greater than '0', was:" << gaitSquat;
            return 1;
        }



    if (footLift < 0.0)
    {
        yError() << "Illegal argument: '--lift' must be greater than or equal to '0', was:" << footLift;
        return 1;
    }

    if (gaitSep <= 0.0)
    {
        yError() << "Illegal argument: '--sep' must be greater than '0', was:" << gaitSep;
        return 1;
    }



//     Initialize specs and components.







    FootSpec footSpec;
    footSpec.length = footLength;
    footSpec.width = footWidth;
    footSpec.lift = footLift;

    GaitSpec gaitSpec;
    gaitSpec.sep = gaitSep;
    gaitSpec.step = gaitStep;
    gaitSpec.squat = gaitSquat;

    TrajectoryGenerator a(robot,period);
    a.configure(distance,gaitSpec,footSpec);
    TargetBuilder::Targets pointsLeft2, pointsRight2, COM2;
    a.generate(pointsLeft2, pointsRight2, COM2);




    a.write(NOMBRE_ARCHIVO,NOMBRE_ARCHIVO2,NOMBRE_ARCHIVO3,NOMBRE_ARCHIVO4,DELIMITADOR);
    // Configure worker.
    double maxDuration=a.duration();
    if (!dryRun)
    {
        double a=1;
        maxDuration*=a;
        period=0.02*a;
        yarp::os::TimerSettings timerSettings(period, maxDuration / period, maxDuration);

        yarp::os::Timer::TimerCallback callback = [&](const yarp::os::YarpTimerEvent & event)
        {
            robot->moviLeftPos(pointsLeft2[event.runCount]);
            robot->moviRightPos(pointsRight2[event.runCount]);

            return true;
        };

        yarp::os::Timer timer(timerSettings, callback, true);

        // Execute trajectory.

        if (timer.start())
        {
            yarp::os::Time::delay(maxDuration);
            timer.stop();
        }
    }
    delete robot;
    cout<<"finalizamos el programa"<<endl;
    return 0;
}
