
#ifndef __WALKING_ROBOT_HPP__
#define __WALKING_ROBOT_HPP__

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IPositionDirect.h>
#include <ICartesianSolver.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IControlLimits.h>
#include <ICartesianControl.h>
#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <KdlVectorConverter.hpp> // TODO: unused
#include <yarp/dev/MultipleAnalogSensorsInterfaces.h>
#include <yarp/dev/IControlMode.h>

class walkingRobot{
public:
    walkingRobot(){}
    ~walkingRobot();
    bool initWalkingRobot(std::string robotRemote, std::string local);
    bool initSensor(std::string sensorRemote,double period,double index_right=0,double index_left=2);
    yarp::dev::PolyDriver robotDevice_Right;

    yarp::dev::PolyDriver robotDevice_Left;
    yarp::dev::PolyDriver solverDevice_Right;
    yarp::dev::PolyDriver solverDevice_Left;
    roboticslab::ICartesianSolver * solver_Right;
    roboticslab::ICartesianSolver * solver_Left;
    std::vector<double> previousJointPose_Right;
    std::vector<double> previousJointPose_Left;
    yarp::dev::IEncoders * enc_Right;
    yarp::dev::IControlLimits * limits_Right;
    yarp::dev::IEncoders * enc_Left;
    yarp::dev::IControlLimits * limits_Left;
    yarp::os::ResourceFinder rf;
    int sensorIndexRight=2;
    int sensorIndexLeft=0;
    int numJoints;
    int PHASE_WALKING=0;
    yarp::dev::PolyDriver sensorDevice,sensorDeviceIMU;
    yarp::dev::ISixAxisForceTorqueSensors * sensor;
    yarp::dev::IThreeAxisGyroscopes * gyros;
    yarp::dev::IOrientationSensors * orientationBase;
    yarp::dev::IThreeAxisLinearAccelerometers * accel;
    KDL::Rotation R_N_sensor;
    KDL::Rotation R_N_sole;
    std::string local,robotRemote;
     yarp::dev::IControlMode * modeRight;
      yarp::dev::IControlMode * modeLeft;
      yarp::dev::IPositionDirect * posdRight;
      yarp::dev::IPositionDirect * posdLeft;
    yarp::os::Bottle bMin_right, bMax_right;
    yarp::os::Bottle bMin_left, bMax_left;
    YARP_LOG_COMPONENT(WR, "walkingRobot")
    bool readSensor(KDL::Wrench &wrench_N_Right, KDL::Wrench &wrench_N_Left);
    bool getZMP(KDL::Vector &ZMP);
    KDL::Vector calculateZMP(KDL::Wrench &wrench_N_Right, KDL::Wrench &wrench_N_Left);
    bool parseFrameRotation(const yarp::os::Value &value, const std::string &name, KDL::Rotation &rot);
    bool checkLimitsLeft(std::vector<double> &JointPose);
    bool checkLimitsRight(std::vector<double> &JointPose);
    bool moviLeftJoint(std::vector<double> &JointPose);
    bool moviRightJoint(std::vector<double> &JointPose);
    bool moviLeftPos(std::vector<double> &targetPosition);
    bool moviRightPos(std::vector<double> &targetPosition);
//    bool movlLeftPos(std::vector<double> &cartesianPosition, double period_ms=2000);
//    bool movlRightPos(std::vector<double> &cartesianPosition, double period_ms=2000);
    std::vector<std::vector<double>> movlLeftPos(std::vector<double> &cartesianPosition, double period_ms=2000);
    std::vector<std::vector<double>> movlRightPos(std::vector<double> &cartesianPosition, double period_ms=2000);
    bool inv_Right(std::vector<double> &cartesianPosition,std::vector<double> &JointPostion);
    bool inv_Left(std::vector<double> &cartesianPosition,std::vector<double> &JointPostion);
    bool stat_Right(std::vector<double> &currentCartesianPosition);
    bool stat_Left(std::vector<double> &currentCartesianPosition);
    bool close();
    bool initIMU(std::string sensorIMURemote, double period);
    bool readIMU(std::vector<double> acelerationCOM, std::vector<double> orientationCOM, std::vector<double> vel_angCOM);
    bool setLimitsRight(double);
    bool setLimitsLeft(double);

};

#endif
