#include "walkingRobot.hpp"
#include <string.h>

#include <unistd.h>
walkingRobot::~walkingRobot(){
    this->close();
}

bool walkingRobot::initWalkingRobot(std::string robotRemote, std::string local){
    this->robotRemote=robotRemote;
    this->local=local;
    // ----- robot device -----
    yarp::os::Property robotOptions {
        {"device", yarp::os::Value("remote_controlboard")},
        {"remote", yarp::os::Value(robotRemote+"/rightLeg")},
        {"local", yarp::os::Value(local + robotRemote+"/rightLeg")}
    };

    if (!robotDevice_Right.open(robotOptions))
    {
        yCError(WR) << "Failed to open robot device";
        return false;
    }


    if (!robotDevice_Right.view(enc_Right) || !robotDevice_Right.view(limits_Right) || !robotDevice_Right.view(posdRight) || !robotDevice_Right.view(modeRight))
    {
        yCError(WR) << "Failed to view robot control interfaces";
        return false;
    }


    enc_Right->getAxes(&numJoints);
    previousJointPose_Right.resize(numJoints);

    int retry = 0;

    while (!enc_Right->getEncoders(previousJointPose_Right.data()))
    {
        if (++retry == 10)
        {
            yCError(WR) << "Failed to get initial joint pose";
            return false;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }



    for (int joint = 0; joint < numJoints; joint++)
    {
        double qMin, qMax;

        if (!limits_Right->getLimits(joint, &qMin, &qMax))
        {
            yCError(WR) << "Unable to retrieve position limits for joint" << joint;
            return false;
        }

        bMin_right.addFloat64(qMin);
        bMax_right.addFloat64(qMax);
    }
    yCInfo(WR) << "Joint position limits:"<< bMin_right.toString() << bMax_right.toString();

    if (!modeRight->setControlModes(std::vector(numJoints, VOCAB_CM_POSITION_DIRECT).data()))
    {
        yCError(WR) << "Failed to set posd control mode";
        return false;
    }



    yarp::os::Property robotOptions2 {
        {"device", yarp::os::Value("remote_controlboard")},
        {"remote", yarp::os::Value(robotRemote+"/leftLeg")},
        {"local", yarp::os::Value(local + robotRemote+"/leftLeg")}
    };

    if (!robotDevice_Left.open(robotOptions2))
    {
        yCError(WR) << "Failed to open robot device";
        return false;
    }



    if (!robotDevice_Left.view(enc_Left) || !robotDevice_Left.view(limits_Left) || !robotDevice_Left.view(posdLeft) || !robotDevice_Left.view(modeLeft))
    {
        yCError(WR) << "Failed to view robot control interfaces";
        return false;
    }



    enc_Left->getAxes(&numJoints);
    previousJointPose_Left.resize(numJoints);

    retry = 0;

    while (!enc_Left->getEncoders(previousJointPose_Left.data()))
    {
        if (++retry == 10)
        {
            yCError(WR) << "Failed to get initial joint pose";
            return false;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }



    for (int joint = 0; joint < numJoints; joint++)
    {
        double qMin, qMax;

        if (!limits_Left->getLimits(joint, &qMin, &qMax))
        {
            yCError(WR) << "Unable to retrieve position limits for joint" << joint;
            return false;
        }

        bMin_left.addFloat64(qMin);
        bMax_left.addFloat64(qMax);
    }
    yCInfo(WR) << "Joint position limits:" << bMin_left.toString() << bMax_left.toString();

    if (!modeLeft->setControlModes(std::vector(numJoints, VOCAB_CM_POSITION_DIRECT).data()))
    {
        yCError(WR) << "Failed to set posd control mode";
        return false;
    }
    // ----- solver device -----

   yarp::os::Property solverOptions_Left {
       {"device", yarp::os::Value("KdlSolver")},
       {"kinematics", yarp::os::Value("teo-leftLeg.ini")},
       {"ikPos", yarp::os::Value("st")},
       {"invKinStrategy", yarp::os::Value("humanoidGait")},
       {"quiet", yarp::os::Value::getNullValue()}
   };

   solverOptions_Left.put("mins", yarp::os::Value::makeList(bMin_left.toString().c_str()));
   solverOptions_Left.put("maxs", yarp::os::Value::makeList(bMax_left.toString().c_str()));

   if (!solverDevice_Left.open(solverOptions_Left))
   {
       yCError(WR) << "Failed to open solver device";
       return false;
   }

   if (!solverDevice_Left.view(solver_Left))
   {
       yCError(WR) << "Failed to view solver interface";
       return false;
   }

//------------------------------------------------------------------------

   yarp::os::Property solverOptions_Right {
       {"device", yarp::os::Value("KdlSolver")},
       {"kinematics", yarp::os::Value("teo-rightLeg.ini")},
       {"ikPos", yarp::os::Value("st")},
       {"invKinStrategy", yarp::os::Value("humanoidGait")},
       {"quiet", yarp::os::Value::getNullValue()}
   };

   solverOptions_Right.put("mins", yarp::os::Value::makeList(bMin_right.toString().c_str()));
   solverOptions_Right.put("maxs", yarp::os::Value::makeList(bMax_right.toString().c_str()));

   if (!solverDevice_Right.open(solverOptions_Right))
   {
       yCError(WR) << "Failed to open solver device";
       return false;
   }

   if (!solverDevice_Right.view(solver_Right))
   {
       yCError(WR) << "Failed to view solver interface";
       return false;
   }
   return true;
}

bool walkingRobot::initSensor(std::string sensorRemote,double period,double index_right,double index_left)
{

    // ----- sensor device -----

    auto sensorFrameRPY = *yarp::os::Value::makeValue("(0 -90 0)");

    if (!parseFrameRotation(sensorFrameRPY, "sensor frame", R_N_sensor))
    {
        return false;
    }
    yarp::os::Property sensorOptions {
        {"device", yarp::os::Value("multipleanalogsensorsclient")},
        {"remote", yarp::os::Value(sensorRemote)},
        {"local", yarp::os::Value(local + sensorRemote)},
        {"timeout", yarp::os::Value(period * 0.5)}
    };

    if (!sensorDevice.open(sensorOptions))
    {
        yCError(WR) << "Failed to open sensor device";
        return false;
    }

    if (!sensorDevice.view(sensor))
    {
        yCError(WR) << "Failed to view sensor interface";
        return false;
    }
    double sensorIndex=0;

    int retry = 0;

    while (sensor->getSixAxisForceTorqueSensorStatus(sensorIndex) != yarp::dev::MAS_OK)
    {
        if (++retry == 10)
        {
            yCError(WR) << "Failed to get first sensor read";
            return false;
        }

        yarp::os::SystemClock::delaySystem(0.1);
    }
    return true;
}
bool walkingRobot::readSensor(KDL::Wrench & wrench_N_Right,KDL::Wrench & wrench_N_Left)
{
    yarp::sig::Vector RightSensor_ft;yarp::sig::Vector LeftSensor_ft;
    double timestamp;
    if (!sensor->getSixAxisForceTorqueSensorMeasure(sensorIndexLeft, LeftSensor_ft, timestamp) && !sensor->getSixAxisForceTorqueSensorMeasure(sensorIndexRight, RightSensor_ft, timestamp))
    {
        yCWarning(WR) << "Failed to retrieve current sensor measurements";
        return false;
    }
    if (!sensor->getSixAxisForceTorqueSensorMeasure(sensorIndexRight, RightSensor_ft, timestamp))
    {
        yCWarning(WR) << "Failed to retrieve current sensor measurements";
        return false;
    }
    KDL::Wrench currentWrench_RightLegSensor (
        KDL::Vector(RightSensor_ft[0], RightSensor_ft[1], RightSensor_ft[2]), // force
        KDL::Vector(RightSensor_ft[3], RightSensor_ft[4], RightSensor_ft[5]) // torque
    );

    KDL::Wrench currentWrench_LeftLegSensor (
        KDL::Vector(LeftSensor_ft[0], LeftSensor_ft[1], LeftSensor_ft[2]), // force
        KDL::Vector(LeftSensor_ft[3], LeftSensor_ft[4], LeftSensor_ft[5]) // torque
    );
    wrench_N_Right = R_N_sensor * currentWrench_RightLegSensor;
    wrench_N_Left = R_N_sensor * currentWrench_LeftLegSensor;
    return true;
}


bool walkingRobot::initIMU(std::string sensorIMURemote, double period)
{
    yarp::os::Property sensorOptions {
        {"device", yarp::os::Value("multipleanalogsensorsclient")},
        {"remote", yarp::os::Value(sensorIMURemote)},
        {"local", yarp::os::Value(local + sensorIMURemote)},
        {"timeout", yarp::os::Value(period * 0.5)}
    };
    if (!sensorDeviceIMU.open(sensorOptions))
    {
        yCError(WR) << "Failed to open sensor device";
        return false;
    }

    if (!sensorDeviceIMU.view(gyros))
    {
        yCError(WR) << "Failed to view gyros interface";
        return false;
    }
    if (!sensorDeviceIMU.view(accel))
    {
        yCError(WR) << "Failed to view accel interface";
        return false;
    }
    if (!sensorDeviceIMU.view(orientationBase))
    {
        yCError(WR) << "Failed to view accel interface";
        return false;
    }
    return true;
}
bool walkingRobot::readIMU(std::vector<double> acelerationCOM, std::vector<double> orientationCOM, std::vector<double> vel_angCOM)
{
    yarp::sig::Vector _acelerationCOM;yarp::sig::Vector _orientationCOM;yarp::sig::Vector _vel_angCOM;
    double timestamp;
    if (!gyros->getThreeAxisGyroscopeMeasure(0,_vel_angCOM,timestamp))
    {
        yCWarning(WR) << "Failed to retrieve current IMU sensor measurements";
        return false;
    }
    if (!accel->getThreeAxisLinearAccelerometerMeasure(0,_acelerationCOM,timestamp))
    {
        yCWarning(WR) << "Failed to retrieve current IMU sensor measurements";
        return false;
    }
    if (!orientationBase->getOrientationSensorMeasureAsRollPitchYaw(0,_orientationCOM,timestamp))
    {
        yCWarning(WR) << "Failed to retrieve current IMU sensor measurements";
        return false;
    }
    acelerationCOM=std::vector<double>(_acelerationCOM.data(), _acelerationCOM.data() + _acelerationCOM.size());
    orientationCOM=std::vector<double>(_vel_angCOM.data(), _vel_angCOM.data() + _vel_angCOM.size());
    acelerationCOM=std::vector<double>(_orientationCOM.data(), _orientationCOM.data() + _orientationCOM.size());
    return true;
}

bool walkingRobot::getZMP(KDL::Vector &ZMP)
{
    KDL::Wrench wrench_N_Left,wrench_N_Right; // expressed in TCP frame

    if (!readSensor(wrench_N_Right,wrench_N_Left))
    {
        yCWarning(WR) << "it isn't read sensor:";
        return false;
    }
    ZMP=calculateZMP(wrench_N_Right,wrench_N_Left);
    return true;
}

KDL::Vector walkingRobot::calculateZMP(KDL::Wrench &wrench_N_Right, KDL::Wrench &wrench_N_Left)
{
    std::vector<double> leftInitial,rightInitial;
    double zmpLeftDefined = 0.0, zmpRightDefined = 0.0;
    KDL::Frame ZMP_Left_TCP,ZMP_Right_TCP;
    double H=2.5*1/100;
//    yCDebug(WR)<<"fuerza izquierda"<<wrench_N_Left.force.data[0]<<"Derecha"<<wrench_N_Right.force.data[0];
    //ZMP IN left leg
    if(wrench_N_Left.force.data[0] < 1.00)
        zmpLeftDefined = 0.0;
    else
    {

        double Px_zmp_l=0;
        double Pz_zmp_l= (wrench_N_Left.torque.data[1]+wrench_N_Left.force.data[2]*H)/wrench_N_Left.force.data[0];
        double Py_zmp_l= -(wrench_N_Left.torque.data[2]-wrench_N_Left.force.data[1]*H)/wrench_N_Left.force.data[0];
    //    KDL::Vector Pzmp_Left(Px_zmp_l,Py_zmp_l,Pz_zmp_l);
        ZMP_Left_TCP=KDL::Frame(KDL::Vector(Px_zmp_l,Py_zmp_l,Pz_zmp_l));
        zmpLeftDefined=1.0;
//         yCDebug(WR) <<"Pierna izquierda sobre el suelo";
//         yCDebug(WR)<<Px_zmp_l<<Py_zmp_l<<Pz_zmp_l;
    }

    //ZMP in right leg
    if(wrench_N_Right.force.data[0] < 1.00)
        zmpRightDefined = 0.0;
    else
    {
        double Px_zmp_r=0;
        double Pz_zmp_r= (wrench_N_Right.torque.data[1]+wrench_N_Right.force.data[2]*H)/wrench_N_Right.force.data[0];
        double Py_zmp_r= -(wrench_N_Right.torque.data[2]-wrench_N_Right.force.data[1]*H)/wrench_N_Right.force.data[0];
        KDL::Vector Pzmp_Right(Px_zmp_r,Py_zmp_r,Pz_zmp_r);
        ZMP_Right_TCP=KDL::Frame(KDL::Vector(Px_zmp_r,Py_zmp_r,Pz_zmp_r));
        zmpRightDefined=1.0;
//         yCDebug(WR) <<"Pierna derecha sobre el suelo";
//         yCDebug(WR)<<Px_zmp_r<<Py_zmp_r<<Pz_zmp_r;
    }
    double totalZ = wrench_N_Left.force.data[0]*zmpLeftDefined+ wrench_N_Right.force.data[0]*zmpRightDefined;

    if (zmpRightDefined==1.0 && zmpLeftDefined==1.0){PHASE_WALKING=0;}
    else if(zmpRightDefined==0.0 && zmpLeftDefined==1.0){PHASE_WALKING=1;}
    else if(zmpRightDefined==1.0 && zmpLeftDefined==0.0){PHASE_WALKING=-1;}
    else {PHASE_WALKING=2;}
    if(!enc_Right->getEncoders(previousJointPose_Right.data()))
    {
         yCError(WR) << "Failed to get initial joint pose";

    }
    solver_Right->fwdKin(previousJointPose_Right,rightInitial);
    auto right_leg=roboticslab::KdlVectorConverter::vectorToFrame(rightInitial);
    if(!enc_Left->getEncoders(previousJointPose_Left.data()))
    {
         yCError(WR) << "Failed to get initial joint pose";

    }
    solver_Left->fwdKin(previousJointPose_Left,leftInitial);
    auto left_leg=roboticslab::KdlVectorConverter::vectorToFrame(leftInitial);

//    yDebug()<<left_leg.p.data;
    KDL::Frame ZMP_Left_BASE=left_leg*ZMP_Left_TCP;
    KDL::Frame ZMP_Right_BASE=right_leg*ZMP_Right_TCP;


    auto ZMP=(wrench_N_Left.force.data[0]*zmpLeftDefined/totalZ)*ZMP_Left_BASE.p+
            (wrench_N_Right.force.data[0]*zmpRightDefined/totalZ)*ZMP_Right_BASE.p;
//    yInfo()<<"ZMP GLOBAL"<<leftInitial[0]<<leftInitial[1]<<leftInitial[2];
    return ZMP;
}
bool walkingRobot::parseFrameRotation(const yarp::os::Value & value, const std::string & name, KDL::Rotation & rot)
{
    if (!value.isNull())
    {
        if (!value.isList() || value.asList()->size() != 3)
        {
            yCError(WR) << "Parameter" << name << "must be a list of 3 doubles";
            return false;
        }

        yCInfo(WR) << name << "RPY [deg]:" << value.toString();

        auto roll = value.asList()->get(0).asFloat64() * KDL::deg2rad;
        auto pitch = value.asList()->get(1).asFloat64() * KDL::deg2rad;
        auto yaw = value.asList()->get(2).asFloat64() * KDL::deg2rad;

        // sequence (old axes): 1. R_x(roll), 2. R_y(pitch), 3. R_z(yaw)
        rot = KDL::Rotation::RPY(roll, pitch, yaw);
    }
    else
    {
        yCInfo(WR) << "Using no" << name;
        rot = KDL::Rotation::Identity();
    }

    return true;
}
bool walkingRobot::checkLimitsRight(std::vector<double> &JointPose){


    for(int i=0;i<JointPose.size();i++){
        double qmin=bMin_right.get(i).asFloat64();
        double qmax=bMax_right.get(i).asFloat64() ;
      if(qmin>JointPose[i] || qmax<JointPose[i]){
          yCError(WR) <<"N de articulacion: "<<i<<", Valor:"<<JointPose[i]<<", qmin="<<qmin<<", qmax="<<qmax;
          return false;
      }
    }
    return true;
}
std::vector<std::vector<double>> walkingRobot::movlLeftPos(std::vector<double> &targetPosition,double period_ms){

//    control_Left->movl(cartesianPosition);
     std::vector<double> initPose(numJoints),currentPose(numJoints);
     std::vector<std::vector<double>> traj;
     this->stat_Left(initPose);
     initPose[2] += KDL::epsilon; // initial pose is hard to attain
     currentPose=initPose;
     double rate=200;
     double diff_x=(targetPosition[0]-currentPose[0])/rate;
     double diff_y=(targetPosition[1]-currentPose[1])/rate;
     double diff_z=(targetPosition[2]-currentPose[2])/rate;
//    yCDebug(WR) <<"diff_x"<<diff_x<<"diff_y"<<diff_y<<"diff_z"<<diff_z;
    for(int i=0;i<rate;i++){
        currentPose[0]+=diff_x;
        currentPose[1]+=diff_y;
        currentPose[2]+=diff_z;
//        usleep(period_ms);
//        this->moviLeftPos(currentPose);
        traj.push_back(currentPose);

    }

    return traj;

}
std::vector<std::vector<double>> walkingRobot::movlRightPos(std::vector<double> &targetPosition, double period_ms){
    std::vector<double> initPose(numJoints),currentPose(numJoints);
    std::vector<std::vector<double>> traj;
    this->stat_Right(initPose);
    initPose[2] += KDL::epsilon; // initial pose is hard to attain
    currentPose=initPose;
    double rate=200;
    double diff_x=(targetPosition[0]-currentPose[0])/rate;
    double diff_y=(targetPosition[1]-currentPose[1])/rate;
    double diff_z=(targetPosition[2]-currentPose[2])/rate;
//    yCDebug(WR) <<"diff_x"<<diff_x<<"diff_y"<<diff_y<<"diff_z"<<diff_z;
   for(int i=0;i<rate;i++){
       currentPose[0]+=diff_x;
       currentPose[1]+=diff_y;
       currentPose[2]+=diff_z;
       usleep(period_ms);
//       this->moviRightPos(currentPose);
        traj.push_back(currentPose);
   }
   return traj;
}
bool walkingRobot::moviLeftPos(std::vector<double> &cartesianPosition)
{
    std::vector<double> JointPose(numJoints);
    if(inv_Left(cartesianPosition,JointPose)){
        posdLeft->setPositions(JointPose.data());
        return true;
    }
    else{
        yCError(WR) <<"Movi Left failure";
        return false;
    }
}

bool walkingRobot::moviRightPos(std::vector<double> &cartesianPosition)
{
    std::vector<double> JointPose(numJoints);
    if(inv_Right(cartesianPosition,JointPose)){
        posdRight->setPositions(JointPose.data());
        return true;
    }
    else{
        yCError(WR) <<"Movi Right failure";
        return false;
    }
}
bool walkingRobot::checkLimitsLeft(std::vector<double> &JointPose){


    for(int i=0;i<JointPose.size();i++){
      double qmin=bMin_left.get(i).asFloat64();
      double qmax=bMax_left.get(i).asFloat64() ;
      if(qmin>JointPose[i] || qmax<JointPose[i]){
          yCError(WR) <<"N de articulacion: "<<i<<", Valor:"<<JointPose[i]<<", qmin="<<qmin<<", qmax="<<qmax;
          return false;
      }
    }
    return true;
}
bool walkingRobot::setLimitsLeft(double l){
    for(int i=0;i<numJoints;i++){
      double qmax=l, qmin=-l;
      yDebug()<<i<<"sdfsdfsdfsdf";
      if( !limits_Left->setLimits(i,qmin,qmax) ){
          yCError(WR) <<"N de articulacion Left: "<<i<<", qmin="<<qmin<<", qmax="<<qmax;
          return false;
      }
    }
}
bool walkingRobot::setLimitsRight(double l){
    for(int i=0;i<(numJoints);i++){
      double qmax=l, qmin=-l;
      if( !limits_Right->setLimits(i,qmin,qmax) ){
          yCError(WR) <<"N de articulacion Right: "<<i<<", qmin="<<qmin<<", qmax="<<qmax;
          return false;
      }
    }
}
bool walkingRobot::moviLeftJoint(std::vector<double> &JointPose)
{
    if(checkLimitsLeft(JointPose)){
        posdLeft->setPositions(JointPose.data());
        return true;
    }
    else{
        yCError(WR) <<"value exceed one or more joint limits";
        return false;
    }
}
bool walkingRobot::moviRightJoint(std::vector<double> &JointPose)
{
    if(checkLimitsRight(JointPose)){
        posdRight->setPositions(JointPose.data());
        return true;
    }
    else{
        yCError(WR) <<"value exceed one or more joint limits";
        return false;
    }
}

bool walkingRobot::inv_Right(std::vector<double> &cartesianPosition, std::vector<double> &JointPostion)
{
    std::vector<double> currentQ(numJoints);
    if (!enc_Right->getEncoders(currentQ.data()))
    {
        yCError(WR) << "getEncoders() failed";
        return false;
    }

    if (!solver_Right->invKin(cartesianPosition, currentQ, JointPostion))
    {
        yCError(WR) << "invKin_Right() failed";
        yCError(WR) << "cartesianPosition" << cartesianPosition;
        yCError(WR) << "currentQ" << currentQ;
        return false;
    }

    return true;
}

bool walkingRobot::inv_Left(std::vector<double> &cartesianPosition, std::vector<double> &JointPostion)
{
    std::vector<double> currentQ(numJoints);
    if (!enc_Left->getEncoders(currentQ.data()))
    {
        yCError(WR) << "getEncoders() failed";
        return false;
    }

    if (!solver_Left->invKin(cartesianPosition, currentQ, JointPostion))
    {
        yCError(WR) << "invKin_Left() failed";
        yCError(WR) << "currentQ" << currentQ;
        return false;
    }
    return true;
}


bool walkingRobot::stat_Right(std::vector<double> &currentCartesianPosition)
{
    std::vector<double> currentQ(numJoints);
    if (!enc_Right->getEncoders(currentQ.data()))
    {
        yCError(WR) << "getEncoders() failed";
        return false;
    }
    if (!solver_Right->fwdKin(currentQ, currentCartesianPosition))
    {
        yCError(WR) << "invKin_Right_stat() failed";
        return false;
    }
    return true;
}



bool walkingRobot::stat_Left(std::vector<double> &currentCartesianPosition)
{
    std::vector<double> currentQ(numJoints);
    if (!enc_Left->getEncoders(currentQ.data()))
    {
        yCError(WR) << "getEncoders() failed";
        return false;
    }
    if (!solver_Left->fwdKin(currentQ, currentCartesianPosition))
    {
        yCError(WR) << "invKin_Left_stat() failed";
        return false;
    }
    return true;
}
bool walkingRobot::close(){
    sensorDevice.close();
    sensorDeviceIMU.close();
    robotDevice_Right.close();
    robotDevice_Left.close();
    solverDevice_Right.close();
    solverDevice_Left.close();
    return true;
}
