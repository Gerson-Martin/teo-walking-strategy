#ifndef __TRAJECTORY_GENERATOR_HPP__
#define __TRAJECTORY_GENERATOR_HPP__

#include <vector>

#include "GaitSpecs.hpp"
#include "walkingRobot.hpp"
#include "LIPMTrajectoryGenerator.hpp"
#include "StepGenerator.hpp"
#include "TargetBuilder.hpp"
#include <kdl/utilities/error.h>

namespace rl = roboticslab;
class TrajectoryGenerator
{
public:
    TrajectoryGenerator(walkingRobot *_robot,double &_period);
    ~TrajectoryGenerator();
    void configure(double &_distance, GaitSpec _gait_spect, FootSpec _foot_spec);
    bool generate(TargetBuilder::Targets &,TargetBuilder::Targets &,TargetBuilder::Targets &);
    double duration();
    void write(std::string data1, std::string data2,std::string dataJoints,std::string datavel,std::string Delimeter);
private:
    double distance;
    double vel;
    double acc;
    double Ts;
    double radius;
    double eqradius;
    double y;
    double maxDuration;
    double minDuration;
    std::vector<KDL::Frame> steps;
    std::vector<KDL::Frame> com;
    KDL::RotationalInterpolation_SingleAxis orient;
    KDL::Trajectory * tCom;
    KDL::Trajectory * tLeft;
    KDL::Trajectory * tRight;
    LIPMTrajectoryGenerator *LIPM_trajectory_generator;
    StepGenerator *step_generate;
    TargetBuilder *target_builder;
    GaitSpec gait_spec;
    FootSpec foot_spec;
    walkingRobot *robot;
    TargetBuilder::Targets pointsLeft, pointsRight, COM;
    KDL::Trajectory_Composite comTraj, leftTraj, rightTraj;
    double maxTime;
    double period;

};

#endif // __TRAJECTORY_GENERATOR_HPP__
