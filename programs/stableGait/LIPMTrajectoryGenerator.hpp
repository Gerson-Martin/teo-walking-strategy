// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __LIPMTRAJECTORY_GENERATOR_HPP__
#define __LIPMTRAJECTORY_GENERATOR_HPP__

#include <vector>

#include <kdl/frames.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <yarp/os/LogStream.h>
#include <kdl/path_circle.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/utilities/error.h>
#include "GaitSpecs.hpp"
#include "walkingRobot.hpp"

namespace rl = roboticslab;
class LIPMTrajectoryGenerator
{
public:
    LIPMTrajectoryGenerator(FootSpec footSpec, double vel, double acc, double Ts);
    LIPMTrajectoryGenerator(FootSpec footSpec, double Ts);
    void configure(const std::vector<KDL::Frame> & steps, const std::vector<KDL::Frame> & com);
    void generate(KDL::Trajectory_Composite & comTraj, KDL::Trajectory_Composite & leftTraj, KDL::Trajectory_Composite & rightTraj);
    std::ofstream data_out;
    using Targets = std::vector<std::vector<double>>;
    void print();
    KDL::Trajectory_Composite *getTrayectoryWithCOMangle(KDL::Trajectory_Composite *trajCOM, KDL::Frame Leg, double angle);
    KDL::Path_Composite *getTrayectoryWithCOMangle(KDL::Frame &point, double distance, KDL::Frame Leg, double angle);
    void singleFootTrajectory(KDL::Trajectory_Composite &comTraj, KDL::Trajectory_Composite &LegInFlightTraj, KDL::Trajectory_Composite &staticLegTraj, KDL::Frame &LegInFlight, KDL::Frame &staticLeg, double &nStep, double distance);
    void generate2(KDL::Trajectory_Composite &comTraj, KDL::Trajectory_Composite &leftTraj, KDL::Trajectory_Composite &rightTraj);
private:
    std::vector<double> findInitialConditions(double stepLength,double dy_mid,double x0,double zModel,double g, double Ts);
    double generateCurveLeg(double h, bool trap, double x, double nPoints=200);
    KDL::Path_Composite *getPathLegSwing(double h, KDL::Frame &step_start, KDL::Frame &step_end, double nPoints);
    FootSpec footSpec;
    double distance;
    double vel;
    double acc;
    double Ts;
    double radius;
    double eqradius;
    double y;
    std::vector<KDL::Frame> steps;
    std::vector<KDL::Frame> com;
    KDL::RotationalInterpolation_SingleAxis orient;
    KDL::Trajectory_Composite comTraj;
    KDL::Trajectory_Composite leftTraj;
    KDL::Trajectory_Composite  rightTraj;
    KDL::Trajectory * tCom;
    KDL::Trajectory * tLeft;
    KDL::Trajectory * tRight;
    walkingRobot *robot;
    double maxTime;

};

#endif // __LIPM_TRAJECTORY_GENERATOR_HPP__
