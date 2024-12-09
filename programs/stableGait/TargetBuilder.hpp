// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TARGET_BUILDER_HPP__
#define __TARGET_BUILDER_HPP__

#include <vector>

#include <kdl/trajectory.hpp>

#include <ICartesianControl.h>
#include "walkingRobot.hpp"
namespace rl = roboticslab;

class TargetBuilder
{
public:
    using Targets = std::vector<std::vector<double>>;
    TargetBuilder(walkingRobot *robot);
    TargetBuilder(walkingRobot &robot);
    void configure(KDL::Trajectory * tCom, KDL::Trajectory * tLeft, KDL::Trajectory * tRight);
    TargetBuilder::Targets build(double period, Targets & vLeft, Targets & vRight);
    bool validate(Targets &vLeft, Targets &vRight);

private:

    walkingRobot *robot;
    rl::ICartesianControl * iCartLeft;
    rl::ICartesianControl * iCartRight;
    KDL::Trajectory * tCom;
    KDL::Trajectory * tLeft;
    KDL::Trajectory * tRight;
    double maxTime;
    double periodo;
};

#endif // __TARGET_BUILDER_HPP__
