// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __STEP_GENERATOR_HPP__
#define __STEP_GENERATOR_HPP__

#include <vector>

#include <kdl/frames.hpp>

#include "GaitSpecs.hpp"

class StepGenerator
{
public:
    StepGenerator(FootSpec footSpec, const KDL::Frame & initialPoseRight,const KDL::Frame & initialPoseLeft);
    void configure(GaitSpec gaitSpec);
    void generate(double distance, std::vector<KDL::Frame> & _steps, std::vector<KDL::Frame> & _com);
    void print();
private:
    FootSpec footSpec;
    GaitSpec gaitSpec;
    KDL::Frame leftInitialPose;
    KDL::Frame rightInitialPose;
    std::vector<KDL::Frame> steps;
    std::vector<KDL::Frame> com;
};

#endif // __STEP_GENERATOR_HPP__
