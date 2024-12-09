// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __ONE_FOOT_STAND_HPP__
#define __ONE_FOOT_STAND_HPP__

#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/RFModule.h>

#include <kdl/frames.hpp>
#include "walkingRobot.hpp"
#include <fcontrol.h>
#include "IPlot.h"
#include <fstream>  // Para ofstream
#include <iostream> // Para cout
#include <numeric>
#include "LIPM2d.h"
#include <unsupported/Eigen/Polynomials>

namespace roboticslab
{
    struct System
    {
        struct turnAnkleTCP
        {
            double x;
            double y;
            double z;
            double gx;
            double gy;
            double gz;
        };
        struct ZMP
        {
            double x;
            double y;
            double z;
        };
        struct Target
        {
            double x;
            double y;
            double z;
        };

        turnAnkleTCP input;
        ZMP output;
        Target target;
    };
/**
 * @ingroup oneFootStand
 *
 * @brief Uses ZMP to maintain stability while standing on one foot.
 */
class OneFootStand : public yarp::os::RFModule,
                     public yarp::os::PeriodicThread
{
public:
    OneFootStand()
        : yarp::os::PeriodicThread(1.0, yarp::os::ShouldUseSystemClock::Yes, yarp::os::PeriodicThreadClock::Absolute)
    {PIDBlock a(1.0/25.0,0.0,0.0,0.02);PIDx=a;}

    ~OneFootStand() override
    { close(); }

    bool configure(yarp::os::ResourceFinder & rf) override;
    bool updateModule() override;
    bool interruptModule() override;
    double getPeriod() override;
    bool close() override;
    void EMA_Exponential_Filter(KDL::Vector &data,double alpha);


    typedef typename std::vector<std::complex<double>> roots;
    KDL::Frame getTrayectoryWithCOMangle(KDL::Frame &point, double diff_x, KDL::Frame &Leg, double diff_angle);
    void printAndSave(double angle, double x, KDL::Vector Pzmp, KDL::Vector target_ZMP);
    void printAndSave(double time, KDL::Vector Pzmp, KDL::Vector target_ZMP);
    bool checkLimits(std::vector<double> &JointPose);
    void saveIdentificationData();
    void controlScheme();
    double zmptoangle(double zmp);
    void generate_input();
    void IMCscheme();
    ///////////////////agragado
    vector<double>trajectoria;
    VectorXd ref;
        ////////////////////////////////
    template<typename T>
    std::vector<T> conv(const std::vector<T> &f, const std::vector<T> &g);
    roots get_nth_root(std::vector<double> &polynomial);
    std::vector<double> calculatePolynomial(roots r);
    SystemBlock calculateIMC_Controller(SystemBlock &sys,double &b);
    void MPCscheme();
protected:
    void run() override;

private:
    void printAndSave(std::vector<double> vd, KDL::Vector Pzmp, KDL::Vector target_ZMP, KDL::Vector controlSignal, std::vector<double> vd2);
    void saveData(System datos);
    bool readSensor(KDL::Wrench & wrench_N) const;
    void publishProjection(const KDL::Vector & p_N_zmp);
    std::vector<System> datos;
    void LIPM2d_system();
    KDL::Rotation R_N_sensor;
    KDL::Rotation R_N_sole;


    PIDBlock PIDx;
    PIDBlock PIDy;
    SystemBlock filtro,filtro2;
    SystemBlock sys,imc;
    MPC mpc;

    IPlot *reference;
    IPlot *pVt;
    IPlot *err;
    IPlot *control;
    IPlot *err_model;
    IPlot *sistema;
    FPDBlock controller;


    bool dryRun;
    std::ofstream data;

    double period;
    double ikStep;
    double maxSpeed;
    double maxAcceleration;
    double index;
    double m=64,L=0.85,g=9.8;
    KDL::Vector datoAnterior;

    double previousStep {0.0};
    std::vector<double>  lefTCP;
    std::vector<double> rightInitial,leftInitial;
    std::vector<double> generatedInput;
    LIPM2d model_Ju;

    walkingRobot robot;
};



} // namespace roboticslab

#endif // __ONE_FOOT_STAND_HPP__




