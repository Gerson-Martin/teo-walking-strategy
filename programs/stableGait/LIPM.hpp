#ifndef __LIMP_HPP__
#define __LIMP_HPP__
#include <kdl/trajectory_segment.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <yarp/os/LogStream.h>

#include <kdl/velocityprofile_rect.hpp>
#include <fcontrol.h>
class LIPM
{
    double g;
    double zModel;
    double Ts;
    double tSingleSupport;
    MatrixXd discretizeMatrix(MatrixXd M,double Ts);
    double norm1(MatrixXd W);
    void calculateSystem();
    StateSpace SystemLIPM;
    std::vector<KDL::Frame> trajCOM;
    KDL::RotationalInterpolation_SingleAxis orient;
    KDL::Path_Composite * pathCom;
    std::vector<double>x0;
public:
    LIPM(){}
    LIPM(double _Ts,double _Zmodel,double _g);
    void setParam(double _Ts,double _Zmodel,double _g);
    void setInitialCondition(std::vector<double> x0,double _timeSingleFoot);
    KDL::Trajectory_Segment getTrayectory();
    void getTrayectory(std::vector<KDL::Frame> &,std::vector<std::vector<double> >&,double,double);
    KDL::Path_Composite *getPath();

};
#endif
