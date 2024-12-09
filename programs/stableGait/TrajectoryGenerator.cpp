// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
#include "TrajectoryGenerator.hpp"

TrajectoryGenerator::TrajectoryGenerator(walkingRobot *_robot,double &_period):
    robot(_robot),
    period(_period)
{}

TrajectoryGenerator::~TrajectoryGenerator()
{
    delete step_generate;
    delete LIPM_trajectory_generator;
    delete target_builder;
}
void TrajectoryGenerator::configure(double &_distance,GaitSpec _gait_spec,FootSpec _foot_spec){
    distance=_distance;
    gait_spec=_gait_spec;
    foot_spec=_foot_spec;
}

bool TrajectoryGenerator::generate(TargetBuilder::Targets &_pointsLeft, TargetBuilder::Targets &_pointsRight, TargetBuilder::Targets &_COM)
{
    std::vector<double> leftInitial,rightInitial;
    robot->stat_Left(leftInitial);
    robot->stat_Right(rightInitial);
    leftInitial[2] += KDL::epsilon; // initial pose is hard to attain
    rightInitial[2] += KDL::epsilon; // initial pose is hard to attain
    KDL::Frame H_leftInitial = rl::KdlVectorConverter::vectorToFrame(leftInitial);
    KDL::Frame H_rightInitial = rl::KdlVectorConverter::vectorToFrame(rightInitial);
    step_generate=new StepGenerator(foot_spec, H_rightInitial, H_leftInitial);

    LIPM_trajectory_generator=new LIPMTrajectoryGenerator(foot_spec, period);
    target_builder=new TargetBuilder(robot);


    step_generate->configure(gait_spec);
    step_generate->generate(distance,steps,com);
    LIPM_trajectory_generator->configure(steps, com);
step_generate->print();

    try
    {
        LIPM_trajectory_generator->generate2(comTraj, leftTraj, rightTraj);
    }
    catch (const KDL::Error_MotionPlanning & e)
    {
        yWarning() << "Error:" << e.Description()<<"____Tipo: "<<e.GetType();
        return false;
    }

    minDuration = std::min(std::min(comTraj.Duration(), leftTraj.Duration()), rightTraj.Duration());
    maxDuration = std::max(std::min(comTraj.Duration(), leftTraj.Duration()), rightTraj.Duration());
    if (maxDuration - minDuration > 0.01)
    {
        yWarning() << "Duration difference exceeds 0.1 seconds:" << maxDuration - minDuration;
       return false;
    }

    LIPM_trajectory_generator->print();

    target_builder->configure(&comTraj, &leftTraj, &rightTraj);
    COM=target_builder->build(period, pointsLeft, pointsRight);
    if (!target_builder->validate(pointsLeft, pointsRight))
    {
        yWarning() << "IK failed";
        return false;
    }
    else{
        _pointsLeft=pointsLeft;
        _pointsRight=pointsRight;
        _COM=COM;
    }

    return true;

}

double TrajectoryGenerator::duration()
{

    return maxDuration;
}

void TrajectoryGenerator::write(std::string NOMBRE_ARCHIVO, std::string NOMBRE_ARCHIVO2, std::string NOMBRE_ARCHIVO3,std::string NOMBRE_ARCHIVO4,std::string DELIMITADOR){
    std::ofstream data1,data2,dataJoints,dataVel;
        data1.open(NOMBRE_ARCHIVO, std::fstream::out);
        data2.open(NOMBRE_ARCHIVO2, std::fstream::out);
        dataJoints.open(NOMBRE_ARCHIVO3, std::fstream::out);
        dataVel.open(NOMBRE_ARCHIVO4, std::fstream::out);
        data1 << "Time" << DELIMITADOR
             << "Left_Leg_x" << DELIMITADOR << "Left_Leg_y"<< DELIMITADOR <<"Left_Leg_z" << DELIMITADOR
             << "Left_Leg_rotx" << DELIMITADOR << "Left_Leg_roty"<< DELIMITADOR <<"Left_Leg_rotz" << DELIMITADOR
             << "Right_Leg_x" << DELIMITADOR<< "Right_Leg_y" << DELIMITADOR << "Right_Leg_z"<< DELIMITADOR
             << "Right_Leg_rotx" << DELIMITADOR<< "Right_Leg_roty" << DELIMITADOR << "Right_Leg_rotz"<< DELIMITADOR
             << "COM_x"  << DELIMITADOR<< "COM_y"  << DELIMITADOR<< "COM_z"<<DELIMITADOR
             << "COM_rotx"  << DELIMITADOR<< "COM_roty"  << DELIMITADOR<< "COM_rotz" <<"\n";

        dataVel << "Time" << DELIMITADOR
             << "vel_Left_Leg_x" << DELIMITADOR << "vel_Left_Leg_y"<< DELIMITADOR <<"vel_Left_Leg_z" << DELIMITADOR
             << "vel_Left_Leg_rotx" << DELIMITADOR << "vel_Left_Leg_roty"<< DELIMITADOR <<"vel_Left_Leg_rotz" << DELIMITADOR
             << "vel_Right_Leg_x" << DELIMITADOR<< "vel_Right_Leg_y" << DELIMITADOR << "vel_Right_Leg_z"<< DELIMITADOR
             << "vel_Right_Leg_rotx" << DELIMITADOR<< "vel_Right_Leg_roty" << DELIMITADOR << "vel_Right_Leg_rotz"<< DELIMITADOR
             << "vel_COM_x"  << DELIMITADOR<< "vel_COM_y"  << DELIMITADOR<< "vel_COM_z"<<DELIMITADOR
             << "vel_COM_rotx"  << DELIMITADOR<< "vel_COM_roty"  << DELIMITADOR<< "COM_rotz" <<"\n";

        data2 << "Time" << DELIMITADOR
             << "Left_Leg_x" << DELIMITADOR << "Left_Leg_y"<< DELIMITADOR <<"Left_Leg_z" << DELIMITADOR
             << "Right_Leg_x" << DELIMITADOR<< "Right_Leg_y" << DELIMITADOR << "Right_Leg_z"<< DELIMITADOR
             << "COM_x"  << DELIMITADOR<< "COM_y"  << DELIMITADOR<< "COM_z" <<"\n";

        double duration=leftTraj.Duration();
        double time=0.0;
        while(time<duration){
            std::vector<double> t=rl::KdlVectorConverter::frameToVector(leftTraj.Pos(time));
            std::vector<double> t2=rl::KdlVectorConverter::frameToVector(rightTraj.Pos(time));
            std::vector<double> t3=rl::KdlVectorConverter::frameToVector(comTraj.Pos(time));

            data1 <<time<<DELIMITADOR<<t[0]<<DELIMITADOR<<t[1]<< DELIMITADOR<<t[2]<<DELIMITADOR<<t[3]<<DELIMITADOR<<t[4]<< DELIMITADOR<<t[5]<<DELIMITADOR
                  <<t2[0]<<DELIMITADOR<<t2[1]<< DELIMITADOR<<t2[2]<<DELIMITADOR<<t2[3]<<DELIMITADOR<<t2[4]<< DELIMITADOR<<t2[5]<<DELIMITADOR
                  <<t3[0]<<DELIMITADOR<<t3[1]<< DELIMITADOR<<t3[2]<<DELIMITADOR<<t3[3]<<DELIMITADOR<<t3[4]<< DELIMITADOR<<t3[5]<<"\n";
            time+=0.02;
        }

        time=0.0;
        while(time<duration){
            std::vector<double> t=rl::KdlVectorConverter::twistToVector(leftTraj.Vel(time));
            std::vector<double> t2=rl::KdlVectorConverter::twistToVector(rightTraj.Vel(time));
            std::vector<double> t3=rl::KdlVectorConverter::twistToVector(comTraj.Vel(time));

            dataVel <<time<<DELIMITADOR<<t[0]<<DELIMITADOR<<t[1]<< DELIMITADOR<<t[2]<<DELIMITADOR<<t[3]<<DELIMITADOR<<t[4]<< DELIMITADOR<<t[5]<<DELIMITADOR
                  <<t2[0]<<DELIMITADOR<<t2[1]<< DELIMITADOR<<t2[2]<<DELIMITADOR<<t2[3]<<DELIMITADOR<<t2[4]<< DELIMITADOR<<t2[5]<<DELIMITADOR
                  <<t3[0]<<DELIMITADOR<<t3[1]<< DELIMITADOR<<t3[2]<<DELIMITADOR<<t3[3]<<DELIMITADOR<<t3[4]<< DELIMITADOR<<t3[5]<<"\n";
            time+=0.02;
        }



        yInfo() << "CoM:" << comTraj.Duration() << "[s], left:" << leftTraj.Duration() << "[s], right:" << rightTraj.Duration() << "[s]";


        for (int i = 0; i < pointsLeft.size(); i++)
        {
             data2<<period*i<< DELIMITADOR << pointsLeft[i][0]<< DELIMITADOR << pointsLeft[i][1]<<DELIMITADOR<< pointsLeft[i][2]<< DELIMITADOR
                  << pointsLeft[i][3]<< DELIMITADOR << pointsLeft[i][4]<<DELIMITADOR<< pointsLeft[i][5]<< DELIMITADOR
                  << pointsRight[i][0]<<DELIMITADOR << pointsRight[i][1]<<DELIMITADOR << pointsRight[i][2]<<DELIMITADOR
                  << pointsRight[i][3]<< DELIMITADOR << pointsRight[i][4]<<DELIMITADOR<< pointsRight[i][5]<< DELIMITADOR
                  << COM[i][0]<<DELIMITADOR << COM[i][1]<<DELIMITADOR << COM[i][2]<<DELIMITADOR
                  << COM[i][3]<<DELIMITADOR << COM[i][4]<<DELIMITADOR << COM[i][5]
                  << "\n";
        }
        std::vector<double> q,q2;

        for (int i = 0; i < pointsLeft.size(); i++)
        {
            robot->inv_Left(pointsLeft[i], q);
            robot->inv_Right(pointsRight[i], q2);
            dataJoints<<i*period<<" "<<q[0]<<" " << q[1]<<" " << q[2]<<" " << q[3]<<" "<< q[4]<<" "<<q[5]<<" "
                <<q2[0]<<" "<< q2[1]<<" "<< q2[2]<<" "<< q2[3]<<" "<< q2[4]<<" "<<q2[5]<<"\n";
        }



        data1.close();
        data2.close();
        dataJoints.close();
        dataVel.close();
}

