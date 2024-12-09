// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-




#include "LIPMTrajectoryGenerator.hpp"
#include <yarp/os/LogStream.h>
#include <kdl/path_circle.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_composite.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/velocityprofile_traphalf.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>

#include "LIPM.hpp"
#include "LIPMTrajectoryGenerator.hpp"
#include <iomanip>



const std::string NOMBRE_ARCHIVO = "/home/gerson/MATLAB-Drive/Caminata/TrajectoriaGeneradanew.txt";
const char DELIMITADOR =' ';


constexpr auto DEFAULT_SQUAT_DURATION = 5.0; // [s]
constexpr auto DEFAULT_RADIUS = 0.01; // [m]
constexpr auto DEFAULT_EQ_RADIUS = 1.0; // [m]
constexpr auto DURATION_ONE_STEP = 3.0; // [s]
constexpr auto DURATION_COM_INITIAL_POSITION = 1.0; // [s]
constexpr auto DEFAULT_ACELERATION_GRAVITY = 9.807; //[m/s^2]
constexpr auto DEFAULT_INITIAL_VELOCITY_LIPM =0.01 ;
constexpr auto DEFAULT_TURN_ANGLE_COM=0.06;
constexpr auto  DURATION_TURN_ANGLE_COM=DURATION_ONE_STEP;

LIPMTrajectoryGenerator::LIPMTrajectoryGenerator(FootSpec _footSpec, double _vel, double _acc, double _Ts)
    : footSpec(_footSpec),
      vel(_vel),
      acc(_acc),
      radius(DEFAULT_RADIUS),
      eqradius(DEFAULT_EQ_RADIUS),
      Ts(_Ts),
      y(0.0)
{}

LIPMTrajectoryGenerator::LIPMTrajectoryGenerator(FootSpec _footSpec, double _Ts)
    : footSpec(_footSpec),
      radius(DEFAULT_RADIUS),
      eqradius(DEFAULT_EQ_RADIUS),
      Ts(_Ts),
      y(0.0)
{
}

void LIPMTrajectoryGenerator::configure(const std::vector<KDL::Frame> & _steps, const std::vector<KDL::Frame> & _com)
{
    steps = _steps;
    com = _com;
}

void LIPMTrajectoryGenerator::generate(KDL::Trajectory_Composite & _comTraj, KDL::Trajectory_Composite & _leftTraj, KDL::Trajectory_Composite & _rightTraj)
{

    data_out.open(NOMBRE_ARCHIVO, std::fstream::out);
    /**Empezamos flexionando la piernas del robot y llevarlo a la posicion inicial ***/
    double nStep=0;
    KDL::Frame rightLeg=steps[nStep];
    KDL::Frame leftLeg=steps[nStep+1];

    KDL::Path_Composite * pathSquatDown = new KDL::Path_Composite();

    KDL::Path_Composite * pathSquatUp = new KDL::Path_Composite();

    pathSquatDown->Add(new KDL::Path_Line(com[0], com[1], orient.Clone(), eqradius));
    pathSquatDown->Add(new KDL::Path_Line(com[1], com[2], orient.Clone(), eqradius));

    pathSquatUp->Add(new KDL::Path_Line(com[com.size() - 3], com[com.size() - 2], orient.Clone(), eqradius));
    pathSquatUp->Add(new KDL::Path_Line(com[com.size() - 2], com[com.size() - 1], orient.Clone(), eqradius));

    // FIXME: hardcoded, check behavior on simulator
//    KDL::VelocityProfile * profSquatDown = new KDL::VelocityProfile_Trap(0.1, 0.1);
    KDL::VelocityProfile_Spline * profSquatDown= new KDL::VelocityProfile_Spline();
    profSquatDown->SetProfileDuration(0,0,pathSquatDown->PathLength(),0,DEFAULT_SQUAT_DURATION);
//    KDL::VelocityProfile * profSquatUp = new KDL::VelocityProfile_Trap(0.1, 0.1);
    KDL::VelocityProfile_Spline * profSquatUp= new KDL::VelocityProfile_Spline();
    profSquatUp->SetProfileDuration(0,0,pathSquatUp->PathLength(),0,DEFAULT_SQUAT_DURATION);



    comTraj.Add(new KDL::Trajectory_Segment(pathSquatDown, profSquatDown));
    rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));


    KDL::Frame PointEnterFoot(com[2]);
//    PointEnterFoot.p[1]-=(footSpec.width/2);

    KDL::Path_Composite* pathFootSingleCOM=getTrayectoryWithCOMangle(PointEnterFoot,0.0, leftLeg,DEFAULT_TURN_ANGLE_COM);
    PointEnterFoot=pathFootSingleCOM->Pos(pathFootSingleCOM->PathLength());
    pathFootSingleCOM->Add(getTrayectoryWithCOMangle(PointEnterFoot,0.0, leftLeg,-DEFAULT_TURN_ANGLE_COM));

//    KDL::VelocityProfile * profFootSingleCOM= new KDL::VelocityProfile_Trap((pathFootSingleCOM->PathLength()/DURATION_TURN_ANGLE_COM)*2,0.2);
    KDL::VelocityProfile_Spline  * profFootSingleCOM= new KDL::VelocityProfile_Spline();
    profFootSingleCOM->SetProfileDuration(0,0,pathFootSingleCOM->PathLength(),0,DURATION_TURN_ANGLE_COM);
//    comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, com[2]));
    comTraj.Add(new KDL::Trajectory_Segment(pathFootSingleCOM,profFootSingleCOM));
    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, leftLeg));
    rightTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, rightLeg));
    _comTraj=comTraj;
    _rightTraj=rightTraj;
    _leftTraj=leftTraj;
     return;


    KDL::Path_Composite* pathOneStep=getPathLegSwing(footSpec.lift,rightLeg,steps[nStep+2],100);
    KDL::VelocityProfile_Spline  *profVelOneStep=  new KDL::VelocityProfile_Spline();//new KDL::VelocityProfile_Trap(pathOneStep->PathLength()*2/DURATION_ONE_STEP,0.1);
    profVelOneStep->SetProfileDuration(0,0,pathOneStep->PathLength(),0,DURATION_ONE_STEP);
    rightTraj.Add(new KDL::Trajectory_Segment(pathOneStep, profVelOneStep));
    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, leftLeg));
    comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, comTraj.Pos(comTraj.Duration())));



    nStep++;
    rightLeg=steps[nStep+1];

    /***Asumimos pasos constantes y rectos modificaremso esto en un futuro**/
    double stepsSep=abs(steps[3].p.y()-steps[2].p.y())/2.0;
    double lenghtStep=abs(steps[3].p.x()-steps[2].p.x());
    double dy=DEFAULT_INITIAL_VELOCITY_LIPM;
    double zModel=com[2].p.z();
    double g=DEFAULT_ACELERATION_GRAVITY;
    LIPM lipm(Ts,zModel,g);
    std::vector<double> x0=findInitialConditions(lenghtStep,dy,stepsSep,zModel,g,Ts);
    double tSingleSupport=x0[x0.size()-1];
    x0.pop_back();
    lipm.setInitialCondition(x0,tSingleSupport);
    std::cout<<"sep:"<<x0[0]<<" longitud: "<<x0[2]<<std::endl;


    //RECUERDA ACTUALIZAR LOS REPOS DE FCONTROL DEL STATESPACE
    /**Una vez tenemos los pasos procederemos a calcular las trayectorias de las piernas y del centro de masas con el LIMP ***/
    //los dos primeros steps son las posiciones iniciales de los pies y los dos ultimos son las posiciones finales por tanto nos iteresan los
    //steps intermedios.

    std::vector<KDL::Frame> stepsLIPM{steps.begin()+2,steps.end()-2};

    std::vector<std::vector<double>>x;
    lipm.getTrayectory(steps,x,footSpec.length,footSpec.width);

    KDL::Vector LIPM_initialPosition(x[0][0],x[0][2],x[0][4]);
    KDL::Frame LIPM_COM_initialPosition(LIPM_initialPosition);
    KDL::Path_Line* COM_initialPosition =new KDL::Path_Line(com[2], LIPM_COM_initialPosition, orient.Clone(), eqradius);

//    KDL::VelocityProfile_Rectangular profVel_COM_initialPosition(COM_initialPosition->PathLength()/DURATION_COM_INITIAL_POSITION);



    KDL::VelocityProfile_Spline  * profVel_COM_initialPosition= new KDL::VelocityProfile_Spline();
    profVel_COM_initialPosition->SetProfileDuration(0,0,COM_initialPosition->PathLength(),0,DURATION_COM_INITIAL_POSITION);

    comTraj.Add(new KDL::Trajectory_Segment(COM_initialPosition,profVel_COM_initialPosition));
    rightTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, leftLeg));
    double before_time=0.0;
    double phaseTime=0.0;
    KDL::Trajectory_Composite * parcialCOMtrayectory=new KDL::Trajectory_Composite();


   KDL::Path_Composite* PathLegSwing;
   for (int i = 0; i<(x.size()-1); i+=1)
   {
      KDL::Vector COM_nextPosition(x[i+1][0],x[i+1][2],x[i+1][4]),COM_currentPosition(x[i][0],x[i][2],x[i][4]);
      KDL::Frame COM_P1(COM_currentPosition),COM_P2(COM_nextPosition);
      KDL::Path_Line * pathCom1 =new KDL::Path_Line(COM_P1,COM_P2, orient.Clone(), eqradius);
//      KDL::VelocityProfile *profRect2=new KDL::VelocityProfile_Rectangular(pathCom1->PathLength()/Ts);
      KDL::VelocityProfile_Spline  * profRect2= new KDL::VelocityProfile_Spline();
      profRect2->SetProfileDuration(0,pathCom1->PathLength(),Ts);
//      profRect2->SetProfileDuration(0,pathCom1->PathLength(),Ts);
//       comTraj.Add(new KDL::Trajectory_Segment(pathCom1, profRect2.Clone(), Ts));
      parcialCOMtrayectory->Add(new KDL::Trajectory_Segment(pathCom1, profRect2));
      //0 double support
      //-1 single support right
      //1 single support left
       if(x[i][x[0].size()-1]!=x[i+1][x[0].size()-1] || i==(x.size()-2)){
           if(i==(x.size()-2))i++;
           phaseTime=i*Ts-before_time;
           before_time=i*Ts;
           if(x[i][x[0].size()-1]==-1){
               nStep++;
               rightTraj.Add(new KDL::Trajectory_Stationary(phaseTime, rightLeg));
               PathLegSwing=getPathLegSwing(footSpec.lift,leftLeg,steps[nStep+1],200);
//                KDL::VelocityProfile_Trap profRect(PathLegSwing->PathLength()*6/phaseTime,0.35);
                KDL::VelocityProfile_Spline  * profRect= new KDL::VelocityProfile_Spline();
                profRect->SetProfileDuration(0,0,PathLegSwing->PathLength(),0,phaseTime);
//               KDL::VelocityProfile_Rectangular profRect(PathLegSwing->PathLength()/phaseTime);
               leftTraj.Add(new KDL::Trajectory_Segment(PathLegSwing, profRect));
               leftLeg=steps[nStep+1];
               comTraj.Add(parcialCOMtrayectory);

           }else if (x[i][x[0].size()-1]==0) {
               rightTraj.Add(new KDL::Trajectory_Stationary(phaseTime, rightLeg));
               leftTraj.Add(new KDL::Trajectory_Stationary(phaseTime, leftLeg));
               comTraj.Add(parcialCOMtrayectory);
           }else{
               nStep++;
               leftTraj.Add(new KDL::Trajectory_Stationary(phaseTime, leftLeg));
               PathLegSwing=getPathLegSwing(footSpec.lift,rightLeg,steps[nStep+1],200);
//               KDL::VelocityProfile_Trap profRect(PathLegSwing->PathLength()*6/phaseTime,0.35);
               KDL::VelocityProfile_Spline  * profRect= new KDL::VelocityProfile_Spline();
               profRect->SetProfileDuration(0,0,PathLegSwing->PathLength(),0,phaseTime);
//               KDL::VelocityProfile_Rectangular profRect(PathLegSwing->PathLength()/phaseTime);
               rightTraj.Add(new KDL::Trajectory_Segment(PathLegSwing, profRect));
               rightLeg=steps[nStep+1];
               comTraj.Add(parcialCOMtrayectory);


           }
        parcialCOMtrayectory=new KDL::Trajectory_Composite();
       }
   }

   //Moviment of COM from LIMP end point  to final support foot
   KDL::Vector LIMP_endPosition(x[x.size()-1][0],x[x.size()-1][2],x[x.size()-1][4]);
   KDL::Frame LIMP_COM_endPosition(LIMP_endPosition);
   KDL::Path_Line* COM_endPosition =new KDL::Path_Line(LIMP_COM_endPosition,com[com.size() - 3], orient.Clone(), eqradius);

//   KDL::VelocityProfile_Rectangular profVel_COM_endPosition(COM_endPosition->PathLength()/DURATION_COM_INITIAL_POSITION);
   KDL::VelocityProfile_Spline  *profVel_COM_endPosition=  new KDL::VelocityProfile_Spline();
   profVel_COM_endPosition->SetProfileDuration(0,0,COM_endPosition->PathLength(),0,DURATION_COM_INITIAL_POSITION);

   comTraj.Add(new KDL::Trajectory_Segment(COM_endPosition,profVel_COM_endPosition));
   rightTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, rightLeg));
   leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, leftLeg));



    //last step in the trayectory
   bool movingLegRight=steps[nStep].p.y()<=0.0;
   if(movingLegRight){
   KDL::Path_Composite* pathEndStep=getPathLegSwing(footSpec.lift,rightLeg,steps[nStep+2],200);
//   KDL::VelocityProfile_Rectangular profVelEndStep(pathEndStep->PathLength()/DURATION_ONE_STEP);
   KDL::VelocityProfile_Spline  *profVelEndStep=  new KDL::VelocityProfile_Spline();
   profVelEndStep->SetProfileDuration(0,0,pathEndStep->PathLength(),0,DURATION_ONE_STEP);


   rightTraj.Add(new KDL::Trajectory_Segment(pathEndStep, profVelEndStep));
   leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, leftLeg));
   comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, com[com.size() - 3]));
   nStep++;
   rightLeg=steps[nStep+1];
    }else{
       KDL::Path_Composite* pathEndStep=getPathLegSwing(footSpec.lift,leftLeg,steps[nStep+2],200);
//       KDL::VelocityProfile_Rectangular profVelEndStep(pathEndStep->PathLength()/DURATION_ONE_STEP);
       KDL::VelocityProfile_Spline  *profVelEndStep=  new KDL::VelocityProfile_Spline();
       profVelEndStep->SetProfileDuration(0,0,pathEndStep->PathLength(),0,DURATION_ONE_STEP);

       leftTraj.Add(new KDL::Trajectory_Segment(pathEndStep, profVelEndStep));
       rightTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, rightLeg));
       comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, com[com.size() - 3]));
       nStep++;
       leftLeg=steps[nStep+1];
   }

    //Go back to final position
   comTraj.Add(new KDL::Trajectory_Segment(pathSquatUp, profSquatUp));
   rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
   leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));

   _comTraj=comTraj;
   _rightTraj=rightTraj;
   _leftTraj=leftTraj;
   return;
}

void LIPMTrajectoryGenerator::generate2(KDL::Trajectory_Composite &comTraj, KDL::Trajectory_Composite &leftTraj, KDL::Trajectory_Composite &rightTraj)
{
    /**Empezamos flexionando la piernas del robot y llevarlo a la posicion inicial ***/

    KDL::Frame rightLeg=steps[nStep];
    //nstep indica el pie que se va a mover
    KDL::Frame leftLeg=steps[nStep+1];


    KDL::Path_Composite * pathSquatDown = new KDL::Path_Composite();
    KDL::Path_Composite * pathSquatUp = new KDL::Path_Composite();

    pathSquatDown->Add(new KDL::Path_Line(com[0], com[1], orient.Clone(), eqradius));
    pathSquatUp->Add(new KDL::Path_Line(com[com.size()-2], com[com.size()-1], orient.Clone(), eqradius));
    // hacemos el squat
    KDL::VelocityProfile_Spline * profSquatDown= new KDL::VelocityProfile_Spline();
    profSquatDown->SetProfileDuration(0,0,pathSquatDown->PathLength(),0,DEFAULT_SQUAT_DURATION);
    KDL::VelocityProfile_Spline * profSquatUp= new KDL::VelocityProfile_Spline();
    profSquatUp->SetProfileDuration(0,0,pathSquatUp->PathLength(),0,DEFAULT_SQUAT_DURATION);

    comTraj.Add(new KDL::Trajectory_Segment(pathSquatDown, profSquatDown));
    rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));

    double hight=com[1].p[2];
    double pointInsideFoot=footSpec.width/2;
    KDL::Rotation M_initial=com[0].M;

    for (double var = 0; var < (steps.size()-2); var++) {
        // el COM esta sobre la planta del pie
        KDL::Frame PointEnterFoot(steps[nStep+1]);
        PointEnterFoot.p[2]=hight;
        PointEnterFoot.M=M_initial;
        if (PointEnterFoot.p[1]<=0){
            PointEnterFoot.p[1]+=pointInsideFoot;
            KDL::Path * pathEnterFoot = new KDL::Path_Line(comTraj.Pos(comTraj.Duration()), PointEnterFoot, orient.Clone(), eqradius);
            KDL::VelocityProfile_Spline * profEnterFoot= new KDL::VelocityProfile_Spline();
            profEnterFoot->SetProfileDuration(0,0,pathEnterFoot->PathLength(),0,DEFAULT_SQUAT_DURATION);

            comTraj.Add(new KDL::Trajectory_Segment(pathEnterFoot, profEnterFoot));
            rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
            leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));
            singleFootTrajectory(comTraj,leftTraj,rightTraj,leftLeg,rightLeg,nStep,0.0);

        }
        else{
            PointEnterFoot.p[1]-=pointInsideFoot;
            KDL::Path * pathEnterFoot = new KDL::Path_Line(comTraj.Pos(comTraj.Duration()), PointEnterFoot, orient.Clone(), eqradius);
            KDL::VelocityProfile_Spline * profEnterFoot= new KDL::VelocityProfile_Spline();
            profEnterFoot->SetProfileDuration(0,0,pathEnterFoot->PathLength(),0,DEFAULT_SQUAT_DURATION);

            comTraj.Add(new KDL::Trajectory_Segment(pathEnterFoot, profEnterFoot));
            rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
            leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));
            singleFootTrajectory(comTraj,rightTraj,leftTraj,rightLeg,leftLeg,nStep,0.0);

        }

    // el COM esta sobre la planta del pie
    KDL::Path * pathEnterFoot3 = new KDL::Path_Line(comTraj.Pos(comTraj.Duration()),com[4], orient.Clone(), eqradius);
    KDL::VelocityProfile_Spline * profEnterFoot3= new KDL::VelocityProfile_Spline();
    profEnterFoot3->SetProfileDuration(0,0,pathEnterFoot3->PathLength(),0,DEFAULT_SQUAT_DURATION);

    comTraj.Add(new KDL::Trajectory_Segment(pathEnterFoot3, profEnterFoot3));
    rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));


    // el COM esta sobre la planta del pie
    KDL::Path * pathEnterFoot4 = new KDL::Path_Line(comTraj.Pos(comTraj.Duration()),com[5], orient.Clone(), eqradius);
    KDL::VelocityProfile_Spline * profEnterFoot4= new KDL::VelocityProfile_Spline();
    profEnterFoot4->SetProfileDuration(0,0,pathEnterFoot4->PathLength(),0,DEFAULT_SQUAT_DURATION);

    comTraj.Add(new KDL::Trajectory_Segment(pathEnterFoot4, profEnterFoot4));
    rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));


    return;




    /*    KDL::Frame LIMP_COM_initialPosition(com[3]);
        KDL::Path_Line* COM_initialPosition =new KDL::Path_Line(comTraj.Pos(comTraj.Duration()), LIMP_COM_initialPosition, orient.Clone(), eqradius);
        KDL::VelocityProfile *profVel_COM_initialPosition=new KDL::VelocityProfile_Trap(COM_initialPosition->PathLength()/DURATION_COM_INITIAL_POSITION*2,0.1);
        comTraj.Add(new KDL::Trajectory_Segment(COM_initialPosition,profVel_COM_initialPosition->Clone(),DURATION_COM_INITIAL_POSITION));
        rightTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, rightLeg));
        leftTraj.Add(new KDL::Trajectory_Stationary(DURATION_COM_INITIAL_POSITION, leftLeg));
        double before_time=0.0;
        double phaseTime=0.0;
        KDL::Trajectory_Composite * parcialCOMtrayectory=new KDL::Trajectory_Composite();
        KDL::Path_Composite * pathCom1 =new KDL::Path_Composite();
       KDL::Path_Composite* PathLegSwing;
       double m=1;
       for (int i = 3; i<com.size()-4; i+=1)
       {
          KDL::Frame COM_P1(com[i]),COM_P2(com[i+1]);

          pathCom1->Add(new KDL::Path_Line(COM_P1,COM_P2, orient.Clone(), eqradius));

             //0 double support
             //-1 single support right
             //1 single support left
           if(phaseWalking[i]!=phaseWalking[i+1] || i==(com.size()-5)){

               if(i==(com.size()-5)|| before_time==0.0)m=0.5,before_time++;
               else m=1;
               if(phaseWalking[i]==-1){

                   double distancia_COM=pathCom1->Pos(pathCom1->PathLength()).p.x()-pathCom1->Pos(0).p.x();
                   singleFootTrajectory(comTraj,leftTraj,rightTraj,leftLeg,rightLeg,nStep,distancia_COM);
                   std::cout<<"----------------------------------------------right"<<comTraj.Duration()<<std::endl;
               }else if (phaseWalking[i]==0) {

                   auto path=new KDL::Path_Line(comTraj.Pos(comTraj.Duration()),pathCom1->Pos(0),orient.Clone(), eqradius);
                   auto path2=pathCom1;
                   pathCom1 =new KDL::Path_Composite();
                   pathCom1->Add(path);
                   pathCom1->Add(path2);
                   KDL::VelocityProfile_Trap profRectDS(pathCom1->PathLength()/DEFAULT_DURATION_DS*2,0.1);
                   KDL::Trajectory_Segment *COM_DOUBLE=new KDL::Trajectory_Segment(pathCom1,profRectDS.Clone(),DEFAULT_DURATION_DS);

                   rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_DURATION_DS, rightLeg));
                   leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_DURATION_DS, leftLeg));
                   comTraj.Add(COM_DOUBLE);
                        std::cout<<"----------------------------------------------COM"<<comTraj.Duration()<<std::endl;
               }else{

                double distancia_COM=pathCom1->Pos(pathCom1->PathLength()).p.x()-pathCom1->Pos(0).p.x();

                singleFootTrajectory(comTraj,rightTraj,leftTraj,rightLeg,leftLeg,nStep,distancia_COM);
std::cout<<"----------------------------------------------left"<<comTraj.Duration()<<std::endl;
               }
               pathCom1 =new KDL::Path_Composite();
           }
}


    /////FASE FINAL DE LA CAMINATA/////////////////

    PointEnterFoot=com[com.size()-3];
//    std::cout<<"COM XXXXXX"<<steps[nStep].p.x()<<std::endl;
    PointEnterFoot.p[1]-=(footSpec.width/2)*copysign(1.0,PointEnterFoot.p.y());
//    std::cout<<"COM ANTES XXXXXX"<<comTraj.Pos(comTraj.Duration()).p.x()<<std::endl;
    KDL::Path* pathEndStep=new KDL::Path_Line(comTraj.Pos(comTraj.Duration()),PointEnterFoot, orient.Clone(), eqradius);
    KDL::VelocityProfile* profEndFoot = new KDL::VelocityProfile_Trap(pathEndStep->PathLength()/(DEFAULT_DURATION_DS)*2,0.1);
    comTraj.Add(new KDL::Trajectory_Segment(pathEndStep, profEndFoot, DEFAULT_DURATION_DS));
//    std::cout<<"rightLeg XXXXXX"<<rightLeg.p.x()<<std::endl;
//    std::cout<<"leftLeg XXXXXX"<<leftLeg.p.x()<<std::endl;
    rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_DURATION_DS, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_DURATION_DS, leftLeg));

    bool movingLegRight=steps[nStep].p.y()<=0.0;
    if(!movingLegRight)
    singleFootTrajectory(comTraj,leftTraj,rightTraj,leftLeg,rightLeg,nStep,0.0);
    else singleFootTrajectory(comTraj,rightTraj,leftTraj,rightLeg,leftLeg,nStep,0.0);

    //regresamos a la posicion inicial
    pathEndStep = new KDL::Path_Line(PointEnterFoot, com[com.size()-2], orient.Clone(), eqradius);
    profEndFoot = new KDL::VelocityProfile_Trap(pathEndStep->PathLength()/(DEFAULT_DURATION_DS)*3,0.15);
    comTraj.Add(new KDL::Trajectory_Segment(pathEndStep, profEndFoot, DEFAULT_DURATION_DS/2));
    rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_DURATION_DS/2, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_DURATION_DS/2, leftLeg));

    //regresamos arriba
    comTraj.Add(new KDL::Trajectory_Segment(pathSquatUp, profSquatUp, DEFAULT_SQUAT_DURATION));
    rightTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, rightLeg));
    leftTraj.Add(new KDL::Trajectory_Stationary(DEFAULT_SQUAT_DURATION, leftLeg));



    KDL::VelocityProfile_Spline a;
    a.SetProfileDuration(0,0.05,pathSquatDown->PathLength(),0.01,10/Ts);
//    a.SetProfileDuration(0.0,pathSquatDown->PathLength(),10.0/Ts);
//    a.SetProfile(0.0,pathSquatDown->PathLength());
    KDL::Trajectory_Segment prueba(pathSquatDown->Clone(),a.Clone());
//    for(double i=0.0;i<=prueba.Duration();i+=Ts){
//    data1 <<prueba.Pos(i).p[2]/*prueba.Vel(i).vel[2]*/ /*a.Vel(i)*/
//      <<DELIMITADOR<<Ts*i<<DELIMITADOR<<prueba.Vel(i).vel[2]<<"\n";
//    }
//    std::cout<<comTraj.Duration()<<std::endl;
//    for(double i=0.0;i<=comTraj.Duration();i+=Ts){
//    double zmp_X=comTraj.Pos(i).p.x()-comTraj.Acc(i).vel[0]*comTraj.Pos(i).p.z()/DEFAULT_ACELERATION_GRAVITY;
//    double zmp_Y=comTraj.Pos(i).p.y()-comTraj.Acc(i).vel[1]*comTraj.Pos(i).p.z()/DEFAULT_ACELERATION_GRAVITY;
//    data1 <<zmp_X<< DELIMI*/TADOR<<zmp_Y<<DELIMITADOR<<i<<DELIMITADOR<<comTraj.Acc(i).vel[0]<<DELIMITADOR<<comTraj.Acc(i).vel[1]<<DELIMITADOR<<comTraj.Acc(i).vel[2]<<"\n";
//    }
//    {
//        auto && log = yInfo();
//        log << steps.size() << "steps ([x, y]):";

//        for (int i = 0; i < steps.size(); i++)
//        {
//            const KDL::Vector & p = steps[i].p;
//            std::ostringstream oss;
//            oss << "[" << p.x() << " " << p.y() << "]";
//            log << oss.str();
//        }
//    }


return;

}





void LIPMTrajectoryGenerator::print()
{
    yDebug()<<"footSpec"<<footSpec.length<<footSpec.lift<<footSpec.margin<<footSpec.width;
//    yDebug()<<"GaitSpec"<<gaitSpec.hop<<GaitSpec.sep<<GaitSpec.step<<GaitSpec.squat;
//    yDebug()<<"distancia"<<distance;
    yDebug()<<"Ts"<<Ts;
    yDebug()<<"----------------Points---------------------";
//    leftTraj.Write(std::cout);
}


std::vector<double> LIPMTrajectoryGenerator::findInitialConditions(double stepLength,double dy_mid,double x0,double zModel,double g, double Ts){
    double Tc = sqrt(zModel/g);
    double y_mid = 0.0;
    double E = -g/(2*zModel)*y_mid*y_mid + 0.5*dy_mid*dy_mid;
    double y0 = -stepLength/2;
    double dy0 = sqrt(2*(E+g/(2*zModel)*y0*y0));
    double tsinglesupport = 2*asinh(stepLength/2/(Tc*dy_mid))*Tc;
    tsinglesupport = floor(tsinglesupport/Ts)*Ts;
    double tf = tsinglesupport/2;
    double dx0 = -x0/Tc * sinh(tf/Tc) / cosh(tf/Tc);
    std::vector<double> InitialCondition{x0,dx0,y0,dy0,tsinglesupport};
    std::cout<<std::setprecision(10)<<" x0:"<<x0<<" y0:"<<y0<<" dx0:"<<dx0<<" dy0:"<<dy0<<std::endl;
    return InitialCondition;
}

KDL::Path_Composite* LIPMTrajectoryGenerator::getPathLegSwing(double h,KDL::Frame &step_start,KDL::Frame &step_end,double nPoints=200){
    KDL::Vector v= step_end.p-step_start.p;
    double L=v.Norm();
    double foothold_x=step_start.p.x();
    double foothold_y=step_start.p.y();
    KDL::Path_Composite * LegSwing = new KDL::Path_Composite();
    double angle=atan2(v.x(),v.y());
    double z=0;
    double dx=L/nPoints;
    double y=0;
    double x=0;
    KDL::Frame before_point=step_start;
//    LegSwing->Add(KDL::Frame(step_start));
    for(double i=0;i<=nPoints;i+=1){
        z=generateCurveLeg(h,true,i,nPoints);
        x=foothold_x+generateCurveLeg(v.x(),false,i,nPoints);//i*dx*sin(angle)+foothold_x;
        y=foothold_y+generateCurveLeg(v.y(),false,i,nPoints);//i*dx*cos(angle)+foothold_y;
//        data_out <<x<< DELIMITADOR<<y<< DELIMITADOR<<z<<DELIMITADOR<<i<<DELIMITADOR<<L<<"\n";
        KDL::Frame current_point(step_start.M,KDL::Vector(x,y,z));
        LegSwing->Add(new KDL::Path_Line(before_point, current_point, orient.Clone(), eqradius));
        before_point=current_point;

    }
    return LegSwing;

}


double LIPMTrajectoryGenerator::generateCurveLeg(double h,bool trap,double x ,double nPoints)
{
//    if(x<=(L/3)){plotVelocity.m
//        y+=(h/(nPoints/3));
//        if(y>=h)y=h;
//    }else if(x>(L/3) && x<(2*L/3)){
//        y=h;
//    }else{
//        y-=(h/(nPoints/3));
//        if(y<=0)y=0;
//    }

//    h=L/2;
//    double theta= acos((L/2-x)/h);
//    double y=h*sin(theta);
//    double y=0;
//    if(x<=L/2){
//       y=sqrt(h*h+(L*L/4))*x;
//    }else{ y=sqrt(h*h+(L*L/4))*L/2-sqrt(h*h+(L*L/4))*(x-L/2);}

//    double a=-4*h/(L*L);
//    double b=0,c=h;
//    double y=a*(x-L/2)*(x-L/2)+b*(x-L/2)+c;
    double y;
    double b;
    double c;
    double size;
    if (trap){
        double porcentage=2;
        size=nPoints/porcentage;
        b=1.5*pow(10,(-(log10(size)-1)));
//        cout<<"x:  "<<x<<"  ;"<<endl;
        if (x<=size){
//        cout<<"subimos"<<endl;
            c=size/2;
            y=h/(1+exp(-b*(x-c)));
        }
        else if (x>size && x<=size*(porcentage-1)) {
//            cout<<"permanecemos"<<endl;

            y=h;
        }else{
//            cout<<"bajamos"<<endl;

            c=size*(porcentage-1)+(size/2);
            y=h-h/(1+exp(-b*(x-c)));
        }
    }else{
        size=nPoints;
        c=size/2;
        b=1.5*pow(10,(-(log10(size)-1)));
        y=h/(1+exp(-b*(x-c)));
    }

    return y;
}





KDL::Trajectory_Composite *LIPMTrajectoryGenerator::getTrayectoryWithCOMangle(KDL::Trajectory_Composite* trajCOM, KDL::Frame Leg, double angle)
{

    KDL::Frame start(trajCOM->Pos(0));
    KDL::Vector diff=(start.p-Leg.p);
    double L=(start.p-Leg.p).Norm();
    double initial_angle=atan2(diff.z(),diff.y());
    if(Leg.p.y()<=0){
        angle=abs(angle);
    }else
    angle=-abs(angle);
    double z=0,y=0,x=0,n=trajCOM->Duration()/Ts,diff_angle=(angle/n)*2;
    KDL::Rotation rot2,rot=start.M;
    KDL::Path_Line* COMpath;
    KDL::Trajectory_Composite * parcialCOMtrayectory=new KDL::Trajectory_Composite();

    for(double i=0;i<(n-1);i+=1){
        if(i<=n/2){
        z=L*sin(initial_angle+diff_angle*i)+Leg.p.z();
        y=L*cos(initial_angle+diff_angle*i)+Leg.p.y();
        x=trajCOM->Pos(i*Ts).p.x();
        rot2=rot.RotX(diff_angle*i);
        }else{
            z=L*sin(initial_angle+angle-diff_angle*(i-n/2))+Leg.p.z();
            y=L*cos(initial_angle+angle-diff_angle*(i-n/2))+Leg.p.y();
            x=trajCOM->Pos(i*Ts).p.x();
            rot2=rot.RotX((angle-diff_angle*(i-n/2)));
        }

        KDL::Frame current_point(rot2,KDL::Vector(x,y,z));
        COMpath =new KDL::Path_Line(start,current_point,orient.Clone(),eqradius);
        KDL::VelocityProfile_Rectangular profRect2(COMpath->PathLength()/Ts);
        parcialCOMtrayectory->Add(new KDL::Trajectory_Segment(COMpath, profRect2.Clone(), Ts));
        start=current_point;
    }
//std::cout<<n<<" ,"<<trajCOM->Duration()<<" ,"<<parcialCOMtrayectory->Duration()<<std::endl;

    return parcialCOMtrayectory;

}

KDL::Path_Composite* LIPMTrajectoryGenerator::getTrayectoryWithCOMangle(KDL::Frame &point,double distance, KDL::Frame Leg, double angle)
{

    KDL::Frame start(point);
    /*std::cout<<point.M.GetRot().x()<<std::endl;*/

    KDL::Vector diff=(start.p-Leg.p);
    double L=(start.p-Leg.p).Norm();
    double initial_angle=atan2(diff.z(),diff.y());
    double dh=angle/5;
    if(Leg.p.y()<=0){
        angle=angle;
    }else
    angle=-angle;
    double z=0,y=0,x=0,x0=start.p.x(),n=100.0,diff_angle=(angle/n),diff_x=distance/n;
    KDL::Rotation rot2,rot=start.M;
    KDL::Path_Line* COMpath;
    KDL::Path_Composite * parcialCOMpath=new KDL::Path_Composite();

    for(double i=0;i<=n;i+=1){
        z=/*start.p.z();//*/L*sin(initial_angle+diff_angle*i)+Leg.p.z()+(dh/n)*i;
        y=L*cos(initial_angle+diff_angle*i)+Leg.p.y();
        x=x0+/*sin(i/n)*distance;//*/diff_x*i;
        rot2=rot.RotX(point.M.GetRot().x()+diff_angle*i*2);
//        data1 <<x<< DELIMITADOR<<y<< DDURATION_ONE_STEPELIMITADOR<<z<<DELIMITADOR<<rot2.GetRot().x()<<"\n";
        KDL::Frame current_point(rot2,KDL::Vector(x,y,z));
        COMpath =new KDL::Path_Line(start,current_point,orient.Clone(),eqradius);
        parcialCOMpath->Add(COMpath);
        start=current_point;
    }
    return parcialCOMpath;

}


void LIPMTrajectoryGenerator::singleFootTrajectory(KDL::Trajectory_Composite &comTraj, KDL::Trajectory_Composite &LegInFlightTraj, KDL::Trajectory_Composite &staticLegTraj, KDL::Frame &LegInFlight, KDL::Frame &staticLeg, double &nStep, double distance)
{
    //Levantamos el pie y giramos el torso a la vez//
    KDL::Frame PointEnterFoot(comTraj.Pos(comTraj.Duration()));
    KDL::Path* pathFootSingleCOM=getTrayectoryWithCOMangle(PointEnterFoot,distance/2.0, staticLeg,DEFAULT_TURN_ANGLE_COM);

    KDL::VelocityProfile_Spline * profFootSingleCOM= new KDL::VelocityProfile_Spline();
    profFootSingleCOM->SetProfileDuration(0,0,pathFootSingleCOM->PathLength(),0,DURATION_TURN_ANGLE_COM);

    KDL::Frame H=pathFootSingleCOM->Pos(pathFootSingleCOM->PathLength());
    yDebug()<<H.M.GetRot().x();
    yDebug()<<H.M.GetRot().x();
    H=H*PointEnterFoot.Inverse();
    KDL::Frame a=LegInFlight;
    a.p[2]+=footSpec.lift;
    KDL::Frame liftLeg=H*a;
    liftLeg.M=LegInFlight.M;


    //    liftLeg.p[0]+=distance;
    KDL::Path* pathFootSingle=new KDL::Path_Line(LegInFlight, liftLeg, orient.Clone(), eqradius);

    KDL::VelocityProfile_Spline * profFootSingle= new KDL::VelocityProfile_Spline();
    profFootSingle->SetProfileDuration(0,0,pathFootSingle->PathLength(),0,DURATION_TURN_ANGLE_COM);

    comTraj.Add(new KDL::Trajectory_Segment(pathFootSingleCOM, profFootSingleCOM));
    LegInFlightTraj.Add(new KDL::Trajectory_Segment(pathFootSingle, profFootSingle));
    staticLegTraj.Add(new KDL::Trajectory_Stationary(DURATION_TURN_ANGLE_COM, staticLeg));


    //Llevamos el pie a la posicion final manteniendo la posicion del torso//
    liftLeg=LegInFlightTraj.Pos(LegInFlightTraj.Duration());
    PointEnterFoot.p[0]+=distance;
    H=pathFootSingleCOM->Pos(pathFootSingleCOM->PathLength())*PointEnterFoot.Inverse();
    KDL::Frame v4=steps[nStep+2];
    v4.p[2]+=footSpec.lift;
    v4=H*v4;
    v4.M=steps[nStep+2].M;
    KDL::Path* pathEndStep=new KDL::Path_Line(liftLeg, v4, orient.Clone(), eqradius);//
//                getPathLegSwing(footSpec.lift,liftLeg,v4,200);

    KDL::VelocityProfile_Spline * profVelEndStep= new KDL::VelocityProfile_Spline();
    profVelEndStep->SetProfileDuration(0,0,pathEndStep->PathLength(),0,DURATION_ONE_STEP);

    KDL::Frame beforeTurnCOM=comTraj.Pos(comTraj.Duration());

    comTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP,beforeTurnCOM));
    staticLegTraj.Add(new KDL::Trajectory_Stationary(DURATION_ONE_STEP, staticLeg));
    LegInFlightTraj.Add(new KDL::Trajectory_Segment(pathEndStep, profVelEndStep));

    //Desgiramos el torso y bajamos una pequeña distancia el pie al suelo
    pathFootSingleCOM=getTrayectoryWithCOMangle(beforeTurnCOM,distance/2.0, staticLeg,-DEFAULT_TURN_ANGLE_COM);
    profFootSingleCOM= new KDL::VelocityProfile_Spline();
    profFootSingleCOM->SetProfileDuration(0,0,pathFootSingleCOM->PathLength(),0,DURATION_TURN_ANGLE_COM);


    pathEndStep=new KDL::Path_Line(v4,steps[nStep+2], orient.Clone(), eqradius);
    profVelEndStep=new KDL::VelocityProfile_Spline();
    profVelEndStep->SetProfileDuration(0,0,pathEndStep->PathLength(),0,DURATION_TURN_ANGLE_COM);



    comTraj.Add(new KDL::Trajectory_Segment(pathFootSingleCOM, profFootSingleCOM));
    LegInFlightTraj.Add(new KDL::Trajectory_Segment(pathEndStep, profVelEndStep));
    staticLegTraj.Add(new KDL::Trajectory_Stationary(DURATION_TURN_ANGLE_COM, staticLeg));


    nStep++;
    LegInFlight=steps[nStep+1];

    return;
}
