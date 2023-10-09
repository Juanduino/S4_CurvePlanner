

//In development - not testet at this moment. Code compiles. 
/**************************************************************************************************************************/

// This is a remake of the TrapezoidalPlanner class for the SimpleFOC library. This iteration calculates 4th order s-curve. 
// 4th order s-curve is divided into 15 time segments. 
// The S-curve is a smoother motion profile than the trapez, since the acceleration and deceleration are not linear, but curved.

/**************************************************************************************************************************/

// Furthermore, the goal is to implement the planner with a circular buffer, so that the planner can be fed commands from a serial port,
// and the planner will execute the commands in the order they were received. Therefore the planner will be able to execute varius G and M commands.
// In order to use it with the SimpleFOC library, the planner will need to be created and linked in the main.cpp file, and the runPlannerOnTick() function will need to be called in the main loop.
// The commander is obviosly also needed. 
//
// We need to create a circularBuffer class in the same lib folder as the planner is #include "../CircularBuffer/CircularBuffer.h" 
// NOTE: The circular buffer class was a colaboration with GPTchat.
//

// TrapezoidalPlanner planner(5);
// 
//
// Further more we need to add the following to the main.cpp.
/*

void doPlanner(char *cmd){
  planner.doGcommandBuffer(cmd);
}

void MPlanner(char *cmd){
  planner.doMCommand(cmd);
}

//And in the void setup() {
 
 ....

      planner.linkMotor(&motor);
  commander.add('G', doPlanner, "Motion Planner");
  commander.add('M', MPlanner, "Motion Planner");

 ....
  
  }

  Here is a example of a loop that will run the planner on every tick.

    // main FOC algorithm function
  motor.loopFOC();

  // Motion control function
  motor.move();

  //motor.monitor();
  
  commander.run();
  planner.runPlannerOnTick();


*/

#ifndef __S4_CurvePlanner__H
#define __S4_CurvePlanner__H
#include <SimpleFOC.h>
#include "../CircularBuffer/CircularBuffer.h"

class S4_CurvePlanner
{
public:

    S4_CurvePlanner(int);
    void executeCommand(char *command, char *command2);
    void doGcommandBuffer(char *command);
    void doMCommand(char *command);
    void linkMotor(StepperMotor*);
    void runPlannerOnTick();
    bool isPlannerMoving();
    void resetTimeVariables();

private:
    
    StepperMotor* motor;
    unsigned long plannerTimeStap;
    int plannerPeriod = 0.5; // 1000 / this number = Hz, i.e. 1000 / 100 = 10Hz, 1000 / 10 = 100Hz, 1000 / 5 = 200Hz, 1000 / 1 = 1000hZ
    float Vmax_ = 30.0f;    // # Velocity max (rads/s)
    float Amax_ = 20.0f;    // # Acceleration max (rads/s/s)
    float Dmax_ = 20.0f;    // # Decelerations max (rads/s/s)
    float Jmax_ = 20.0f;    // # Jerk max (rads/s/s/s)
    float Smax_ = 20.0f;    // # Snap max (rads/s/s/s/s)
    float qs;              // Start position
    float qe;              // Final position

    //Trajectory parameters in time Ts (varying jerk ), Tj (constant jerk), Ta (constant acceleration), Tv (constant velocity), , Tf (final)
    float Ts;
    float Tj;
    float Ta;
    float Tv;
    float Tf;

    float t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15;
    float tau1, tau2, tau3, tau4, tau5, tau6, tau7, tau8, tau9, tau10, tau11, tau12, tau13, tau14, tau15;
    float T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13, T14, T15;
    float q0, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, q13;
    float vs, v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13;
    

    float CalculateTs();
    float CalculateTj(float Ts);
    float CalculateTa();
    float CalculateTv(float Ta);
    float findTf(float Ts, float Tj, float Ta, float Tv, float Td);

    float jmax; // Max jerk
    float amax; // Max acceleration
    float vmax; // Max velocity
    float dmax; // Max displacemnt 


    //M400 commands tells the planner to wait until all moves are completed before proceeding.
    bool m400_flag = false;

    float Y_;
    float Yd_;
    float Ydd_;
    float Xi_, Xf_, Vi_, Ar_, Dr_, Vr_, Tv_;
    unsigned long plannerStartingMovementTimeStamp;
    bool isTrajectoryExecuting;
    float sign(float val);
    float sign_hard(float val);

    bool calculateVariables(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax, float Jmax, float Smax);

    void Initiate_Move(float Pos);
    void RuntimePlanner(float currentTrajectoryTime);

    char* tailItem;
    char* nextItem;
    char* command_char;
    //CircuarBuffer holder for commands, set desired size.
    CircularBuffer buffer = CircularBuffer(100);

     // Additional setpoints for trajectory
    float dXmin;

    // COSINE PLANNER VARIABLES
    float t;
    float desiredAmplitude;
};

#endif