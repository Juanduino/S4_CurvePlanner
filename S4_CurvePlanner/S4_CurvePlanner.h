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
    void linkMotor(FOCMotor*);
    void runPlannerOnTick();
    bool isPlannerMoving();
    void resetTimeVariables();

    //For testing purposes
    float t0, t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15;
    bool calculateVariables(float Xf, float Xi, float Vi, float Vmax_, float Amax_, float Jmax_, float Smax_);
    void RuntimePlanner(float currentTrajectoryTime);
    bool double_decel_move = false;
    bool Vi_is_positive = false;
    float last_time;

private:
    
    FOCMotor * motor;
    unsigned long plannerTimeStep;
    int plannerPeriod = 0.5; // 1000 / this number = Hz, i.e. 1000 / 100 = 10Hz, 1000 / 10 = 100Hz, 1000 / 5 = 200Hz, 1000 / 1 = 1000hZ
    float _Vmax_ = 30.0f;    // # Velocity max (rads/s)
    float _Amax_ = 20.0f;    // # Acceleration max (rads/s/s)
    float _Jmax_ = 20.0f;    // # Jerk max (rads/s/s/s)
    float _Smax_ = 20.0f;    // # Snap max (rads/s/s/s/s)
    float qs;              // Start position
    float qe;              // Final position
    
    //Variables for calculating pos_target using deltaTime, velocity and acceleration.
    float Last_position;
    float last_velocity;
    float last_acceleratio;                                 
    float now_;
    float deltaTime;

    //Variables related to trajectory/time
    //Tau == DeltaTime
    float tau1, tau2, tau3, tau4, tau5, tau6, tau7, tau8, tau9, tau10, tau11, tau12, tau13, tau14, tau15;
    
    //T(x) == TimeSegment duration
    float T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13, T14, T15;

    //v(x) == Zero crossing time for each time-segment.
    float vs, v0, v1, v2, v3, v4, v5, v6, v7;
    float v1_rd, v2_rd, v3_rd, v4_rd, v5_rd, v6_rd, v7_rd;

    //Functions related to trajectory timing. Here we calculate the actual execution time, and find out what type of move we are in.
    void CalculateTs(float vmax, float amax, float jmax);
    float CalculateTj(float Ts_jerk, float Tj_vmax, float Tj_jmax, float Tj_amax);
    float CalculateTa(float Ts_, float Tj_, float amax_, float vmax_);
    float CalculateTv();
    float CalculateTv_rampToCero();

    float findTf(float Ts, float Tj, float Ta, float Tv, float Td);

    float jmax; // Max jerk
    float amax; // Max acceleration
    float vmax; // Max velocity
    float dmax; // Max displacemnt 
    float smax; // MAX snap
    
    //Variables for cases where ramp down is diferent from ramp up.
    float jmax_rampToCero; // Max jerk
    float amax_rampToCero; // Max acceleration
    float vmax_rampToCero; // Max velocity

    //Trajectory parameters in time Ts (varying jerk ), Tj (constant jerk), Ta (constant acceleration), Tv (constant velocity), , Tf (final).
    float Ts;
    float Tj;
    float Ta;
    float Tv;
    float Tf;

    //Initial Ts calculations made in order to determine the type of move.
    float Ts_d;
    float Ts_v;
    float Ts_a;
    float Ts_j;

    //Jerk calculations
    float Tjv;
    float Tjd;
    float Tja;

    //Acceleraton calculations
    float Tad;
    float Tav;

    //Variables for ramp-down, if ramp-donw is different from ramp-up.
    float Ts_rampToCero;
    float Ts_d_rampToCero;
    float Tv_rampToCero;
    float Ta_rampToCero;
    float Tj_rampToCero;

    //Current planner output @execution time
    float pos_target;
    float jerk_now;
    float acel_now;
    float vel_target;

    //M400 commands tells the planner to wait until all moves are completed before proceeding.
    bool m400_flag = false;

    float Y_; //Velocity Target

    //float Yd_;
    //float Ydd_;
    float Xi_, Xf_, Vi_, Ar_, Dr_, Vr_, Tv_;

    //Are we in a move
    bool isTrajectoryExecuting;
    
    //When this movement began
    unsigned long plannerStartingMovementTimeStamp;

    //Send new position for trajectory calculation
    void Initiate_Move(float Pos);

    //Varius funtions related to trajectory planning.
    void disp_void();
    void disp_void_rampToCero();
    void vel_void();
    void vel_void_rampToCero();
    void acel_void();
    void acel_void_rampToCero();
    void jerk_void();
    void jerk_void_rampToCero();
    void Tjd_void();
    void Tjv_void();
    void Tad_void();
    void Tad_void_rampToCero();
    void Tav_void();

    //Debug output
    void print_debug_variables();

    //Buffer related variables
    char* tailItem;
    char* nextItem;
    char* command_char;

    //CircuarBuffer holder for commands, set desired size.
    CircularBuffer buffer = CircularBuffer(100);

    // Additional setpoints for trajectory
    float dXmin;

};

#endif