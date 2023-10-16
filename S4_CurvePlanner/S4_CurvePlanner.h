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

private:
    
    FOCMotor * motor;
    unsigned long plannerTimeStap;
    int plannerPeriod = 0.5; // 1000 / this number = Hz, i.e. 1000 / 100 = 10Hz, 1000 / 10 = 100Hz, 1000 / 5 = 200Hz, 1000 / 1 = 1000hZ
    float _Vmax_ = 30.0f;    // # Velocity max (rads/s)
    float _Amax_ = 20.0f;    // # Acceleration max (rads/s/s)
    float _Jmax_ = 20.0f;    // # Jerk max (rads/s/s/s)
    float _Smax_ = 20.0f;    // # Snap max (rads/s/s/s/s)
    float qs;              // Start position
    float qe;              // Final position

    //Trajectory parameters in time Ts (varying jerk ), Tj (constant jerk), Ta (constant acceleration), Tv (constant velocity), , Tf (final)
    float Ts;
    float Tj;
    float Ta;
    float Tv;
    float Tf;

    float tau5_last;
    float accel_last;
    float vel_velocity;
    float tau_last = 0.0f;
    float time_step = 0.0f;
    float position_map;
    float time_step_seconds;
    float last_time;

  

    float a0_, a1_, a2_, a3_, a4_, a5_, a6_, a7_, a8_, a9_, a10_;
    float tau1, tau2, tau3, tau4, tau5, tau6, tau7, tau8, tau9, tau10, tau11, tau12, tau13, tau14, tau15;

    float tou1, tou2, tou3, tou4, tou5, tou6, tou7, tou8, tou9, tou10, tou11, tou12, tou13, tou14, tou15;
    float T1, T2, T3, T4, T5, T6, T7, T8, T9, T10, T11, T12, T13, T14, T15;
    float q0, q1, q2, q3, q4, q5, q6, q7, q8, q9, q10, q11, q12, q13, q14;
    float vs, v0, v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14;

    float average_acceleration;
    float previous_acceleration;
    

    float CalculateTs(float vmax_);
    float CalculateTj();
    float CalculateTa(float Ts_, float Tj_);
    float CalculateTv();
    float findTf(float Ts, float Tj, float Ta, float Tv, float Td);

    float jmax; // Max jerk
    float amax; // Max acceleration
    float vmax; // Max velocity
    float dmax; // Max displacemnt 
    float smax; // MAX snap

    float v1_rampToCero, v2_rampToCero, v3_rampToCero, v6_rampToCero, v7_rampToCero;

    //Variables for cases where ramp down is diferent from ramp up.
    float jmax_rampToCero; // Max jerk
    float amax_rampToCero; // Max acceleration
    float vmax_rampToCero; // Max velocity

    float ds_max;
    float Ts_d;
    float Ts_v;
    float Ts_a;
    float Ts_j;

    float Tjv;
    float Tjd;
    float Tja;

    float Tad;
    float Tav;

    float Ts_rampToCero;
    float Ts_d_rampToCero;
    float Tv_rampToCero;
    float Ta_rampToCero;
    float Tj_rampToCero;

    float CalculateTv_rampToCero();

    float am, vm;

    float factor;
    float pos_target;
    float jerk_now;
    float acel_now;
    float vel_target;

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

    

    void Initiate_Move(float Pos);
    void disp_void();
    void disp_void_rampToCero();
    void vel_void();
    void vel_void_rampToCero();
    void acel_void();
    void jerk_void();
    void Tjd_void();
    void Tjv_void();
    void Tad_void();
    void Tav_void();

    char* tailItem;
    char* nextItem;
    char* command_char;
    //CircuarBuffer holder for commands, set desired size.
    CircularBuffer buffer = CircularBuffer(100);

     // Additional setpoints for trajectory
    float dXmin;

    // COSINE PLANNER VARIABLES
    //float t;
    float desiredAmplitude;
};

#endif