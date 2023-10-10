#include <Arduino.h>
#include <S4_CurvePlanner.h>

#define __debug

S4_CurvePlanner::S4_CurvePlanner(int tickPeriod){
    plannerPeriod = tickPeriod;
    isTrajectoryExecuting = false;
    return;
}

void S4_CurvePlanner::linkMotor(StepperMotor *motorReference){
    motor = motorReference;
}

bool S4_CurvePlanner::isPlannerMoving(){
    return isTrajectoryExecuting;
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max){ 
     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void S4_CurvePlanner::doMCommand(char *MCommand){
  #ifdef __debug
        //Serial.print("GGode command: M");
        //Serial.println(MCommand);
    #endif


       
      // Parse this string for vals to int
        String commandSrt = String(MCommand);
        int commandValue_Int = 0;
        commandValue_Int = commandSrt.toInt();

  
        // The controller must be able to report its capabilities, typically with the M115 command.
        if (commandValue_Int == 115){
            // M115
            // Send firmware version and capabilities
            // Note: Not final. Suggested by co-pilot.
            Serial.println("FIRMWARE_NAME:SimpleFOC");
            Serial.println("FIRMWARE_VERSION:1.0.0");
            Serial.println("FIRMWARE_URL:GIT_URL");
            Serial.println("PROTOCOL_VERSION:1.0");
            Serial.println("AVAILABLE_COMMANDS:M,V,A,L");
            Serial.println("CAPABILITY:MOTOR_VOLTAGE,INPUT_VOLTAGE,POWER_SUPPLY,POSITION_CONTROL,VELOCITY_CONTROL,VELOCITY_RAMP,TRAJECTORY_CONTROL");
            Serial.println("POWER_SUPPLY:24V");
            Serial.println("MOTOR_VOLTAGE:24V");
            Serial.println("INPUT_VOLTAGE:24V");
            Serial.println("POSITION_CONTROL:1");
            Serial.println("VELOCITY_CONTROL:1");
            Serial.println("VELOCITY_RAMP:1");
            Serial.println("TRAJECTORY_CONTROL:1");
            Serial.println("POSITION_MIN:-3.14159265359");
            Serial.println("POSITION_MAX:3.14159265359");
            Serial.println("VELOCITY_MIN:-12.5663706144");
            Serial.println("VELOCITY_MAX:12.5663706144");
            Serial.println("ACCELERATION_MIN:-12.5663706144");
            Serial.println("ACCELERATION_MAX:12.5663706144");


        }

        //The controller can report axes positions, including extra axes (A, B, C etc.), typically with the M114 command.

        if (commandValue_Int == 114){
        Serial.println("ok");
        // M114
        // Send current position
        
        //MM per revolution movement 
        //int mm_per_rev = 17;
        
        //convert to mm from 2 x radians per revolution using the mm_per_rev variable as the mm per revolution
        
        float mm = mapfloat(motor->shaft_angle, 0, 2*PI, 0, buffer.mm_per_rev);
        Serial.print("X:");
        Serial.println(mm, 4);
        

       
          } 


         // The controller must be able to wait for motion completion, typically with the M400 command. Any further commands sent after the M400 must be suspended until motion completion. 
         // The controller must only acknowledge the command, when motion is complete i.e. the "ok" (COMMAND_CONFIRM_REGEX) response must be suspended until then, providing blocking 
         // synchronization to OpenPnP.

            if (commandValue_Int == 400){
                // M400
                // Wait for current move to complete
                m400_flag = true;
               // Serial.println("ok");

            }

            //The controller must support dynamic acceleration and/or jerk limits, typically by the M204 command for acceleration or the M201.3 command for jerk.
            float commandValue2;
             if (commandValue_Int == 204){
                // M204
                // Set acceleration
        
        
                // Remove M so can convert to a float
        
        commandValue2 = commandSrt.toFloat();
        Vmax_ = commandValue2;
        // Try calling the planner to use this new velocity value
        // We have to use the current pos, vel, and accel
        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
        //calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
        #ifdef __debug
            Serial.print("User wants velocity change. Vmax_: ");
            Serial.println(Vmax_);
        #endif
      
            }

            if (commandValue_Int == 201.3){
                // M201.3
                // Set jerk

              // Remove M so can convert to a float

        //TODO

        //commandValue2 = commandSrt.toFloat();
        //Amax_ = commandValue2;
        //Dmax_ = Amax_;
        // Try calling the planner to use this new acceleration value
        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
        //calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
        #ifdef __debug
            Serial.print("User wants acceleration change. Amax_: ");
            Serial.println(Amax_);
        #endif 
       
            }

            // The controller must be able to home. The homing procedure must be configurable, typically with the M206 command. The controller must be able to home to a position other than 0.

            if (commandValue_Int == 206){
                // M206
                // Set current position, for homing to 0. 

                commandSrt = commandSrt.substring(4);
                commandValue2 = commandSrt.toFloat();
        
                motor->shaft_angle = commandValue2; 

                // NOTE: Probably need to set position in a better mannor.

                Serial.print("Homing to: ");
                Serial.println(commandValue2);

                Serial.print("Chekking shaft_angle : ");
                Serial.println(motor->shaft_angle);
       
            }
  

            if (commandValue_Int == 203){
                // M203
                // Set maximum feedrate

                //  TODO  TODO TODO TODO
        //  TODO  TODO TODO TODO
      
            }

          

            if (commandValue_Int == 201){
                // M201
                // Set maximum acceleration

                //  TODO  TODO TODO TODO
        //  TODO  TODO TODO TODO
        
            }

            
            if (commandValue_Int == 202){
                // M202
                // Set maximum jerk

                //  TODO  TODO TODO TODO
        //  TODO  TODO TODO TODO
        
            }


            if (commandValue_Int == 205){
                // M205
                // Advanced settings

                //  TODO  TODO TODO TODO
        //  TODO  TODO TODO TODO

            }

            

        // Place different if statements like "fetch current position".
/*
        #ifdef __debug
            Serial.print("M Command :  ");
            Serial.println(commandValue_Int);
        #endif
     
*/
    
}



void S4_CurvePlanner::doGcommandBuffer(char *gCodeCommand){

        buffer.push(gCodeCommand);

}


         

void S4_CurvePlanner::executeCommand(char *gCodeCommand, char *gCodeCommand2){
    #ifdef __debug
        Serial.print("GGode command: G");
        Serial.println(gCodeCommand);
        
    #endif
    //Serial.println("ok");

    
    // Parse this string for vals
    String commandSrt = String(gCodeCommand);
    commandSrt = commandSrt.substring(1);
    float commandValue;
    switch (gCodeCommand[0]){


   

    case 'V':
        // Remove V so can convert to a float
        commandSrt = commandSrt.substring(1);
        commandValue = commandSrt.toFloat();
        Vmax_ = commandValue;
        // Try calling the planner to use this new velocity value
        // We have to use the current pos, vel, and accel
        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
        //calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
        //This new value will be used when the gCommand is executed. 
        #ifdef __debug
            Serial.print("User wants velocity change. Vmax_: ");
            Serial.println(Vmax_);
        #endif
        break;
    case 'A':
        // Remove A so can convert to a float
        commandSrt = commandSrt.substring(1);
        commandValue = commandSrt.toFloat();
        Amax_ = commandValue;
        Dmax_ = Amax_;
        // Try calling the planner to use this new acceleration value
        // calc_plan_trapezoidal_path_at_start(Xf_, Xi_, Vi_, Vmax_, Amax_, Dmax_);
        //calculateTrapezoidalPathParameters(Xf_, motor->shaft_angle, motor->shaft_velocity, Vmax_, Amax_, Dmax_);
        //This new value will be used when the gCommand is executed.
        #ifdef __debug
            Serial.print("User wants acceleration change. Amax_: ");
            Serial.println(Amax_);
        #endif 
        break;
    case 'M':

    commandSrt = commandSrt.substring(1);
    //commandValue = commandSrt.toFloat();
    commandSrt.toCharArray(command_char, 10);
    doMCommand(command_char);

        break;
    default:
        // Remove G so can convert to a float



        commandSrt = commandSrt.substring(1);
        commandValue = commandSrt.toFloat();
        //Serial.print("Float: ");
          //  Serial.println(commandValue, 5);

         //=  map(commandValue, 0, buffer.mm_per_rev, 0, 2*PI);

        float angle_command = mapfloat(commandValue, 0, buffer.mm_per_rev, 0, 2*PI);
        #ifdef __debug
        
            Serial.print("Move to new position (rads): ");
            Serial.println(angle_command, 5);
        #endif 
        // We start moving to the new position
        Initiate_Move(angle_command);
        break;
    }

}




void S4_CurvePlanner::runPlannerOnTick(){
    // This should get entered 100 times each second (100Hz)
    
    if ((unsigned long)(millis() - plannerTimeStap) > plannerPeriod){
        plannerTimeStap = millis();

        
        
        if (!isTrajectoryExecuting){
            // we are not in a move, let's see if we have a new move to start

            
            if (!buffer.isEmpty()){
                // we have a new move to start
                buffer.pop(tailItem, nextItem);

                #ifdef __debug
                Serial.println("tailItem: ");
                Serial.println(tailItem);
                Serial.println("nextItem: ");
                Serial.println(nextItem);
                #endif

                executeCommand(tailItem, nextItem);
            }
        }
             // see if we are in a move or not
            if (isTrajectoryExecuting){
            // we are in a move, let's calc the next position
            float timeSinceStartingTrajectoryInSeconds = (millis() - plannerStartingMovementTimeStamp) / 1000.0f;
            RuntimePlanner(timeSinceStartingTrajectoryInSeconds);
            motor->target = Y_;

            float EventHorizon =    0.1f; // 100ms

             if (!buffer.isEmpty() && !m400_flag && timeSinceStartingTrajectoryInSeconds >= ((t1 + t2) - EventHorizon)){

               
                   
                   buffer.pop(tailItem, nextItem);
                  
                   executeCommand(tailItem, nextItem);
            }



            // see if we are done with our move
            if (timeSinceStartingTrajectoryInSeconds >= Tf){
                // we are done with move
                // motor.monitor_downsample = 0; // disable monitor
                #ifdef __debug
                    Serial.println("Done with move");
                #endif 
                Serial.println("ok");
                float map_pos = mapfloat(motor->shaft_angle, 0, 2*PI, 0, buffer.mm_per_rev);
                Serial.print("X:");
                Serial.println(map_pos, 4);
                Serial.print("Shaft angle:");
                Serial.println(motor->shaft_angle, 4);

               
                isTrajectoryExecuting = false;
                
                 if (m400_flag){
                 Serial.println("ok");
                 m400_flag = false;

                }

            

           
            
             }
         }
     }

}



float S4_CurvePlanner::sign(float val){
    if (val < 0)
        return -1.0f;
    if (val == 0)
        return 0.0f;
    // if val > 0:
    return 1.0f;

}



float S4_CurvePlanner::sign_hard(float val){
    if (val < 0)
        return -1.0f;
    return 1.0f;

}



float S4_CurvePlanner::CalculateTs() {
    // Step 1: Calculate Ts from displacement constraint
    float dmax = std::abs(qs - qe);
    float Ts_d = (4 * dmax) / (8 * Smax_);

    // Step 2: Calculate Ts from velocity constraint
    float vmax = std::abs(2 * (qe - qs) / 3);
    float Ts_v = (3 * vmax) / (2 * Smax_);

    // Step 3: Calculate Ts from acceleration constraint
    float amax = std::abs((qe - qs) / pow(Ts_d, 2));
    float Ts_a = (pow(amax, 2)) / Smax_;

    // Step 4: Calculate Ts from jerk constraint
    float Ts_j = Jmax_ / Smax_;

    // Choose the minimum Ts among the constraints
    return std::min({Ts_d, Ts_v, Ts_a, Ts_j});

}



float S4_CurvePlanner::CalculateTj() {
    // Calculate dmax from the displacement constraint
    float dmax = std::abs(qe - qs);
     Ts = (4 * dmax) / (8 * Smax_);

    // Calculate Tdj based on the displacement constraint (Equation 16)
    float Tdj = (std::abs(qe - qs) / (4 * Jmax_)) + (Ts / (27 * std::pow(Vmax_, 2))) + ((std::abs(qe - qs) * Ts) / (54 * Jmax_))
        + ((std::pow(std::abs(qe - qs), 2)) / (16 * std::pow(Jmax_, 2))) + ((3 * Ts) / (27 * std::pow(Vmax_, 2))) 
        - ((std::abs(qe - qs)) / (4 * Jmax_));

    // Calculate Tvj based on the velocity constraint (Equation 18)
    float Tvj = (std::pow(Ts, 2) / (4 * Smax_)) + (Vmax_ / Jmax_) - ((3 * std::pow(Ts, 2)) / (2));

    // Calculate Taj based on the acceleration constraint (Equation 20)
    float Taj = (Amax_ / Jmax_) - Ts;

    // Choose the minimum Tj among the constraints
    return std::min({Tdj, Tvj, Taj});

    if (Tj == ((std::abs(qe - qs) / (4 * Jmax_)) + (Ts / (27 * std::pow(Vmax_, 2))) + ((std::abs(qe - qs) * Ts) / (54 * Jmax_))
    + ((std::pow(std::abs(qe - qs), 2)) / (16 * std::pow(Jmax_, 2))) + ((3 * Ts) / (27 * std::pow(Vmax_, 2))) 
    - ((std::abs(qe - qs)) / (4 * Jmax_)))) {
        // Case 1: Only varying jerk and constant jerk
        float am = Jmax_ * (Ts + Tj);
        float vm = am * (2 * Ts + Tj);
        SerialUSB.println("Case 1: Varying jerk and constant jerk");
        SerialUSB.print("Acceleration: ");
        SerialUSB.print(am);
        SerialUSB.print("\nVelocity: ");
        SerialUSB.println(vm);

    } else if (Tj == ((std::pow(Ts, 2) / (4 * Smax_)) + (Vmax_ / Jmax_) - ((3 * std::pow(Ts, 2)) / (2)))) {
        // Case 2: Maximum velocity reached without max acceleration
        float am = Jmax_ * (Ts + Tj);
        SerialUSB.println("Case 2: Maximum velocity reached");
        SerialUSB.print("Acceleration: ");
        SerialUSB.println(am);
        // You can calculate and print remaining motion parameters here
    } else {
        // Case 3: Calculate other time intervals or motion parameters
        SerialUSB.println("Case 3: Calculate other time intervals or motion parameters");
        // You can calculate and print remaining motion parameters here
    }

}



float S4_CurvePlanner::CalculateTa() {
    // Calculate dmax from the displacement constraint
    float dmax = std::abs(qe - qs);
    
    // Calculate Tda based on the displacement constraint (Equation 23)
    float Tda = ((3 * std::pow(Tj, 2)) / 2) - (3 * Ts) + ((std::pow(2 * Ts + Tj, 2)) / 4) + (std::abs(qe - qs) / (Amax_));

    // Calculate Tva based on the velocity constraint (Equation 25)
    float Tva = (Vmax_ / Amax_) - Tj - (2 * Ts);

    // Choose the minimum Ta among the constraints
    return std::min(Tda, Tva);

}



float S4_CurvePlanner::CalculateTv() {
    // Calculate dmax from the displacement constraint
    float dmax = std::abs(qe - qs);
    float Ts = (4 * dmax) / (8 * Smax_);
    float Tj = Ts;

    // Calculate Tv based on the velocity constraint (Equation 27)
    float Tv = (std::abs(qe - qs) / Vmax_) - (4 * Ts + 2 * Tj + Ta);

    return Tv;
}


//ChatGPT: "So, the two sets of code do not give the same result, as they use different equations and constraints to calculate the time intervals. 
//The choice of which set of equations to use depends on your specific requirements and constraints for trajectory planning."

bool S4_CurvePlanner::calculateVariables(float Xf, float Xi, float Vi, float Vmax, float Amax, float Dmax, float Jmax, float Smax) {


        // Calculate the time intervals using snap
        //Ts = Jmax_ / Smax_;
        //Tj = Amax_ / Jmax_ - Ts;
        //Ta = Vmax_ / Amax_ - Tj - 2 * Ts;
        //Tv = (qe - qs) / Vmax_ - Ta - 2 * Tj - 4 * Ts;

        // Calculate the time intervals
        Ts = CalculateTs();
        Tj = CalculateTj();
        Ta = CalculateTa();
        Tv = CalculateTv();

        // Calculate the maximum motion parameters
        jmax = Smax_ * Ts;
        amax = Smax_ * Ts * (Ts + Tj);
        vmax = Smax_ * Ts * (Ts + Tj) * (2 * Ts + Tj + Ta);
        dmax = Smax_ * Ts * (Ts + Tj) * (2 * Ts + Tj + Ta) * (4 * Ts + 2 * Tj + Ta + Tv);


        // Calculate transition times for acceleration phase, using Ts (varying jerk), Tj (constant jerk), and Tv(constant velocity - aka cruice)
         t1 = Ts;
         t2 = t1 + Tj;
         t3 = t2 + Ts;
         t4 = t3 + Tj;
         t5 = t4 + Ts;
         t6 = t5 + Tj;
         t7 = t6 + Ts;

        //Constant velocity
         t8 = t7 + Tv;

        // Calculate transition times for deceleration phase (symetrical to acceleration).
         t9 = t8 + Ts;
         t10 = t9 + Tj;
         t11 = t10 + Ts;
         t12 = t11 + Tj;
         t13 = t12 + Ts;
         t14 = t13 + Tj;
         t15 = t14 + Ts;


         /*Remnant*/
         Xi_ = Xi;
         Xf_ = Xf;
         Vi_ = Vi;


    float dX = Xf - Xi;                             // Distance to travel
    float stop_dist = (Vi * Vi) / (2.0f * Dmax);    // Minimum stopping distance
    float dXstop = sign(Vi) * stop_dist;            // Minimum stopping displacement
    float s = sign_hard(dX - dXstop);               // Sign of coast velocity (if any)
    Ar_ = s * Amax;                                 // Maximum Acceleration (signed)
    Dr_ = -s * Dmax;                                // Maximum Deceleration (signed)
    Vr_ = s * Vmax;                                 // Maximum Velocity (signed)


    // If we start with a speed faster than cruising, then we need to decelerate instead of accelerate
    // aka "double deceleration move" in the paper
    if ((s * Vi) > (s * Vr_)){
        Ar_ = -s * Amax;
    }

   
    #ifdef __debug
      
        Serial.println("--------------------------------- ");
        Serial.println(" Calculated S-curve Values:   ");
        Serial.println("     Ts: " + String(Ts));
        Serial.println("     Tj: " + String(Tj));
        Serial.println("     Ta: " + String(Ta));
        Serial.println("     Tv: " + String(Tv));
        Serial.println("     --------------------- ");
        Serial.println("     Ar: " + String(Ar_));
        Serial.println("     Vr: " + String(Vr_));
        Serial.println("     Dr: " + String(Dr_));
        Serial.println("     --------------------- ");
        Serial.println("     Xf: " + String(Xf));
        Serial.println("     Xi: " + String(Xi));
        Serial.println("     Vi: " + String(Vi));
        Serial.println("     --------------------- ");
        Serial.println("     dX: " + String(dX));
        Serial.println("     dXmin: " + String(dXmin));
        Serial.println("     --------------------- ");
 

    #endif

    return true;
}


void S4_CurvePlanner::resetTimeVariables() {
    t1 = t2 = t3 = t4 = t5 = t6 = t7 = t8 = t9 = t10 = t11 = t12 = t13 = t14 = t15 = 0.0f;
}

void S4_CurvePlanner::Initiate_Move(float Pos){
    
    // set our global of the new position
    Xf_ = Pos;

    // At this poin we are atarting to move following the trapezoidal profile
    plannerStartingMovementTimeStamp = millis();

    // take the position from SimpleFOC and set it to our start position
    Xi_ = motor->shaft_angle;

    // take the velocity from SimpleFOC and set it to our start velocity
    Vi_ = motor->shaft_velocity;

    // TODO: If we are being asked to move but are already in motion, we should go with the current velocity, position, and acceleration
    // and keep moving.

    // Now we need to do our calcs before we start moving
    calculateVariables( Xf_,  Xi_,  Vi_,  Vmax_,  Amax_,  Dmax_,  Jmax_,  Smax_);
    motor->target = Y_; // could possibly put this in the method above
    // Tell user how long this move will take
    #ifdef __debug
        Serial.println("Starting to move to a new position");
        Serial.print("Time to complete move (secs):");
        Serial.println(Tf);
        // Velocity and Accel of move
        Serial.print("Velocity for move: ");
        Serial.print(Vmax_);
        Serial.print(", Acceleration: ");
        Serial.println(Amax_);
        Serial.print("Jerk: ");
        Serial.println(Jmax_);

    #endif
    // set our global bool so the tick method knows we have a move in motion
    isTrajectoryExecuting = true;
}






float S4_CurvePlanner::findTf(float Ts, float Tj, float Ta, float Tv, float Td) {
    float Tf = 0.0f;

// Perform the summation using a loop
    for (int i = 1; i <= 15; ++i) {
        Tf += i * Ts + i * Tj + i * Ta + i * Tv;
    }

}




void S4_CurvePlanner::RuntimePlanner(float currentTrajectoryTime) {
    
     //float currentTrajectoryTime = (millis() - plannerStartingMov
  
if (t >= t0 && t < t1) {
    // Code for the [t0, t1] time range
    tau1 = t - t0;

    jerk_now = jmax / T1 * tau1;

    acel_now = (jmax / (2.0f * T1)) * (tau1 * tau1);

    vel_target  = (jmax / (6.0f * T1)) * (tau1 * tau1 * tau1) + vs;

    pos_target = (jmax / (24.0f * T1)) * (tau1 * tau1 * tau1 * tau1) + (vs * tau1) + qs;
}

if (t >= t1 && t < t2) {
    // Code for the [t1, t2] time range
    tau2 = t - t1;

    jerk_now = jmax;

    acel_now = jmax * tau2 + (jmax / 2.0) * T1;

    v1 = v0 + (jmax / 6.0) * (T1 * T1);   

    vel_target  = (jmax / 2.0) * (tau2 * tau2) + (jmax / 2.0) * T1 * tau2 + v1;

    q1 = v0 * T1 + (jmax / 24.0) * (T1 * T1 * T1);

    pos_target = (jmax / 6.0) * (tau2 * tau2 * tau2) + (jmax / 4) * T1 * (tau2 * tau2) + (v1 *tau2) + q1; 
}

if (t >= t2 && t < t3) {
    tau3 = t - t2;
    // Code for the [t2, t3] time range
    jerk_now = jmax - (jmax / T3) * tau3;

    acel_now = (jmax * tau3) - (jmax / (2 * T3)) * (tau3 * tau3) + (jmax / 2) * (T1 + (2 * T2));

    v2 = v1 + (jmax / 2.0) * T2 * (T1 + T2);

    vel_target  = (jmax / 2.0) * (tau3 * tau3) - (jmax / (6.0 * T3)) * (tau3 * tau3 * tau3) + (jmax / 2.0) * (T1 + (2 * T2)) * tau3 + v2;

    q2 = q1 + (jmax / 12.0) * T2 * T2 * (2 * T2 + 3 * T1) + v1 * T2;

    pos_target = (jmax / 6.0) * (tau3 * tau3 * tau3) - (jmax / (6 * T3)) * (tau4 * tau4 * tau4) + (jmax / 4.0) * (T1 + (2 * T2)) * (tau3 * tau3) + (v2 * tau3) + q2; 
}

if (t >= t3 && t < t4) {
    // Code for the [t3, t4] time range
    tau4 = t - t3;

    jerk_now = 0.0;

    acel_now = amax;

    v3 = v2 + (jmax / 6.0) * T3 * ((3 * T1) + (6 * T2) + (2 * T3));

    vel_target  = acel_now * tau4 - v3;

    q3 = q2 + (jmax / 8.0) * (T3 * T3) * ((2 * T1) + (4 * T2) + T3) + (v2 * T3);

    pos_target = (amax / 2.0) * (tau4 * tau4) + (v3 * tau4) + q3;
}

if (t >= t4 && t < t5) {
    // Code for the [t4, t5] time range
    tau5 = t - t4;

    jerk_now = (jmax / T5) * tau5;

    acel_now = - (jmax / (2.0 * T5)) * (tau5 * tau5) + amax;

    v4 = v3 + (amax * T4);

    vel_target  = - (jmax / (6.0 * T5)) * (tau5 * tau5 * tau5) + (amax * tau5) + v4;

    q4 = q3 + (amax / 2.0) * (tau4 * tau4) + (v3 * T4);

    pos_target = - (jmax / (24.0 * T5)) * (tau5 * tau5 * tau5 * tau5) + (amax / 2.0) * (tau5 * tau5) + q4;
}

if (t >= t5 && t < t6) {
    
    // Code for the [t5, t6] time range
    tau6 = t - t5;
    
    jerk_now = -jmax;

    acel_now = -jmax * tau6 + amax - (jmax / (2.0 * T5)) * tau5;

    v5 = v4 - (amax / (6.0 * T5 * T5)) + amax * T5;

    vel_target  = - (jmax / 2.0) * pow(tau6, 2) + (amax - (jmax / 2.0) * tau5) * tau6 + v5;

    q5 = q4 - (jmax / (24.0 * T5 * T5 * T5)) + (amax / (2.0 * T5 * T5)) + v4 * T5;

    pos_target = - (jmax / 6.0) * pow(tau6, 3) + (1.0 / 2.0) * ((amax - (jmax / 2.0) * tau5) * pow(tau6, 2)) + v5 * tau6 + q5;
}

if (t >= t6 && t < t7) {
    // Code for the [t6, t7] time range
    tau7 = t - t6;

    jerk_now = -jmax + (jmax / T7) * tau7;

    acel_now = -jmax * tau7 + (jmax / (2.0 * T7)) * pow(tau7, 2) + amax - (jmax / (2.0 * T5)) - jmax * T6;

    v6 = v5 - (jmax / 2.0) * pow(tau6, 2) + (amax - (jmax / 2.0) * T5) * T6;

    vel_target  = -(jmax / (2.0 * pow(tau7, 2))) + (jmax / (6.0 * T7 * pow(tau7, 3))) + (amax - (jmax / (2.0 * T5)) - jmax * T7) * tau7 + v6;

    q6 = q5 - (jmax / (24.0 * T7 * pow(tau7, 4))) + ((2.0 * amax - jmax * T5 - 2.0 * jmax * T1) / (4.0 * pow(tau7, 2))) + v6 * tau7;

    pos_target = (-jmax / 6.0) * pow(tau7, 3) - (jmax / (24 * T7)) * pow(tau7, 4) + (((2 * amax) - (jmax * T5) - ((2 * jmax) * T1)) / 4) * pow(tau7, 2) + (v6 * tau7) + q6;


}

if (t >= t7 && t < t8) {
    // Code for the [t7, t8] time range
    tau8 = t - t7;

    v7 = v6 - (jmax / (3.0 * T7 * T7)) + (amax - (jmax / (2.0 * T5)) - jmax * T6) * T7;

    q7 = q6 - (jmax / (8.0 * T7 * T7 * T7)) + (2.0 * amax - 2.0 * jmax * T5 - 2.0 * jmax * T6) / (4.0 * T7 * T7);

    // Use v7 and q7 as needed for further calculations
    // j(t) and a(t) are both 0 within this range
    jerk_now = 0.0;

    acel_now = 0.0;

    vel_target = v7;

    pos_target = v7 * tau8 + q7;
}


}



