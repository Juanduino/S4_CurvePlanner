# S4 CurvePlanner C++ 4th order S-curve motion-planner

In development - not testet at this moment. Code compiles. 

This libery calculates and executes 4th order S-curve motion. 

4th order S-curve is divided into 15 time segments. 
It is only when a constant-jerk time exist, that the algorithm uses all 15 time-segments. When max jerk is not reached, only 7 time-segments is required.

The S-curve is a smoother motion profile compared to the trapez, since acceleration and deceleration are curved.

All the math for the 4th order planner was inspired by:


[Kinematically Constrained Jerk–Continuous S-Curve Trajectory Planning in Joint Space for Industrial Robots](https://www.mdpi.com/2079-9292/12/5/1135)



Furthermore, the goal is to implement the planner with a circular buffer, so that the planner can be fed commands from a serial port or USB connection,
and execute the commands in the order they were received. This will make the S4_CurvePlanner capable of handeling varius G and M commands.

In order to use it with the SimpleFOC library, the planner will need to be created and linked in the main.cpp file, and the runPlannerOnTick() function will need to be called in the main loop.
The commander is obviosly also needed. 

We also have to create a circularBuffer class in the same lib folder as the planner is #include "../CircularBuffer/CircularBuffer.h" 
The CircularBuffer folder is in this repo.

This figure illustrates how the motion-profile is divided into 15 time segments.

![alt text](https://github.com/Juanduino/S4_CurvePlanner/blob/main/Images/Figure%203.png)


Here is a move to 200. Pos_target is calculated using deltaTime, velocity_now, last_velocity, acceleration_now, last_acceleration.


![alt text](https://github.com/Juanduino/S4_CurvePlanner/blob/main/Images/math%20checks%20out.png)


[FOR USE WITH SIMPLEFOC ](https://community.simplefoc.com/)

```cpp
S4_CurvePlanner planner(5);

void doPlanner(char *cmd){
  planner.doGcommandBuffer(cmd);
}

void MPlanner(char *cmd){
  planner.doMCommand(cmd);
}
```


//And in the void setup() 

```cpp
  planner.linkMotor(&motor);
	  
  commander.add('G', doPlanner, "Motion Planner");
  
  commander.add('M', MPlanner, "Motion Planner");
```
  
//Here is a example of a loop that will run the planner on every tick.

```cpp
void loop() {

// main FOC algorithm function
   
  motor.loopFOC();

// Motion control function
  
  motor.move();

  //motor.monitor();
  
  commander.run();
  
  planner.runPlannerOnTick();
  
  }
```


