# S4 CurvePlanner C++ 4th order S-curve motion-planner

In development - not testet at this moment. Code compiles. 

This is a remake of the TrapezoidalPlanner class for the SimpleFOC library. This iteration calculates 4th order s-curve. 

4th order s-curve is divided into 15 time segments. 

The S-curve is a smoother motion profile than the trapez, since the acceleration and deceleration are curved.

All the math for the 4th order planner was taken from this ressource, with the help of ChatGPT.


[Kinematically Constrained Jerk–Continuous S-Curve Trajectory Planning in Joint Space for Industrial Robots](https://www.mdpi.com/2079-9292/12/5/1135)



Furthermore, the goal is to implement the planner with a circular buffer, so that the planner can be fed commands from a serial port,
and the planner will execute the commands in the order they were received. Therefore the planner will be able to execute varius G and M commands.
In order to use it with the SimpleFOC library, the planner will need to be created and linked in the main.cpp file, and the runPlannerOnTick() function will need to be called in the main loop.
The commander is obviosly also needed. 

We need to create a circularBuffer class in the same lib folder as the planner is #include "../CircularBuffer/CircularBuffer.h" 
NOTE: The circular buffer class was a colaboration with GPTchat.

This figure illustrates how the motion-profile is divided into 15 time segments.

![alt text](https://github.com/Juanduino/S4_CurvePlanner/blob/main/Images/Figure%203.png)





[FOR USE WITH SIMPLEFOC ](https://community.simplefoc.com/)

S4_CurvePlanner planner(5);
// Further more we need to add the following to the main.cpp.


void doPlanner(char *cmd){
  planner.doGcommandBuffer(cmd);
}

void MPlanner(char *cmd){
  planner.doMCommand(cmd);
}



And in the void setup() 
 
 ....

  planner.linkMotor(&motor);
	  
  commander.add('G', doPlanner, "Motion Planner");
  
  commander.add('M', MPlanner, "Motion Planner");

 ....
  
  
  
  

  Here is a example of a loop that will run the planner on every tick.

   // main FOC algorithm function
   
  motor.loopFOC();

  // Motion control function
  
  motor.move();

  //motor.monitor();
  
  commander.run();
  
  planner.runPlannerOnTick();





