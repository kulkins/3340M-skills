#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;

//The motor constructor takes motors as (port, ratio, reversed), so for example
//motor LeftFront = motor(PORT1, ratio6_1, false);

//Add your devices below, and don't forget to do the same in robot-config.h:
inertial DrivetrainInertial = inertial(PORT14);
motor intake1 = motor(PORT10, ratio18_1, false);
motor intake2 = motor(PORT9, ratio18_1, true);
digital_out Clamp = digital_out(Brain.ThreeWirePort.H);
controller Controller1 = controller(primary);
motor intakeL = motor(PORT8, ratio18_1, false);

void vexcodeInit( void ) {
  // nothing to initialize
}

void togglePiston(void){
  if(Clamp.value()){
    Clamp.set(false);
  }
  else{
    Clamp.set(true);
  }
  wait(250, msec);
}
