using namespace vex;

extern brain Brain;

//To set up a motor called LeftFront here, you'd use
//extern motor LeftFront;

//Add your devices below, and don't forget to do the same in robot-config.cpp:
extern motor intake1;
extern motor intake2;
extern motor intakeL;
extern digital_out Clamp;
extern inertial DrivetrainInertial;
extern controller Controller1;
void  vexcodeInit( void );

void togglePiston(void);