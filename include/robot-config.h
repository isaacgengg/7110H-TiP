using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor FrontRight;
extern motor BackRight;
extern motor MiddleRight;
extern motor FrontLeft;
extern motor BackLeft;
extern motor MiddleLeft;
extern motor FrontLift;
extern motor BackLift;
extern motor GoalLock;
extern inertial Inertial;
extern rotation BLRotation;
extern motor Conveyor;
extern digital_out Claw;
extern digital_out FrontClamp;
extern digital_out BackClamp;
extern digital_out GoalCover;
extern digital_out BackTilter;
extern rotation FLRotation;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );