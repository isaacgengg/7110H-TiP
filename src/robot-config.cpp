#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FrontRight = motor(PORT14, ratio6_1, false);
motor BackRight = motor(PORT12, ratio6_1, false);
motor MiddleRight = motor(PORT13, ratio6_1, true);
motor FrontLeft = motor(PORT4, ratio6_1, true);
motor BackLeft = motor(PORT2, ratio6_1, true);
motor MiddleLeft = motor(PORT3, ratio6_1, false);
motor FrontLift = motor(PORT15, ratio36_1, false);
motor BackLift = motor(PORT17, ratio36_1, false);
motor GoalLock = motor(PORT20, ratio36_1, true);
inertial Inertial = inertial(PORT11);
rotation BLRotation = rotation(PORT17, false);
motor Conveyor = motor(PORT1, ratio6_1, false);
digital_out Claw = digital_out(Brain.ThreeWirePort.A);
digital_out FrontClamp = digital_out(Brain.ThreeWirePort.G);
digital_out BackTilter = digital_out(Brain.ThreeWirePort.C);

digital_out BackClamp = digital_out(Brain.ThreeWirePort.E);
digital_out GoalCover = digital_out(Brain.ThreeWirePort.F);                                                                                                 
rotation FLRotation = rotation(PORT16, false);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

// /**
//  * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
//  * 
//  * This should be called at the start of your int main function.
//  */
void vexcodeInit( void ) {
  // nothing to initialize
}