/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FrontRight           motor         1               
// BackRight            motor         19              
// FrontLeft            motor         2               
// BackLeft             motor         18              
// FrontLift            motor         11              
// BackLift             motor         10              
// GoalLock             motor         12              
// Inertial             inertial      20              
// BLRotation           rotation      8               
// Conveyor             motor         4               
// Claw                 digital_out   A               
// FLRotation           rotation      17              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include "pid.h"
#include "AutoSelector.h"
#include "motion-profile.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/////////////////////// Drivetrain Config ///////////////////////
motor_group LeftDrive(BackLeft, FrontLeft, MiddleLeft);
motor_group RightDrive(BackRight, FrontRight, MiddleRight);
drivetrain RobotDrive(LeftDrive, RightDrive, 12.56, 16, 16, inches);

////////////////////////////////// PID //////////////////////////////////
//// Drive and Turn PID Constants ////
PID drivePID(0.185, 0.005); //0.07, 0.2 //0.185, 0.2
PID turnPID(0.185,0.2); //0.185, 0.2

//// Ideal parameters ////
// Turn: 10    With Goal: 7
// Move: 10    Prog Skills: 6;
void Move(double target, double maxPower, double lineAssist, 
double maxlA = 12, double maxTime = 3000) {
  LeftDrive.resetPosition();
  RightDrive.resetPosition();
  timer DriveTimer;
  DriveTimer.reset();
  DriveTimer.time(msec);
  while (true){
    double DrivePosition = (LeftDrive.position(degrees) + RightDrive.position(degrees)) / 2;
    double DriveHeading = Inertial.rotation();

    double DriveError = target - DrivePosition;
    double TurnError = lineAssist - DriveHeading;
    
    if (TurnError > 180) {
      TurnError -=360;
    }

    if (TurnError < -180) {
      TurnError += 360;
    }

    double DrivePower = drivePID.calculateErr(DriveError);
    double TurnPower = turnPID.calculateErr(TurnError);

    if (TurnPower > maxlA) {
      TurnPower = maxlA;
    }
    if (TurnPower < -maxlA) {
      TurnPower = -maxlA;
    }

    if (DrivePower > maxPower) {
      DrivePower = maxPower;
    }
    if (DrivePower < -maxPower) {
      DrivePower = -maxPower;
    }

    // double combinedErr = std::abs(err) + std::abs(terr);
    if ( (std::abs(DriveError) < 5 && std::abs(TurnError) < 0.25) || DriveTimer >= maxTime )  {
      LeftDrive.stop(coast);
      RightDrive.stop(coast);
      DrivePower = 0;
      TurnPower = 0;
      break;
    }

    Brain.Screen.clearScreen(black);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print( "Drive Error: %4.0f  Drive Power: %4.0f", 
    DriveError, DrivePower );
    Brain.Screen.newLine();
    Brain.Screen.print( "Turn Error: %4.0f  Turn Power: %4.0f", 
    TurnError, TurnPower );

    // FrontLeft.spin(forward, DrivePower + TurnPower, voltageUnits :: volt);
    // MiddleLeft.spin(forward, DrivePower + TurnPower, voltageUnits :: volt);
    // BackLeft.spin(forward, DrivePower + TurnPower, voltageUnits :: volt);
    // FrontLeft.spin(forward, DrivePower - TurnPower, voltageUnits :: volt);
    // MiddleLeft.spin(forward, DrivePower - TurnPower, voltageUnits :: volt);
    // BackLeft.spin(forward, DrivePower - TurnPower, voltageUnits :: volt);

    LeftDrive.spin(forward, DrivePower + TurnPower, voltageUnits::volt);
    RightDrive.spin(forward, DrivePower - TurnPower, voltageUnits::volt);
    wait(20, msec);
  }
  LeftDrive.resetPosition();
  RightDrive.resetPosition();
  drivePID.reset();
  
}

void BRAKE()
{
  FrontLeft.stop(hold);
  FrontRight.stop(hold);
  MiddleLeft.stop(hold);
  MiddleRight.stop(hold);
  BackRight.stop(hold);
  BackLeft.stop(hold);
}

void TurnonPID(double gyroRequestedValue, double MaxspeedinRPM, double timeout = 2000)
{
  float gyroSensorCurrentValue;
  float gyroError;
  float gyroDrive;
  float lastgyroError;
  float gyroP;
  float gyroD;


  const float gyro_Kp = 3.7;
  const float gyro_Ki = 0.0;
  const float gyro_Kd = 0.0;

  // const float gyro_Kp = 3.7;
  // const float gyro_Ki = 0.0;
  // const float gyro_Kd = 0.0001;

  timer::system();
  int timeStart =   timer::system();
  int TimeExit = 2;
  double Threshold = 0.25;
  while(1){
    //Reads the sensor value and scale
    gyroSensorCurrentValue = Inertial.rotation();
    Brain.Screen.setCursor(3, 1);

    //calculates error
    gyroError = gyroRequestedValue - gyroSensorCurrentValue;

    //Exit loop
    if(gyroError < Threshold and gyroError > -Threshold){
      break;
    }
    else if( (timer::system() - timeStart ) == timeout){
      Brain.Screen.clearScreen();
      BRAKE();
      break;
    }
    else{
      TimeExit = 0;
    }

    //calculate drive PID
    gyroP = (gyro_Kp * gyroError);
    static float gyroI = 0;
    gyroI += gyroError * gyro_Ki;
    if(gyroI > 1){
      gyroI = 1;
    }
    if(gyroI < -1){
      gyroI = -1;
    }
    gyroD = (gyroError - lastgyroError) * gyro_Kd;
    gyroDrive = gyroP + gyroI + gyroD;

    if(gyroDrive > MaxspeedinRPM){
      gyroDrive = MaxspeedinRPM;
    }
    if(gyroDrive < -MaxspeedinRPM){
      gyroDrive = -MaxspeedinRPM;
    }

    //Move Motors with PID
    int powerValue = gyroDrive;
    FrontRight.spin(vex::directionType::rev,(powerValue), vex::velocityUnits::rpm);
    MiddleRight.spin(vex::directionType::rev,(powerValue), vex::velocityUnits::rpm);
    BackRight.spin(vex::directionType::rev, (powerValue), vex::velocityUnits::rpm);

    MiddleLeft.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);
    FrontLeft.spin(vex::directionType::fwd, (powerValue), vex::velocityUnits::rpm);
    BackLeft.spin(vex::directionType::fwd,  (powerValue), vex::velocityUnits::rpm);

    lastgyroError = gyroError;
    // this_thread::sleep_for(50);
    wait(50, vex::timeUnits::msec);
  }
  BRAKE();
}


double parkP = .5;
void AutoPark(double maxPark, double startA) {
  timer ParkTimer;
  ParkTimer.reset();
  ParkTimer.time(msec);
  while(true) {
    double parkPosition = Inertial.roll(degrees);
    // double parkError =  parkPosition;
    double parkPower = parkPosition * parkP;

    if (parkPower > maxPark) {
      parkPower = maxPark;
    }
    if (parkPower < -maxPark) {
      parkPower = -maxPark;
    }

    Brain.Screen.clearScreen(black);
    Brain.Screen.clearLine(1);
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print( "Park Error: %4.0f  Park Power: %4.0f", 
    parkPosition, parkPower );

    if (std::abs(parkPosition) < 17 ) {
      LeftDrive.stop(brake);
      RightDrive.stop(brake);
    }

    LeftDrive.spin(forward, parkPower, volt);
    RightDrive.spin(forward, parkPower, volt);
    wait(5, msec);
  }
}

// void Park(double ptarget, double pMaxPower, double plineAssist, 
// double pmaxlA = 12, double pmaxTime = 3000) {
//   LeftDrive.resetPosition();
//   RightDrive.resetPosition();
//   while (true){
//     double DrivePosition = (LeftDrive.position(degrees) + RightDrive.position(degrees)) / 2;
//     double DriveHeading = Inertial.rotation();

//     double DriveError = ptarget - DrivePosition;
//     double TurnError = plineAssist - DriveHeading;
    
//     if (TurnError > 180) {
//       TurnError -=360;
//     }

//     if (TurnError < -180) {
//       TurnError += 360;
//     }

//     double DrivePower = drivePID.calculateErr(DriveError);
//     double TurnPower = turnPID.calculateErr(TurnError);

//     if (TurnPower > pmaxlA) {
//       TurnPower = pmaxlA;
//     }
//     if (TurnPower < -pmaxlA) {
//       TurnPower = -pmaxlA;
//     }

//     if (DrivePower > pMaxPower) {
//       DrivePower = pMaxPower;
//     }
//     if (DrivePower < -pMaxPower) {
//       DrivePower = -pMaxPower;
//     }

//     Brain.Screen.clearScreen(black);
//     Brain.Screen.setCursor(1, 1);
//     Brain.Screen.print( "Drive Error: %4.0f  Drive Power: %4.0f", 
//     DriveError, DrivePower );
//     Brain.Screen.newLine();
//     Brain.Screen.print( "Turn Error: %4.0f  Turn Power: %4.0f", 
//     TurnError, TurnPower );


//     LeftDrive.spin(forward, DrivePower + TurnPower, voltageUnits::volt);
//     RightDrive.spin(forward, DrivePower - TurnPower, voltageUnits::volt);
//     wait(20, msec);
//   }

//   LeftDrive.resetPosition();
//   RightDrive.resetPosition();
//   drivePID.reset();
// }

//// Back Lift PID ////
void LiftPID(double targetHeight, double MaxspeedinRPM = 100.0, double timeout = 3000)
{
  float liftEncoderCurrentValue;
  float liftError;
  float liftPower;
  float lastLiftError;

  float liftP;
  float liftD;

  const float lift_kP = 1.5;
  const float lift_kI = 0.0;
  const float lift_kD = 0.7;

  timer::system();
  int timeStart =   timer::system();
  int TimeExit = 2;
  double Threshold = 0.25;
  while(1){
    //Reads the sensor value and scale
    liftEncoderCurrentValue = FrontLift.rotation(degrees);
    Brain.Screen.setCursor(3, 1);

    //calculates error
    liftError = targetHeight - liftEncoderCurrentValue;

    //Exit loop
    if(liftError < Threshold and liftError > -Threshold){
      break;
    }
    else if( (timer::system() - timeStart ) == timeout){
      Brain.Screen.clearScreen();
      //BRAKE();
      break;
    }
    else{
      TimeExit = 0;
    }

    //calculate drive PID
    liftP = (lift_kP * liftError);
    static float liftI = 0;
    liftI += liftError * lift_kI;
    if(liftI > 1){
      liftI = 1;
    }
    if(liftI < -1){
      liftI = -1;
    }
    liftD = (liftError - lastLiftError) * lift_kD;
    liftPower = liftP + liftI + liftD;

    if(liftPower > MaxspeedinRPM){
      liftPower = MaxspeedinRPM;
    }
    if(liftPower < -MaxspeedinRPM){
      liftPower = -MaxspeedinRPM;
    }

    //Move Motors with PID
    int powerValue = liftPower;
    //Arm.spin(directionType :: fwd, powerValue, voltageUnits :: volt);
    FrontLift.spin(directionType :: fwd, powerValue, velocityUnits :: rpm);

    lastLiftError = liftError;
    wait(50, vex::timeUnits::msec);
  }
  FrontLift.stop(brakeType :: hold);

  

}


PID BackLiftPID(.8, 0);
void BackLiftTo(double bltarget, double maxblpower = 12, double maxbltime = 2000) {
  timer BLTimer;
  BLTimer.reset();
  BLTimer.time(msec);
  while(true) {
      double blposition = BLRotation.position(degrees);
      double blerror = bltarget - blposition;
      
      if(blerror > 180) {
        blerror -=360;
      }
      if(blerror < -180) {
        blerror += 360;
      }

      double blpower = BackLiftPID.calculateErr(blerror);

      if (blpower > maxblpower) {
        blpower = maxblpower;
      }
      if (blpower < -maxblpower) {
        blpower = - maxblpower;
      }

      if (std::abs(blerror) < 5 || BLTimer > maxbltime) {
        BackLift.stop(brake);
        break;
      }

      Brain.Screen.setCursor(1,1);
      Brain.Screen.print( "  BL Error   : %4.0f   BL Power: %4.0f",
      blerror, blpower );

      BackLift.spin(forward, blpower, volt);
      wait(20, msec);
    }
    BackLiftPID.reset();
  }


PID FrontLiftPID(1, 0);
void FrontLiftTo(double fltarget, double maxflpower = 12, double maxfltime = 2000) {
  timer FLTimer;
  FLTimer.reset();
  FLTimer.time(msec);
  while(true) {
      double flposition = FrontLift.position(degrees);
      double flerror = fltarget - flposition;
      
      if(flerror > 130) {
        flerror -=360;
      }
      if (flerror < -130) {
        flerror +=360;
      }
      double flpower = FrontLiftPID.calculateErr(flerror);

      if (flpower > maxflpower) {
        flpower = maxflpower;
      }
      if (flpower < -maxflpower) {
        flpower = - maxflpower;
      }

      if (std::abs(flerror) < 5 || FLTimer > maxfltime) {
        FrontLift.stop(brake);
        break;
      }

      Brain.Screen.setCursor(1,1);
      Brain.Screen.print( "  FL Error   : %4.0f   FL Power: %4.0f",
      flerror, flpower );
      FrontLift.spin(forward, flpower, volt);
      wait(20, msec);
    }
    FrontLiftPID.reset();
  }

  void fireFrontClamp()
  {
    FrontClamp.set(!FrontClamp);
  }
  void fireBackClamp()
  {
    BackClamp.set(!BackClamp);
    
  }

  void fireTilters()
  {
    BackTilter.set(!BackTilter);
  }
  void fireGoalCover()
  {
    GoalCover.set(!GoalCover);
    
  }


void armUp()
{
  FrontLift.spin(forward, 12, voltageUnits::volt);
  waitUntil(!Controller1.ButtonL1.pressing() );
  FrontLift.stop(brakeType :: hold);
}

void armDown()
{
  FrontLift.spin(reverse, 12, voltageUnits::volt);
  waitUntil(!Controller1.ButtonL2.pressing());
  {
    FrontLift.stop(brakeType :: hold);
  }
}

bool motorBrakeType = false;
void swapMotorDrive()
{
  motorBrakeType = !motorBrakeType;
  if(motorBrakeType)
  {
    FrontLeft.setBrake(brakeType :: hold);
    FrontRight.setBrake(brakeType :: hold);
    MiddleLeft.setBrake(brakeType :: hold);
    MiddleRight.setBrake(brakeType :: hold);
    BackLeft.setBrake(brakeType :: hold);
    BackRight.setBrake(brakeType :: hold);
  }
  else if(!motorBrakeType)
  {
    BackRight.setBrake(brakeType::coast);
  BackLeft.setBrake(brakeType::coast);
  FrontRight.setBrake(brakeType::coast);
  FrontLeft.setBrake(brakeType::coast);
  MiddleLeft.setBrake(brakeType::coast);
  MiddleRight.setBrake(brakeType::coast);
  }
}

bool isConveyorSpinning = false;
void spinConveyor()
{
  isConveyorSpinning = !isConveyorSpinning;
  if(isConveyorSpinning)
  {
    Conveyor.spin(reverse, 12, volt);
  }
  else if (!isConveyorSpinning)
  {
    Conveyor.stop();
  }
}

void reverseConveyor()
{
  if(isConveyorSpinning)
  {
    Conveyor.spin(forward, 12, volt);
  }
}


////////////////////////////// Toggle Functions //////////////////////////////

//// Hudsonville Robotics - Copy and pasted, ty
/////////////// Conveyor Toggle /////////////
double ConSpeed = 0; 
bool ConLast = false;
double ConPosition = 0;
double LastConPosition = 0;
int ConveyorControl() {
  while(true) {
  ConPosition = Conveyor.position(degrees);

	if(Controller1.ButtonRight.pressing()) {
		if(!ConLast)
		{
			//If not going forward
			if(ConSpeed != 95) 
			{
				//Go forward
				ConSpeed = 95; 
			}
			else {
				//Else stop
				ConSpeed = 0; 
			}
			ConLast = true;
		}
	} else if(Controller1.ButtonLeft.pressing()) {
		if(!ConLast)
		{
			//If not going reverse
			if(ConSpeed != -50) 
			{
				//Go reverse
				ConSpeed = -50;
			}
			else {
				//Else stop
				ConSpeed = 0;
			}
			ConLast = true;
		}
	}
	//when button is released, let go of latch.
	else {
		ConLast = false;
	}

  Conveyor.spin(forward, ConSpeed, pct);

if (std::abs(ConSpeed) > 0) {
    if (std::abs(Conveyor.velocity(pct)) < 1){
      Conveyor.spin(reverse, 50, pct);
      wait(500, msec);
      Conveyor.spin(forward, ConSpeed, pct); 
    }
  }

  // LastConPosition = ConPosition;
  wait(20, msec);
  }
  return(1);
}

//////////////// Claw Toggle //////////////
bool ClawState = false;
bool ClawLast = false;
int ClawControl() {
  while(true) {

    if(Controller1.ButtonL1.pressing()) {
      if(!ClawLast) {
        ClawState = !ClawState;
        ClawLast = true;
      }
    }

    else {
      ClawLast = false;
    }

    if(ClawState) {
      Claw.set(true);
    }
    else Claw.set(false);

  wait(20, msec);
  }
  return(1);
  
  

}

////////////// Drive Toggle ///////////
bool DriveState = false;
bool DriveLast = false;
int DriveToggle() {
  while(true) {
    if(Controller1.ButtonA.pressing()) {
      if(!DriveLast) {
        DriveState = !DriveState;
        DriveLast = true;
      }
    }
    else {
      DriveLast = false;
    }

    if(DriveState) {
      RobotDrive.setStopping(hold);
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.clearLine(1);
      Controller1.Screen.print("Drive Hold");

    }
    else {
      RobotDrive.setStopping(brake);
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.clearLine(1);
      Controller1.Screen.print("Drive Brake");
    }

    wait(20, msec);
  }
  return(1);
}



////////////////////////////// Display Task //////////////////////////////
int displayTask() {
    while(1) {
      // display some useful info
      Brain.Screen.setCursor(2,1);
      Brain.Screen.print( " FrontLeft Speed: %4.0f    Position: %6.2f", 
      FrontLeft.velocity( percent ), FrontLeft.position( degrees ) );
      Brain.Screen.newLine();
      Brain.Screen.print( " BackLeft Speed: %4.0f     Position: %6.2f", 
      BackLeft.velocity( percent ), BackLeft.position( degrees ));
      Brain.Screen.newLine();
      Brain.Screen.print( " FrontRight Speed: %4.0f   Position: %6.2f", 
      FrontRight.velocity( percent ), FrontRight.position( degrees ));
      Brain.Screen.newLine();
      Brain.Screen.print( " BackRight Speed: %4.0f    Position: %6.2f", 
      BackRight.velocity( percent ), BackRight.position( degrees ));
      Brain.Screen.newLine();
      Brain.Screen.newLine();

      // motor group velocity and position is returned for the first motor in the group
      Brain.Screen.print( " LeftDrive Speed: %4.0f    Position: %6.2f", 
      LeftDrive.velocity( percent ), LeftDrive.position( degrees ));
      Brain.Screen.newLine();
      Brain.Screen.print( " RightDrive Speed: %4.0f   Position: %6.2f", 
      RightDrive.velocity( percent ), RightDrive.position( degrees ));
      Brain.Screen.newLine();
      Brain.Screen.newLine();

      // drivetrain velocity is the average of the motor velocities for left and right
      Brain.Screen.print( " RobotDrive Speed: %4.0f   Position: %6.2f", 
      RobotDrive.velocity( percent ), BLRotation.position(degrees) );
      Brain.Screen.newLine();
      Brain.Screen.print( " Inertial Heading: %4.0f   Rotation: %4.0f", 
      Inertial.heading(degrees), Inertial.rotation(degrees) );
      wait(20, msec);
    }
}

int Display() {
  while(true) {
  Brain.Screen.setCursor(1, 1);
  Brain.Screen.print( "7110H");
/////////////////////////////// DriveTrain Info /////////////////////////////////
    Brain.Screen.setCursor(2, 1); //FrontLeft Temp
    Brain.Screen.print( "FrontLeft Temp: %4.0f", FrontLeft.temperature(fahrenheit));

    Brain.Screen.setCursor(2, 25); //BackLeft Temp
    Brain.Screen.print( "BackLeft Temp: %4.0f", BackLeft.temperature(fahrenheit));

    Brain.Screen.setCursor(3, 1); //FrontRight Temp
    Brain.Screen.print( "FrontRight Temp: %4.0f", FrontRight.temperature(fahrenheit));

    Brain.Screen.setCursor(3, 25); //BackRight Temp
    Brain.Screen.print( "BackRight Temp: %4.0f", BackRight.temperature(fahrenheit));

    Brain.Screen.setCursor(5, 1); //Left and Right Drive Position
    Brain.Screen.print( "LeftDrive: %4.0f", LeftDrive.position(degrees));
    Brain.Screen.setCursor(5, 25);
    Brain.Screen.print( "RightDrive: %4.0f", RightDrive.position(degrees));

/////////////////////////////// Extra Motor Info ///////////////////////////////////
    Brain.Screen.setCursor(7, 1); //FrontLift Position and Temp
    Brain.Screen.print( "FrontLift Temp: %4.0f", FrontLift.temperature(fahrenheit));
    Brain.Screen.setCursor(7, 25);
    Brain.Screen.print( "FrontLift Position: %4.0f", FLRotation.position(degrees));

    Brain.Screen.setCursor(8, 1); //BackLift Position and Temp
    Brain.Screen.print( "BackLift Temp: %4.0f", BackLift.temperature(fahrenheit));
    Brain.Screen.setCursor(8, 25);
    Brain.Screen.print( "BackLift Position: %4.0f", BLRotation.position(degrees));

    Brain.Screen.setCursor(9, 1); //Conveyor Position and Temp
    Brain.Screen.print( "Conveyor Temp: %4.0f", Conveyor.temperature(fahrenheit));
    Brain.Screen.setCursor(9, 25);
    Brain.Screen.print( "Conveyor Position: %4.0f", Conveyor.position(degrees));

    Brain.Screen.setCursor(10, 1); //GoalLock Position and Temp
    Brain.Screen.print( "GoalLock Temp: %4.0f", GoalLock.temperature(fahrenheit));
    Brain.Screen.setCursor(10, 25);
    Brain.Screen.print( "GoalLock Position: %4.0f", GoalLock.position(degrees));

///////////////////////// Inertial Heading and Rotation ////////////////////////////
    Brain.Screen.setCursor(12, 1); //Inertial Heading and Rotation
    Brain.Screen.print( "Inertial Heading: %4.0f", Inertial.heading());
    Brain.Screen.setCursor(12, 25);
    Brain.Screen.print( "Rotation: %4.0f", Inertial.rotation());

    wait(20, msec);
    }
}


////////////////////////////// Auton Functions //////////////////////////////////////
void pause() {
  wait(250, msec);
}

void BLDown() {
  BackLift.setVelocity(90, pct);
  BackLift.spin(reverse);
  wait(1.3, sec);
  BackLift.stop(brake);
  // return 1;
}
void BLUp() {
  BackLift.setVelocity(70, pct);
  BackLift.spin(forward);
  wait(1, sec);
  BackLift.stop(brake);
  // return 1;
}



//// BackLift Functions ////
void BackLiftMax() {
  BackLiftTo(75, 12, 2000);
}
void BackLiftDown() {
  BackLiftTo(340, 12, 1250);
}
void BackLiftUp() {
  BackLiftTo(73, 12, 2000);
}
void BackLiftNeutral() {
  BackLiftTo(30, 12, 1500);
}
void BackLiftAlliance() {
  BackLiftTo(80, 12, 2000);
  wait(200, msec);
  GoalLock.spin(forward, 100, pct);
  wait(600, msec); 
  GoalLock.spin(reverse, .1, volt);
  BackLiftTo(60, 12, 2000);
 }


void creep(double v)
{
  LeftDrive.spin(forward, v, voltageUnits::volt);
  RightDrive.spin(forward, v, voltageUnits::volt);
}

//// FrontLift Functions ////
void LiftMax() {
  FrontLiftTo(110, 12, 2000);
}
void LiftUp() {
  FrontLiftTo(90, 12, 2000);
}
void LiftDown() {
  LiftPID(0);
  //FrontLiftTo(357, 12, 2000);
}
void LiftRing() {
  FrontLiftTo(30, 12, 2000);
}
void StackGoal() {
  FrontLiftTo(75, 12, 2000);
  wait(500, msec);
  Claw.set(false);
  RobotDrive.driveFor(reverse, 1, inches, false);
  wait(250, msec);
  FrontLiftTo(85, 12, 2000);
}
void LiftDelayed() {
  wait(600, msec);
  FrontLiftTo(100, 12, 2000);
}


void forwardDriveRush(double v)
{
  FrontLeft.spin(fwd, v, volt);
  BackLeft.spin(fwd, v, volt);
  MiddleLeft.spin(fwd, v, volt);
  FrontRight.spin(fwd, v, volt);
  BackRight.spin(fwd, v, volt);
  MiddleRight.spin(fwd, v, volt);
}

//// GoalLock Functions ////
void GoalLockDown() {
  GoalLock.setVelocity(100, pct);
  GoalLock.spin(forward);
  wait(750, msec);
  GoalLock.stop(hold);
}
void GoalLockUp() {
  GoalLock.setVelocity(80, pct);
  GoalLock.spin(reverse);
  wait(.5, sec);
  GoalLock.stop(brake);
}

//// Ring Height ////
void BLRingHeight() {
  BackLift.spin(forward, 85, pct);
  wait(1.15, sec);
  BackLift.stop(brake);
  thread(GoalLockDown).detach();
  wait(750, msec);
  GoalLock.spin(forward, .25, voltageUnits::volt);
  BackLift.spinFor(reverse, 260, degrees);
  wait(250, msec);
  Conveyor.spin(forward, 95, pct);
}

void setMotorVelocity(double velocity)
{
  FrontLeft.setVelocity(velocity, pct);
  FrontRight.setVelocity(velocity, pct);
  MiddleLeft.setVelocity(velocity, pct);
  MiddleRight.setVelocity(velocity, pct);
  BackLeft.setVelocity(velocity, pct);
  BackRight.setVelocity(velocity, pct);
}

void forwardDrive(double Rev)
{
  FrontLeft.rotateFor(directionType :: fwd, Rev, rotationUnits :: rev, false);
  BackLeft.rotateFor(directionType :: fwd, Rev, rotationUnits :: rev, false);
  MiddleLeft.rotateFor(directionType :: fwd, Rev, rotationUnits :: rev, false);
  FrontRight.rotateFor(directionType :: fwd, Rev, rotationUnits :: rev, false);
  BackRight.rotateFor(directionType :: fwd, Rev, rotationUnits :: rev, false);
  MiddleRight.rotateFor(directionType :: fwd, Rev, rotationUnits :: rev, true);
}

void BLRingHeightNoRings() {
  BackLift.spin(forward, 85, pct);
  wait(1.15, sec);
  BackLift.stop(brake);
  thread(GoalLockDown).detach();
  wait(750, msec);
  GoalLock.spin(forward, .25, voltageUnits::volt);
  BackLift.spinFor(reverse, 260, degrees);
}

void MatchLoad(double mlangle) {
  Move(360, 4, mlangle, 3, 1500);
  pause();
  Move(-360, 4, mlangle, 3, 1500);
}

void ResetEncoders() {
  RightDrive.resetPosition();
  LeftDrive.resetPosition();
}

///////////////////////////////////////////////////////////////////////////////////

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

bool enableSelect = true;
void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  //// Claw State ////
  FrontClamp.set(true);
  BackClamp.set(false);
  GoalCover.set(true);
    FrontLift.setPosition(0, deg);


  //// Reset Sensors ////
  Inertial.calibrate();
  //Inertial.resetRotation();

  //// Set Brakes ////
  // GoalLock.setStopping(brake);
  // FrontLift.setStopping(brake);
  // BackLift.setStopping(brake);
  
  while(enableSelect) {
    
      for(int i = 0; i < 9; i++){
        autonButtons[i].render();

      if(autonButtons[i].isClicked()){

        for(int i = 0; i < 9; i++){
        autonButtons[i].buttonColor = vex::black;
        }
        autonButtons[i].buttonColor = vex::color(255, 31, 227);//(255,145,175);
        autonToRun = i;
        }

      }

      if (autonToRun == 0) {
      Brain.Screen.clearLine(1);
      Brain.Screen.setCursor(1 , 1);
      Brain.Screen.print( "RightWP - Right Neutral and Line of Rings");
      }
      
      if (autonToRun == 1) {
      Brain.Screen.clearLine(1);
      Brain.Screen.setCursor(1 , 1);
      Brain.Screen.print( "LeftWP - Left Neutral and Match Load");
      }

      if (autonToRun == 2) {
      Brain.Screen.clearLine(1);
      Brain.Screen.setCursor(1 , 1);
      Brain.Screen.print( "SoloWP - Tall Neutral");
      }

      if (autonToRun == 3) {
      Brain.Screen.clearLine(1);
      Brain.Screen.setCursor(1 , 1);
      Brain.Screen.print( "Right Tall - Tall Neutral and Line of Rings");
      }

      if (autonToRun == 4) {
      Brain.Screen.clearLine(1);
      Brain.Screen.setCursor(1 , 1);
      Brain.Screen.print( "Right Double Neutral + Alliance");
      }

      if (autonToRun == 5) {
      Brain.Screen.clearLine(1);
      Brain.Screen.setCursor(1 , 1);
      Brain.Screen.print( "SoloWP - Right Neutral");
      }

      if (autonToRun == 6) {
      Brain.Screen.clearLine(1);
      Brain.Screen.setCursor(1 , 1);
      Brain.Screen.print( "Left Mid and Neutral");
      }

      if (autonToRun == 7) {
      Brain.Screen.clearLine(1);
      Brain.Screen.setCursor(1 , 1);
      Brain.Screen.print( "SoloWP - Rings");
      }

      if (autonToRun == 8) {
      Brain.Screen.clearLine(1);
      Brain.Screen.setCursor(1 , 1);
      Brain.Screen.print( "Prog Skills - Stack");
      }
      wait(20, msec);
}


  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

//////////////////////////////////// Auton Routines ///////////////////////////////
void SimpleTest()
{
  thread liftThread = thread([]{LiftPID(3, 100, 3000);});
  liftThread.detach();
}

void RightWP() { //Auton 0 
  ResetEncoders(); // Reset Drive Position
  //Move(1000, 12, 90, 7, 1500);
  FrontClamp.set(false); 
  fireGoalCover();
  
  Move(670, 10, 0, 8, 1400);
  creep(2.0);
   // Grab Neutral
  wait(500, msec);
  fireFrontClamp();
  creep(0.0);

  // thread(BackLiftDown).detach();  // Reverse and lower BackLift | ily michael - mary
  //thread(GoalLockUp).detach();
  thread liftThread = thread([]{LiftPID(150, 100, 2500);});
  liftThread.detach();
  Move(-740, 6, 270, 2.3, 2500);
  wait(250, msec);

  //Move(0, 10, 280, 7, 1300);  // Turn and grab Alliance Goal
  //BackLiftDown();
  // RobotDrive.setDriveVelocity(50, pct);
  // RobotDrive.drive(reverse);
  fireBackClamp();
  wait(500, msec);
  RobotDrive.stop();
  RobotDrive.driveFor(forward, .65, inches,false);
  pause();

  //thread(BackLiftAlliance).detach();
  //FrontLift.spinFor(forward, 100, degrees, false);
  //wait(300, msec);

  Move(65, 6, 0, 8, 1500); // Move forward and intake
  pause();
  Conveyor.spin(reverse, 95, pct);
  Move(800, 3.1, 0, 5, 4500);
  pause();

  // thread(LiftDown).detach();
  wait(200, msec);                                
  Move(-900, 10, 0, 0, 1500);
  fireBackClamp();
  
}

void LeftWP() { //Auton 1
  ResetEncoders(); //Reset Drive Position
  FrontClamp.set(false);
  
  Move(800, 6, -45, 10, 2000); // Grab Neutral
  // pause();
  // fireBackClamp();
  // pause();
  // Conveyor.spin(reverse, 95, pct);
  // Move(950, 10, 95, 12, 1500);
  // creep(2.0);
  // wait(400, msec);
  // creep(0.0);
  // fireFrontClamp();
  // wait(100, msec);
  // thread liftThread = thread([]{LiftPID(150, 100, 2500);});
  // liftThread.detach();
  // Move(-550, 10, -180, 8.5, 1500);
  // Move(1400, 3, -180, 10, 5000);
  // Conveyor.stop();
  // fireBackClamp();
  // Move(100, 12, -180, 10, 500);
  


  // Claw.set(true);

  // Move(-880, 10, 16, 9, 15000);  //Reverse and drop Neutral
  // pause();
  // Claw.set(false);
  // pause();

  // FrontLift.setVelocity(80, pct);  //FrontLift up, turn, and pick up Alliance
  // FrontLift.spinFor(forward, 600, degrees, false);
  // Move(-240, 6, 0, 4, 1000);
  // pause();
  // Move(0, 0, 275, 9, 800);
  // pause();
  // thread(GoalLockUp).detach();  
  // BackLiftDown();
  // pause();
  
  // Move(-525, 10, 275, 7, 1000); //Drive back and forth for match loads
  // pause();
  // BackLiftTo(60, 12, 1000);
  // wait(250, msec);
  // BackLiftAlliance();
  // Conveyor.spin(forward, 95, pct);

  // Move(800, 2.3, 270, 2, 3750);
  // // pause();
  // // Move(480, 6, 280, 2, 1000);
  // // pause();
  // // Move(-480, 6, 280, 2, 700);
  // // pause();
  // // Move(480, 6, 280, 2, 1000);
  // pause();
  // BackLift.spinFor(reverse, 50, degrees, false);
  // GoalLock.spinFor(reverse, 50, degrees);
}

void SoloWP(int SoloMode) { //Auton 2
  ResetEncoders(); //Reset drive position
  

  FrontLift.setVelocity(50, pct);  //Deposit first ring
  FrontLift.spinFor(forward, 100, degrees);
  //Claw.set(false);
  creep(2);
  wait(300, msec);
  fireFrontClamp();
  creep(0);
  pause();
  Move(-300, 3, 0, 8, 5000);
  Move(0, 0, -70, 10, 500);
  Move(4900, 10, 0, 5.7, 2000);
  Move(0, 0, 180, 8, 1000);
  //Inertial.setRotation(0, degrees);
  //Move(-480, 10, 90, 8, 3000);
  //Move(0, 10, 180, 8, 250);
    //Move backwards and lower BackLift
  pause();
  Move(-1500, 5, 180, 7, 1000);
  wait(250, msec);
  fireBackClamp();
  wait(500, msec);
  fireTilters();
  Move(0, 0, 180, 8, 1000);
  FrontLift.spinFor(reverse, 100, degrees, false);
  
  // thread(BackLiftDown).detach();
  // thread(GoalLockUp).detach();

  //Move(-2350, 10, 180, 11, 2600);
  //RobotDrive.driveFor(reverse, 2, inches);

  //pause();
  //fireBackClamp();
  //RobotDrive.driveFor(forward, 1, inches);
  //BackLiftAlliance(); //BackLift to ring height and FrontLift down
  //pause();
    //RobotDrive.driveFor(forward, 2, inches);

  //Move(0, 0, -180, 10, 750);
  


  if(SoloMode == 1) {
  // thread liftThread = thread([]{LiftPID(0, 100, 1000);});
  // liftThread.join();
  wait(250, msec);
  Conveyor.spin(reverse, 95, pct);

  Move(2800, 9, 216, 8, 1000);
  creep(2.0);  //Grab Tall Neutral
  wait(750, msec);
  creep(0.0);
  fireFrontClamp();
  wait(100, msec);
  //FrontClamp.set(true);
  //pause();
  thread liftThread = thread([]{LiftPID(450, 100, 1500);});
  liftThread.detach();

  Move(3800, 10, 140, 8, 3000);
  Move(800, 8, 90, 9, 2000);

  //wait(2, sec);
  fireBackClamp();
  //thread(GoalLockUp).detach();
  //BackLift.spinFor(reverse, 50, degrees);
  }

  if (SoloMode == 2) {
  // thread liftThread = thread([]{LiftPID(3, 100, 1500);});
  // liftThread.detach();
  //wait(250, msec);
  Conveyor.spin(reverse, 69, pct);

  Move(1250, 6, 270, 3, 800);
    //Grab Neutral
  creep(3.0);
  wait(750, msec);
  fireFrontClamp();
  pause();
  creep(0);
  Move(-1900, 8, 270, 9, 700);
  FrontLift.spinFor(forward, 150, degrees, false);
  //thread liftThread = thread([]{LiftPID(100, 100, 1500);});
  //liftThread.detach();
  Move(950, 5, -40, 12, 750);
  Move(1500, 5, -90, 10, 2000);
  Move(-2700, 12, -80, 12, 1000);
  fireTilters();
  wait(250, msec);
  fireBackClamp();

 

  //thread(GoalLockUp).detach();
  //BackLift.spinFor(reverse, 50, degrees);
  }

  if (SoloMode == 3) {
  // FrontLiftTo(30, 12, 1500);
  wait(300, msec);

  Move(60, 6, 270, 8, 1500); // Move forward and intake
  pause();
  Conveyor.spin(forward, 95, pct);
  Move(850, 3.1, 270, 5, 4500);
  pause();

  wait(200, msec);                                
  Move(-900, 10, 270, 0, 1500);
  //BackLift.spinFor(reverse, 50, degrees, false);
  //GoalLock.spinFor(reverse, 50, degrees);
  }
}

void RightMid() { //Auton 3
  RightDrive.resetPosition(); //Reset Drive Position
  LeftDrive.resetPosition();
  FrontClamp.set(false);
  fireGoalCover();

  //FrontLift.spin(reverse, .25, volt);
  Move(1090, 12, 316, 12, 1000); //Grab Tall Neutral
  creep(2.0);
  wait(250, msec);
  creep(0);
  fireFrontClamp();
  thread liftThread = thread([]{LiftPID(150, 100, 2500);});
  liftThread.detach();
  //FrontLift.stop(brake);
  // thread(BackLiftDown).detach();    //Reverse and lower BackLift
  Move(-920, 10, 316, 8, 1500);
  Move(-600, 8, 270, 6, 1000);
  //thread(GoalLockUp).detach();
  wait(250, msec);
  fireBackClamp();

  Conveyor.spin(reverse, 95, pct);

  
    //Turn to align with Alliance
  //thread(BackLiftDown).detach();

  // RobotDrive.setDriveVelocity(50, pct); //Back up and grab Alliance
  // RobotDrive.drive(reverse);
  // wait(1500, msec);
  // RobotDrive.stop();
  // RobotDrive.driveFor(forward, 1, inches,false);
  // pause();
  
  // thread(BackLiftAlliance).detach();
  // FrontLift.spinFor(forward, 100, degrees, false);
  wait(300, msec);

  Move(70, 6, 0, 7.4, 1500); // Move forward and intake
  pause();
  //Conveyor.spin(forward, 95, pct);
  Move(850, 3.1, 0, 5, 4500);
  pause();

  // thread(LiftDown).detach();
  wait(200, msec);                                
  Move(-900, 10, 0, 0, 1500);
  fireBackClamp();
}

void DoubleNeutral() {
  ResetEncoders();  //Reset Drive Position
  Claw.set(false);
  FrontLift.spin(reverse, .25, volt);

  Move(880, 13, 0, 9, 1400); // Grab Neutral
  pause();
  Claw.set(true);
  FrontLift.stop(brake);

  Move(-500, 10, 355, 9, 15000); //Reverse and drop Neutral
  pause();
  Move(0, 0, 50, 10, 800);
  Claw.set(false);

  Move(-180, 8, 335, 7, 1500);  //Grab Tall Neutral
  pause();
  Move(807, 8.5, 332, 8, 1500);
  pause();
  Claw.set(true);
  pause();

  //Reverse and lower BackLift
  // Move(-810, 8, 320, 9, 1250);
  Move(-740, 8, 320, 9, 1250);
  thread(GoalLockUp).detach();
  Claw.set(false);
  wait(500, msec);
  // Move(0, 11, 260, 12, 1000);  //Turn to align with Alliance
  Move(0, 0, 270, 10, 800);
  pause();
  thread(BackLiftDown).detach();
  wait(750, msec);

  RobotDrive.setDriveVelocity(50, pct); //Back up and grab Alliance
  RobotDrive.drive(reverse);
  wait(1500, msec);
  RobotDrive.stop();
  RobotDrive.driveFor(forward, .75, inches,false);
  pause();
  thread(BackLiftAlliance).detach();
  wait(1500, msec);

  thread(LiftDown).detach();
  Move(0, 0, 320, 10, 800);
  Conveyor.spin(forward, 95, pct);
  Move(260, 7, 325, 5, 1000);
  pause();
  Claw.set(true);
  FrontLift.stop(brake);
  pause();
  Move(-400, 10, 5, 7, 1000);
  BackLift.spinFor(reverse, 50, degrees, false);
  GoalLock.spinFor(reverse, 50, degrees);
}

void newProgSkills()
{
  ResetEncoders(); //Reset Drive Position
  FrontClamp.set(false);

  Move(-150, 6, 0, 10, 500); // Grab Neutral
  pause();
  fireBackClamp();
  pause();
  Conveyor.spin(reverse, 95, pct);
  Move(950, 10, 95, 12, 1500);
  creep(2.0);
  wait(500, msec);
  creep(0.0);
  fireFrontClamp();
  wait(100, msec);
  thread liftThread = thread([]{LiftPID(480, 100, 2500);});
  liftThread.detach();
  Move(1125, 6, 125, 7, 2000);
  LiftPID(340, 1000);
  wait(500, msec);
  fireFrontClamp();
  Move(-200, 10, 180, 8, 2500);
  fireBackClamp();
  liftThread = thread([]{LiftPID(0, 100, 3000);});
  liftThread.join();
  Move(200, 10, 180, 8, 1500);
  Move(0, 10, 0, 6, 1000);
  Move(180, 5, 0, 3, 2000);
  fireFrontClamp();
  liftThread = thread([]{LiftPID(10, 100, 1500);});
  liftThread.detach();
  Move(0, 10, 180, 8, 2000);
  //TurnonPID(-180, 300, 1000);
  wait(1000, msec);
  Move(-1000, 10, 180, 8, 2000);
  fireBackClamp();
  liftThread = thread([]{LiftPID(360, 100, 3000);});
  liftThread.detach();
  Move(0, 10, 181, 7, 2000);

  Move(820, 10, 180, 7, 2000);
  Move(0, 10, 90, 7, 1000);
  creep(3.0);
  wait(750, msec);
  creep(0);
  fireFrontClamp();
  Move(0, 10, 180, 8, 1000);
  Move(600, 10, 180, 8, 2000);
  liftThread = thread([]{LiftPID(0, 100, 3000);});
  liftThread.detach();
  Move(650, 10, 273, 8, 1000);
  creep(2.0);
  wait(1000, msec);
  creep(0.0);
  fireFrontClamp();
  wait(100, msec);
  liftThread = thread([]{LiftPID(450, 100, 3000);});
  liftThread.detach();
  Move(1500, 5, 262, 6, 6000);
  Inertial.setRotation(0, degrees);
  creep(-2.0);
  wait(500, msec);
  //TurnonPID(0, 300, 1000);
  Move(0, 10, 90, 8, 1000);
  Move(-600, 5, 90, 8, 3000);
  Move(350, 2, 90, 3, 6000);
  LiftPID(3);
  
  Move(885, 12, 90, 0, 3000);
  FrontLeft.stop(brakeType :: hold);
    FrontRight.stop(brakeType :: hold);
    MiddleLeft.stop(brakeType :: hold);
    MiddleRight.stop(brakeType :: hold);
    BackLeft.stop(brakeType :: hold);
    BackRight.stop(brakeType :: hold);
  wait(0.5, sec);
  fireBackClamp();

  //Move(0, 10, 180, 8, 1000);

 

  
}

void parkTest()
{
    
    FrontLift.setBrake(hold);
    FrontLift.resetPosition();
    LiftPID(450);
    FrontClamp.set(true);
    fireBackClamp();
    FrontLeft.setBrake(brakeType::hold);
    FrontRight.setBrake(brakeType::hold);
    MiddleRight.setBrake(brakeType::hold);
    MiddleLeft.setBrake(brakeType::hold);
    BackRight.setBrake(brakeType::hold);
    BackRight.setBrake(brakeType::hold);
    setMotorVelocity(10);
    forwardDrive(3);
    LiftPID(30, 100, 1500);
    setMotorVelocity(80);
    forwardDrive(6.9);
    forwardDrive(0.000001);
    fireBackClamp();




}


void BetterProgSkills(bool sPark = false) {
  ResetEncoders();  //Reset Drive Position
  FrontLift.setBrake(brake);
  Claw.set(false);
  thread(GoalLockUp).detach();

  BackLiftDown(); //Grab Alliance and Neutral
  Move(-480, 6, 0, 7, 1200);
  BackLiftTo(65, 12, 1500);
  FrontLift.spin(reverse, .25 ,volt);
  pause();
  Move(0, 0, 90, 10, 800);
  Move(1300, 8, 90, 7, 2500);
  // Move(1600, 6, 99, 9, 2500);
  pause();
  Claw.set(true);
  FrontLift.stop(brake);
  pause();

  thread(LiftMax).detach();
  wait(100, msec);
  Move(1150, 6, 125, 7, 2000);  //Stack Goal
  pause();
  FrontLiftTo(75, 12, 800);
  Move(0, 0, 105, 7, 800);
  wait(500, msec);

  Claw.set(false);
  RobotDrive.driveFor(reverse, 1, inches, false);
  wait(200, msec);
  FrontLiftTo(85, 12, 1000);

  thread(BackLiftDown).detach();
  Move(-520, 6, 120, 7, 1000);  //Back up and lower Alliance goal
  thread(LiftDown).detach();
  Move(360, 6, 120, 7, 1000);

  thread(BackLiftMax).detach(); 
  wait(900, msec);
  Move(0, 0, 297, 7, 1000);
  Move(520, 6, 297, 7, 1000);
  pause();
  Claw.set(true);

  Move(0, 0, 120, 7, 1500); //Turn

  thread(LiftMax).detach();
  wait(500, msec);

  Move(850, 6, 130, 7, 1500); //Score Alliance
  pause();
  StackGoal();
  thread(BackLiftDown).detach();
  wait(250, msec);

  Move(-1250, 7, 178, 8, 2000);  //Pick up new Alliance
  pause();
  RobotDrive.driveFor(forward, .75, inches);
  thread(BackLiftAlliance).detach();
  FrontLiftTo(30, 12, 1500);
  wait(600, msec);

  Move(450, 6, 210, 7, 3000);
  Conveyor.spin(forward, 95, pct);
  pause();
  Move(1600, 4, 180, 7, 7000); //Score rings
  pause();

  Move(-110, 6, 180, 7, 1500);
  Move(0, 0, 270, 7, 1500);

  thread(LiftDown).detach();  //Grab Neutral
  Move(500, 4, 270, 7, 1500);
  Claw.set(true);
  pause();

  thread(LiftUp).detach();
  
  if (sPark) {
  Move(1550, 5, 260, 7, 5000);
  pause();

  Move(-50, 6, 260, 0, 1500);  //Align and drop Platform
  pause();
  Move(0, 10, 0, 9, 1500);
  pause();
  FrontLiftTo(340, 12, 2000);

  //Park plz work
  Move(1000, 12, 0, 11, 5000);
  // Park(11);
  RobotDrive.stop(hold);
  }

  else {
  Move(1030, 6, 296, 7, 2000);  //Stack Neutral
  pause();
  StackGoal();

  thread(GoalLockUp).detach();
  BackLift.spin(reverse, 35, pct);
  Move(-580, 6, 300, 7, 1500);  //Back up and lower Alliance goal
  wait(500, msec);
  thread(LiftDown).detach();
  Move(310, 6, 300, 7, 1500);
  BackLift.stop(brake);

  Move(0, 0, 180, 9, 1250);     //Turn and pick up Alliance
  thread(BackLiftMax).detach();
  wait(500, msec);
  Move(0, 0, 125, 7, 1250);
  pause();
  Move(460, 6, 125, 7, 1750);
  pause();
  Claw.set(true);

  Move(0, 0, 300, 7, 1500); //Turn and score Alliance
  thread(LiftUp).detach();
  Move(1090, 7, 305, 6, 1500); 
  StackGoal();
  // wait(250, msec);
  // Claw.set(false);

  Move(-200, 6, 300, 7, 1500);  //Reverse and pick up last Alliance
  pause();
  thread(LiftDown).detach();
  Move(0, 10, 180, 9, 1500);
  Move(1000, 7, 180, 8, 1500);
  pause();
  Claw.set(true);

  pause();  //Reverse and push over Tall Neutral
  thread(BackLiftDown).detach();
  thread(LiftDelayed).detach();
  Move(-2540, 10, 210, 11, 5000);
  pause();

  // pause();
  // thread(ProgSkillsMid).detach();
  // wait(100, msec);
  // thread(BackLiftDown).detach();
  // wait(350, msec);
  // LiftMax();
  // wait(1, sec);

  Move(1100, 8, 135, 8.5, 1500); //Stack last Alliance
  pause();
  Claw.set(false);
  pause();
  Move(-200, 6, 130, 7, 1500);
  }
}


void LeftMid() {
  ResetEncoders();
  Claw.set(false);
  
  thread(LiftRing).detach();
  thread(GoalLockUp).detach();

  // Conveyor.spin(forward, 95, pct);
  Move(1100, 7.5, 90, 3, 4000);
  Move(-360, 8, 45, 5, 1000);
  thread(LiftDown).detach();
  wait(250, msec);
  Move(700, 7, 45, 5, 1500);
  wait(250, msec);
  Claw.set(true);

  thread(BackLiftDown).detach();
  wait(750, msec);
  Move(0, 10, 90, 8, 1000);
  wait(250, msec);
  Move(-720, 9, 90, 7, 1000);
  thread(BackLiftNeutral).detach();
  Move(-1200, 11, 350, 12, 3000);
}

//////// Test Autons ////////
void BoxTest() {
  while (true) {
  Move(360, 10, 0, 9);
  pause();
  Move(0, 10, 270, 10);
  pause();
  Move(360, 10, 270, 9);
  pause();
  Move(0, 10, 180, 10);
  pause();
  Move(360, 10, 180, 9);
  pause();
  Move(0, 10, 90, 10);
  pause();
  Move(360, 10, 90, 10);
  pause();
  Move(0, 10, 0, 10);
  }
}

void Tuning() {
  while(true) {
  Move(400, 10, 0, 9);
  pause();
  Move(-400, 10, 0, 9);
  pause();
  Move(0, 10, 90, 10);
  pause();
  Move(0, 10, 180, 10);
  pause();
  Move(0, 10, 0, 10);
  pause();
}
}

void TurnTuning() {
  Move(0, 0, 90, 7, 1500);
  pause();
  Move(0, 0, 0, 7, 1500);
  pause();
  Move(0, 0, 270, 7, 1500);
  pause();
  Move(0, 0, 0, 7, 1500);
}

void FrontLiftTest() {
  FrontLiftTo(90, 12);
  wait(.5, sec);
  FrontLiftTo(357, 12);
}

void BackLiftTest() {
  BackLiftTo(80, 12);
  GoalLockDown();
  wait(.5, sec);
  GoalLockUp();
  BackLiftTo(340, 12);
}


void RightRush()
{
  fireGoalCover();
  FrontClamp.set(false);
  forwardDriveRush(12);
  wait(0.8, sec);
  BRAKE();
  wait(0.1, sec);
  fireFrontClamp();
  forwardDriveRush(-12.0);
}

void ProgRingTest() {
  RobotDrive.driveFor(forward, .75, inches);
  thread(BackLiftAlliance).detach();
  Conveyor.spin(forward, 95, pct);
  FrontLiftTo(50, 12, 1500);
  wait(600, msec);

  Move(480, 6, 25, 7, 3000);
  pause();
  Move(1550, 3, 0, 7, 65000); //Score rings
  pause();

  Move(-110, 6, 0, 7, 1500);
  Move(0, 0, 90, 7, 1500);

  thread(LiftDown).detach();  //Grab Neutral
  wait(500, msec);
  Move(500, 6, 90, 7, 1000);
  Claw.set(true);
}

void MotionProfileTest()
{
  
   std::vector<float> test = profileGeneration(2);
   // LiftPID(10);
  // for(int i = 0; i < 500; i++)
  // {
  //   test.push_back(0.25);
  // }
  // test.push_back(0);
  // profileFollow(test);
  profileFollow( test ) ;
  
}

////////////////////////////////////////////////////////////////////////////////

void autonomous(void) {
  // Brain.Screen.clearScreen();
  //Inertial.resetRotation();
  RightDrive.resetPosition();
  LeftDrive.resetPosition();

  
  
  //Move(0, 0, 90, 7, 3000);
  //Move(0, 0, 270, 7, 1500);
  //Move(0, 0, 0, 7, 1500);
  
  if (autonToRun == 0) {
      //Move(0, 0, 90, 7, 3000);
    RightWP();
//Move(-480, 10, 180, 6, 3000);     
     //Move(2000, 10, 45, 8, 4000);
     //MotionProfileTest();
     
  }

  if (autonToRun == 1) {
     LeftWP();
  }  

  if (autonToRun == 2) {
     SoloWP(1);
  }

  if (autonToRun == 3) {
     SoloWP(1);
  }

  if (autonToRun == 4) {
    RightRush();
  }
  
  if (autonToRun == 5) {
     SoloWP(2);
  }

  if (autonToRun == 6) {
      LeftMid();
  }

  if (autonToRun == 7) {
    SoloWP(3);
  }

  if (autonToRun == 8) {
     BetterProgSkills(false);
     parkTest();
     newProgSkills();
  }
  
  
  

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {

  enableSelect = false;
  Brain.Screen.clearScreen(black);
  vex::task runDisplayTask = vex::task(Display);
  Conveyor.setVelocity(100, pct);
  //vex::task runConveyorControl = vex::task(ConveyorControl);
  // vex::task runClawControl = vex::task(ClawControl);
  // vex::task runDriveToggle = vex::task(DriveToggle);

  //// Drivetrain Settings ////
  RobotDrive.setStopping(coast);

  //// Front Lift Settings ////
  // FrontLift.setVelocity(100, pct);

  //// Back Lift Settings ////
  // BackLift.setVelocity(80, pct);
  
  // User control code here, inside the loop
  while (1) {
    //////////////////////// Drivetrain Controls ////////////////////////
  //Read Controller inputs
    double DriveSpeed = Controller1.Axis3.position(percent) ;
    double TurnSpeed = ( Controller1.Axis1.position(percent)  );

    // double LeftStickPos = ( std::abs( Controller1.Axis3.position(percent) ) + 
    // std::abs( Controller1.Axis4.position(percent) ) );
    // double RightStickPos = ( std::abs( Controller1.Axis1.position(percent) ) + 
    // std::abs( Controller1.Axis1.position(percent) ) );
  
  //Deadband
    // if (std::abs(DriveSpeed) < 10) {
    //  DriveSpeed = 0; 
    // }
    // if (std::abs(TurnSpeed) < 10) {
    //   TurnSpeed = 0;
    // } 

  //Output to Drivetrain
    LeftDrive.spin(forward, (DriveSpeed + TurnSpeed*0.9)*0.13, volt);
    RightDrive.spin(forward, (DriveSpeed - TurnSpeed*0.9)*0.13, volt);

    // if (LeftStickPos > 1) {
    //   LeftDrive.setStopping(coast);
    //   RightDrive.setStopping(coast);
    // }
    // else if (RightStickPos > 1) {
    //   LeftDrive.setStopping(brake);
    //   RightDrive.setStopping(brake);
    // }
  //////////////////////// Front Lift Controls ////////////////////////
  // if (Controller1.ButtonL1.pressing()) {
  //   FrontLift.spin(forward, 13, volt);
  // }
  // else if(Controller1.ButtonL2.pressing()) {
  //   FrontLift.spin(reverse, 13, volt);
  // }
  // else FrontLift.stop(brake);

  //////////////////////// Back Clamp Controls ////////////////////////
  //Controller1.ButtonR2.pressed(fireBackClamp); 
    
  
  
  
  //////////////////////// Claw/Pneumatic Controls ////////////////////////
  //Controller1.ButtonR1.pressed(fireFrontClamp);

  /////// Goal Lock //////
  

    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);
      Controller1.ButtonL1.pressed(armUp);
      Controller1.ButtonL2.pressed(armDown);
      Controller1.ButtonR1.pressed(fireFrontClamp);
      Controller1.ButtonY.pressed(fireTilters);
      Controller1.ButtonR2.pressed(fireBackClamp);
      Controller1.ButtonB.pressed(reverseConveyor);
      Controller1.ButtonX.pressed(spinConveyor);
      Controller1.ButtonA.pressed(swapMotorDrive);
      


  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
