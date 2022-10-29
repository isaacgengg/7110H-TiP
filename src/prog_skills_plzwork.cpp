// void PogProg() {
//   ResetEncoders();  //Reset Drive Position
//   FrontLift.setBrake(brake);
//   Claw.set(false);
//   Move(80, 6, 0, 5, 750);
//   Claw.set(true);
//   LiftUp();
//   Move(0, 0, 92, 9, 1000);
//   thread(BackLiftDown).detach();
//   Move(880, 8, 92, 7, 1000);
  

// }


// void ProgSkills(bool skillsPark = false) {
//   ResetEncoders();  //Reset Drive Position
//   FrontLift.resetPosition();
//   FrontLift.setBrake(brake);
//   FrontLift.stop(brake);
//   Claw.set(false);

//   thread(GoalLockUp).detach();
//   BLDown(); //Grab Alliance and Neutral
//   Move(-480, 6, 0, 7, 1200);
//   BackLift.spinFor(forward, 500, degrees, false);
//   wait(1, sec);
//   FrontLift.spin(forward, .1, volt);
//   Move(1360, 6, 103, 7, 2000);
//   pause();
//   Claw.set(true);
//   FrontLift.stop(brake);
//   pause();

//   FrontLift.setVelocity(80, pct); 
//   FrontLift.spinTo(700, degrees, false);

//   Move(1210, 6, 120, 7, 2000);  //Stack Goal
//   pause();
//   FrontLift.spinFor(reverse, 200, degrees, false);
//   pause();
//   Move(0, 0, 112, 7, 800);
//   pause();
//   FrontLift.spinFor(reverse, 75, degrees, false);
//   wait(800, msec);
//   Claw.set(false);
//   RobotDrive.driveFor(reverse, 1, inches, false);
//   pause();
//   FrontLift.spinFor(forward, 300, degrees, false);
//   wait(500, msec);

//   BackLift.spin(reverse, 85, pct);
//   Move(-520, 6, 120, 7, 1000);  //Back up and lower Alliance goal
//   FrontLift.setVelocity(100, pct);
//   FrontLift.spin(reverse);
//   BackLift.stop(brake);
//   wait(1.5, sec);  
//   FrontLift.stop(brake);
//   Move(430, 6, 120, 7, 1000);

//   BackLift.spin(forward, 80, pct); //Turn and pick up Alliance
//   wait(1, sec);
//   BackLift.stop(brake);
//   pause();
//   Move(0, 0, 297, 7, 1000);
//   Move(460, 6, 297, 7, 1000);
//   pause();
//   Claw.set(true);

//   Move(0, 0, 120, 7, 1500); //Turn

//   FrontLift.setVelocity(80, pct);
//   FrontLift.spinTo(700, degrees, false);
//   wait(500, msec);

//   Move(800, 6, 125, 7, 1500); //Score Alliance
//   pause();
//   FrontLift.spinFor(reverse, 300, degrees, false);
//   wait(850, msec);
//   Claw.set(false);
//   BackLift.spin(reverse);
//   RobotDrive.driveFor(reverse, 1, inches, false);
//   pause();
//   FrontLift.spinFor(forward, 300, degrees, false);
//   wait(1, sec);
//   BackLift.stop(brake);

//   Move(-1250, 7, 178, 8.5, 2000);  //Pick up new Alliance
//   pause();
//   RobotDrive.driveFor(forward, .75, inches);
//   BLRingHeight();
//   FrontLift.spinFor(reverse, 500, degrees, false);
//   Move(430, 6, 210, 7, 1000);
//   pause();
//   Move(1550, 3, 180, 7, 10000); //Score rings
//   pause();
//   Move(-100, 6, 180, 7, 1500);
//   Move(0, 0, 270, 7, 1500);

//   FrontLift.setVelocity(75, pct); //Pick up Neutral
//   FrontLift.spin(reverse);
//   wait(1.4, sec);
//   FrontLift.stop(brake);
//   pause();
//   Move(500, 5, 270, 7, 1000);
//   Claw.set(true);
//   pause();

//   FrontLift.setVelocity(60, pct); //Align with wall, while raising FrontLift
//   FrontLift.spinFor(forward, 700, degrees, false);
  
//   if (skillsPark) {
//   Move(1550, 5, 260, 7, 5000);
//   pause();

//   Move(-50, 6, 260, 0, 1500);  //Align and drop Platform
//   pause();
//   Move(0, 10, 0, 9, 1500);
//   pause();
//   FrontLift.setVelocity(90, pct);
//   FrontLift.spin(reverse);
//   wait(1.4, sec);
//   //Park plz work
//   Move(800, 12, 0, 11, 5000);
//   RobotDrive.stop(brake);
//   }


//   else {
//   Move(1100, 6, 300, 7, 2000);  //Stack Goal
//   pause();
//   FrontLift.spinFor(reverse, 300, degrees, false);
//   wait(1, sec);
//   Claw.set(false);
//   RobotDrive.driveFor(reverse, 1, inches, false);
//   pause();
//   FrontLift.spinFor(forward, 300, degrees, false);
//   wait(500, msec);


//   thread(GoalLockUp).detach();
//   BackLift.spin(reverse, 20, pct);
//   Move(-680, 6, 300, 7, 1500);  //Back up and lower Alliance goal
//   pause();
//   FrontLift.setVelocity(80, pct);
//   FrontLift.spin(reverse);
//   wait(2, sec);  
//   BackLift.stop(brake);
//   FrontLift.stop(brake);
//   Move(430, 6, 300, 7, 1500);
//   BackLift.spin(forward, 100, pct);

//   wait(800, msec);
//   Move(0, 0, 117, 7, 1500);
//   BackLift.stop(brake);
//   Move(500, 6, 115, 7, 1500);
//   pause();
//   Claw.set(true);

//   Move(0, 0, 300, 7, 1500); //Turn

//   FrontLift.setVelocity(80, pct);
//   FrontLift.spinTo(700, degrees, false);
//   wait(500, msec);

//   Move(850, 6, 310, 7, 1500); //Score Alliance
//   pause();
//   FrontLift.spinFor(reverse, 300, degrees, false);
//   wait(850, msec);
//   Claw.set(false);
//   RobotDrive.driveFor(reverse, 1, inches, false);
//   pause();
//   FrontLift.spinFor(forward, 300, degrees, false);
  
//   Move(-200, 6, 300, 7, 1500);  //Reverse and pick up Alliance
//   pause();
//   FrontLift.setVelocity(80, pct);
//   FrontLift.spin(reverse);
//   Move(0, 10, 180, 9, 1500);
//   wait(1.5, sec);
//   FrontLift.stop(brake);
//   Move(1000, 5, 180, 6, 2500);
//   pause();
//   Claw.set(true);

//   thread(BLDown).detach();
//   wait(500, msec);
//   FrontLift.setVelocity(20, pct);
//   FrontLift.spinFor(forward, 600, degrees, false);
//   Move(-2540, 8, 212, 9, 10000);
//   pause();
//   Move(900, 6, 120, 7, 3000);

//   pause();
//   // FrontLift.spinFor(reverse, 180, degrees, false);
//   wait(250, msec);
//   Claw.set(false);
//   pause();
//   // RobotDrive.driveFor(reverse, 1, inches, false);
//   // pause();
//   // FrontLift.spinFor(forward, 300, degrees, false);
//   Move(-100, 6, 120, 7, 1500);

//   }
// }


// void ProgSkills(bool skillsPark = false) {
//   ResetEncoders();  //Reset Drive Position
//   FrontLift.resetPosition();
//   FrontLift.setBrake(brake);
//   FrontLift.stop(brake);
//   Claw.set(false);

//   thread(GoalLockUp).detach();
//   BLDown(); //Grab Alliance and Neutral
//   Move(-480, 6, 0, 7, 1200);
//   BackLift.spinFor(forward, 500, degrees, false);
//   wait(1, sec);
//   FrontLift.spin(forward, .1, volt);
//   Move(1360, 6, 103, 7, 2000);
//   pause();
//   Claw.set(true);
//   FrontLift.stop(brake);
//   pause();

//   FrontLift.setVelocity(80, pct); 
//   FrontLift.spinTo(700, degrees, false);

//   Move(1210, 6, 120, 7, 2000);  //Stack Goal
//   pause();
//   FrontLift.spinFor(reverse, 200, degrees, false);
//   pause();
//   Move(0, 0, 112, 7, 800);
//   pause();
//   FrontLift.spinFor(reverse, 75, degrees, false);
//   wait(800, msec);
//   Claw.set(false);
//   RobotDrive.driveFor(reverse, 1, inches, false);
//   pause();
//   FrontLift.spinFor(forward, 300, degrees, false);
//   wait(500, msec);

//   BackLift.spin(reverse, 85, pct);
//   Move(-520, 6, 120, 7, 1000);  //Back up and lower Alliance goal
//   FrontLift.setVelocity(100, pct);
//   FrontLift.spin(reverse);
//   BackLift.stop(brake);
//   wait(1.5, sec);  
//   FrontLift.stop(brake);
//   Move(430, 6, 120, 7, 1000);

//   BackLift.spin(forward, 80, pct); //Turn and pick up Alliance
//   wait(1, sec);
//   BackLift.stop(brake);
//   pause();
//   Move(0, 0, 297, 7, 1000);
//   Move(460, 6, 297, 7, 1000);
//   pause();
//   Claw.set(true);

//   Move(0, 0, 120, 7, 1500); //Turn

//   FrontLift.setVelocity(80, pct);
//   FrontLift.spinTo(700, degrees, false);
//   wait(500, msec);

//   Move(800, 6, 125, 7, 1500); //Score Alliance
//   pause();
//   FrontLift.spinFor(reverse, 300, degrees, false);
//   wait(850, msec);
//   Claw.set(false);
//   BackLift.spin(reverse);
//   RobotDrive.driveFor(reverse, 1, inches, false);
//   pause();
//   FrontLift.spinFor(forward, 300, degrees, false);
//   wait(1, sec);
//   BackLift.stop(brake);

//   Move(-1250, 7, 178, 8.5, 2000);  //Pick up new Alliance
//   pause();
//   RobotDrive.driveFor(forward, .75, inches);
//   BLRingHeight();
//   FrontLift.spinFor(reverse, 500, degrees, false);
//   Move(430, 6, 210, 7, 1000);
//   pause();
//   Move(1550, 3, 180, 7, 10000); //Score rings
//   pause();
//   Move(-100, 6, 180, 7, 1500);
//   Move(0, 0, 270, 7, 1500);

//   FrontLift.setVelocity(75, pct); //Pick up Neutral
//   FrontLift.spin(reverse);
//   wait(1.4, sec);
//   FrontLift.stop(brake);
//   pause();
//   Move(500, 5, 270, 7, 1000);
//   Claw.set(true);
//   pause();

//   FrontLift.setVelocity(60, pct); //Align with wall, while raising FrontLift
//   FrontLift.spinFor(forward, 700, degrees, false);
  
//   if (skillsPark) {
//   Move(1550, 5, 260, 7, 5000);
//   pause();

//   Move(-50, 6, 260, 0, 1500);  //Align and drop Platform
//   pause();
//   Move(0, 10, 0, 9, 1500);
//   pause();
//   FrontLift.setVelocity(90, pct);
//   FrontLift.spin(reverse);
//   wait(1.4, sec);
//   //Park plz work
//   Move(800, 12, 0, 11, 5000);
//   RobotDrive.stop(brake);
//   }


//   else {
//   Move(1100, 6, 300, 7, 2000);  //Stack Goal
//   pause();
//   FrontLift.spinFor(reverse, 300, degrees, false);
//   wait(1, sec);
//   Claw.set(false);
//   RobotDrive.driveFor(reverse, 1, inches, false);
//   pause();
//   FrontLift.spinFor(forward, 300, degrees, false);
//   wait(500, msec);


//   thread(GoalLockUp).detach();
//   BackLift.spin(reverse, 20, pct);
//   Move(-680, 6, 300, 7, 1500);  //Back up and lower Alliance goal
//   pause();
//   FrontLift.setVelocity(80, pct);
//   FrontLift.spin(reverse);
//   wait(2, sec);  
//   BackLift.stop(brake);
//   FrontLift.stop(brake);
//   Move(430, 6, 300, 7, 1500);
//   BackLift.spin(forward, 100, pct);

//   wait(800, msec);
//   Move(0, 0, 117, 7, 1500);
//   BackLift.stop(brake);
//   Move(500, 6, 115, 7, 1500);
//   pause();
//   Claw.set(true);

//   Move(0, 0, 300, 7, 1500); //Turn

//   FrontLift.setVelocity(80, pct);
//   FrontLift.spinTo(700, degrees, false);
//   wait(500, msec);

//   Move(850, 6, 310, 7, 1500); //Score Alliance
//   pause();
//   FrontLift.spinFor(reverse, 300, degrees, false);
//   wait(850, msec);
//   Claw.set(false);
//   RobotDrive.driveFor(reverse, 1, inches, false);
//   pause();
//   FrontLift.spinFor(forward, 300, degrees, false);
  
//   Move(-200, 6, 300, 7, 1500);  //Reverse and pick up Alliance
//   pause();
//   FrontLift.setVelocity(80, pct);
//   FrontLift.spin(reverse);
//   Move(0, 10, 180, 9, 1500);
//   wait(1.5, sec);
//   FrontLift.stop(brake);
//   Move(1000, 5, 180, 6, 2500);
//   pause();
//   Claw.set(true);

//   thread(BLDown).detach();
//   wait(500, msec);
//   FrontLift.setVelocity(20, pct);
//   FrontLift.spinFor(forward, 600, degrees, false);
//   Move(-2540, 8, 212, 9, 10000);
//   pause();
//   Move(900, 6, 120, 7, 3000);

//   pause();
//   // FrontLift.spinFor(reverse, 180, degrees, false);
//   wait(250, msec);
//   Claw.set(false);
//   pause();
//   // RobotDrive.driveFor(reverse, 1, inches, false);
//   // pause();
//   // FrontLift.spinFor(forward, 300, degrees, false);
//   Move(-100, 6, 120, 7, 1500);

//   }
// }
