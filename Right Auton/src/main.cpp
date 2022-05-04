/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Jacob Krebs                                               */
/*    Created:      Thu Jan 27 2022                                           */
/*    Description:  RingBot v5                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;

// MOTOR DEF
  motor rightFrontMotor = motor(PORT8, ratio18_1, false);
  motor rightMidMotor = motor(PORT9, ratio18_1, true);
  motor rightBackMotor = motor(PORT10, ratio18_1, false);
  motor leftFrontMotor = motor(PORT11, ratio18_1, true);
  motor leftMidMotor = motor(PORT12, ratio18_1, false);
  motor leftBackMotor = motor(PORT13, ratio18_1, true);

  motor liftMotor = motor(PORT20, ratio36_1, false);

  motor intake = motor(PORT14, ratio18_1, false);

  controller Master = controller();

  inertial IMU = inertial(PORT19);

  triport ThreeWirePort = triport(PORT22); 
  pneumatics frontClaw = pneumatics(ThreeWirePort.C);
  pneumatics backClaw = pneumatics(ThreeWirePort.G);
  pneumatics tilter = pneumatics(ThreeWirePort.H);

  competition Competition;

// GLOBAL VARS

  const int UP = 1;
  const int DOWN = 0;
///////////////   DRIVE       ///////////////
  /*          Drive Vars              */
    double drivekp = 0;
    double drivekd = 0;
  /*          Drive Fucntions         */

    // HELPER FUNCTIONS
      /**
        Resets drive sensors
      */
      void resetDrive() {
        rightBackMotor.resetPosition();
        rightBackMotor.resetRotation();

        rightMidMotor.resetPosition();
        rightMidMotor.resetRotation();

        rightFrontMotor.resetPosition();
        rightFrontMotor.resetRotation();

        leftBackMotor.resetPosition();
        leftBackMotor.resetRotation();

        leftMidMotor.resetPosition();
        leftMidMotor.resetRotation();

        leftFrontMotor.resetPosition();
        leftFrontMotor.resetRotation();

        IMU.resetRotation();
      }

      void setDriveBreak() {
        rightBackMotor.setBrake(coast);
        rightMidMotor.setBrake(coast);
        rightFrontMotor.setBrake(coast);
        leftBackMotor.setBrake(coast);
        leftMidMotor.setBrake(coast);
        leftFrontMotor.setBrake(coast);
      }

      /*
      get right drivetrain average pos 
      @return - avg pos of right drive
      */
      double getRightDrivePos() {
        return (rightFrontMotor.rotation(deg) + rightMidMotor.rotation(deg) + rightBackMotor.rotation(deg))/3;
      }

      /*
      get left drive avg pos
      @return - left avg pos of drive
      */
      double getLeftDrivePos() {
        return (leftFrontMotor.rotation(deg) + leftMidMotor.rotation(deg) + leftBackMotor.rotation(deg))/3;
      }

      /*
      get drive avg pos
      @return - avg drive pos
      */
      double getDrivePos() {
        return (getRightDrivePos() + getLeftDrivePos())/2;
      }

      /*
      set kp and kd value for drivetrain
      @param kp - kp value for pd loop; defult = .046
      @param kd - kd value for pd loop; defult = .0387
      */
      void setDrivePID(double kp = .2, double kd = .178) {
        drivekp = kp;
        drivekd = kd;
      }

    // Lateral Movement
      /*
      drive function - using slew and pd loops
      @param target - desired travel distance; positive = fwd, negitive = reverse 
      */
      void drivePD(int target) {
        double error = 0;
        double derivitive;
        double prevError = 0;
        double output = 0;
        double pdPower = 0;
        double currentPos = 0;

        double correctionRate = 0;
        int acceptableError = 3;

        int dir = target/abs(target);
        int imuError;

        if (drivekd == 0 || drivekp == 0) {
          setDrivePID();
        }

        resetDrive();

        while(currentPos < (target - acceptableError) || currentPos > (target + acceptableError)) {

          imuError = IMU.rotation();

          currentPos = fabs(getDrivePos());
          target = abs(target);

          error = target - currentPos;
          derivitive = error - prevError;

          pdPower = error*drivekp + derivitive*drivekd;

          if(pdPower > output) {
            output += 0.1;
          }
          else {
            output = pdPower;
          } 

          leftFrontMotor.spin(fwd, output*dir - imuError*correctionRate, volt);
          leftMidMotor.spin(fwd, output*dir - imuError*correctionRate, volt);
          leftBackMotor.spin(fwd, output*dir - imuError*correctionRate, volt);
          rightFrontMotor.spin(fwd, output*dir + imuError*correctionRate, volt);
          rightMidMotor.spin(fwd, output*dir + imuError*correctionRate, volt);
          rightBackMotor.spin(fwd, output*dir + imuError*correctionRate, volt);

          prevError = error;
          vex::task::sleep(10);
        }
        drivekd = 0;
        drivekp = 0;
        leftFrontMotor.stop(coast);
        leftBackMotor.stop(coast);
        leftMidMotor.stop(coast);
        rightFrontMotor.stop(coast);
        rightBackMotor.stop(coast);
        rightMidMotor.stop(coast);
      }

      void timeDrive(int speed, int t) {
        rightFrontMotor.spin(fwd, speed, volt);
        rightMidMotor.spin(fwd, speed, volt);
        rightBackMotor.spin(fwd, speed, volt);
        leftFrontMotor.spin(fwd, speed, volt);
        leftMidMotor.spin(fwd, speed, volt);
        leftBackMotor.spin(fwd, speed, volt);
        task::sleep(t);
        leftFrontMotor.stop();
        leftBackMotor.stop();
        leftMidMotor.stop();
        rightFrontMotor.stop();
        rightMidMotor.stop();
        rightBackMotor.stop();
      }
    // TURN MOVEMENT
      /*
      turn function - using slew and pd loops
      @param target - desired travel distance; positive = left, negitive = right 
      */
      void turnPD(int target, double minSpeed=4) {
        double kp = .237;
        double kd = .189;

        double error = 0;
        double derivitive;
        double prevError = 0;
        double output = 0;
        double pdOutput = 0;
        double currentPos = 0;

        int acceptableError = 1;

        int dir = target/abs(target);

        double slew = .1;

        resetDrive();

        while(currentPos < (target - acceptableError) || currentPos > (target + acceptableError)) {
          currentPos = IMU.rotation(degrees);
          printf("IMU: %f\n", currentPos);

          error = target - currentPos;
          derivitive = error - prevError;

          pdOutput = error*kp + derivitive*kd;

          if (pdOutput > output) {
            if(dir < 0) {
              output -= slew;
            } else if (dir > 0 ){
              output += slew;
            }
          } else {
            output = pdOutput;
          }

          if (output < minSpeed) {
            if (dir < 0) {
              output = minSpeed*-1;
            } else if (dir > 0) {
              output = minSpeed;
            }
          }

          output = fabs(output)*dir;
            
          leftFrontMotor.spin(fwd, output, volt);
          leftMidMotor.spin(fwd, output, volt);
          leftBackMotor.spin(fwd, output, volt);
          rightFrontMotor.spin(reverse, output, volt);
          rightMidMotor.spin(reverse, output, volt);
          rightBackMotor.spin(reverse, output, volt);

          prevError = error;
          vex::task::sleep(10);
        }
        leftFrontMotor.stop();
        leftMidMotor.stop();
        leftBackMotor.stop();
        rightFrontMotor.stop();
        rightMidMotor.stop();
        rightBackMotor.stop();
        IMU.setRotation(0, degrees);
      }

      /*
      move right drive till at desired angle
      */
      void rightDrive(int target, double speed) {
        rightBackMotor.spin(fwd, speed, volt);
        rightMidMotor.spin(fwd, speed, volt);
        rightFrontMotor.spin(fwd, speed, volt);
        while(IMU.rotation() > (target + 2) || IMU.rotation() < (target - 2)) {
          task::sleep(10);
        }
        rightFrontMotor.stop(coast);
        rightMidMotor.stop(coast);
        rightBackMotor.stop(coast);
      }
      void leftDrive(int target, double speed) {
        leftBackMotor.spin(fwd, speed, volt);
        leftMidMotor.spin(fwd, speed, volt);
        leftFrontMotor.spin(fwd, speed, volt);
        while(IMU.rotation() > (target + 2) || IMU.rotation() < (target - 2)) {
          task::sleep(10);
        }
        leftFrontMotor.stop(coast);
        leftMidMotor.stop(coast);
        leftBackMotor.stop(coast);
      }
///////////////   LIFT        ///////////////
  /*          Lift Vars               */
    double liftKP = .435;
    double liftKI = .08;
    double liftKD = .198;

  /*          Lift Functions          */
    // helper function
    double getLiftPos() {
      return liftMotor.rotation(degrees);
    }
    void setLiftPD(double kp, double kd) {
      liftKP = kp;
      liftKD = kd;
    }
  
    // motion functions
    /*
    function to control lift movements
    @param target - int; desired lift value
    @param downPos - bool; is desiredPos all the way down?
    */
    void liftPD(int target, bool downPos = false, bool resetMotorPos = false) {
      double error = 0;
      double derivitive;
      double prevError = 0;
      double output;
      double currentPos = 0;
      int acceptableError = 5;

      if(resetMotorPos) {
        liftMotor.resetPosition();
        liftMotor.resetRotation();
      }

      while(currentPos < (target - acceptableError) || currentPos > (target + acceptableError)) {
        currentPos = getLiftPos();

        error = target - currentPos;
        derivitive = error - prevError;

        output = error*liftKP + derivitive*liftKD;

        liftMotor.spin(fwd, output, volt);

        prevError = error;
        vex::task::sleep(10);
      }
      if(downPos) {
        liftMotor.stop(coast);
      } else {
        liftMotor.stop(hold);
      }
    }

    void startLift(double speed) {
      liftMotor.spin(fwd, speed, volt);
    }
    void stopLift(int down=1) {
      if(down == 0) {
        liftMotor.stop(hold);
      } else if (down == 1) {
        liftMotor.stop(coast);
      }
    }

///////////////   PNEUMATICS  ///////////////

  // PNEUMATIC VARS
    int tilterState = UP;

  // functions
    // tilter
      void moveTilter(int Pos) {
        switch(Pos){
          case UP: {
            tilter.open();
            tilterState = UP;
            break;
          }
          case DOWN: {
            tilter.close();
            tilterState = DOWN;
            break;
          }
        }
      }
    // claw
      void moveFrontClaw(int Pos) {
        switch(Pos) {
          case UP: {
            frontClaw.close();
            break;
          }
          case DOWN: {
            frontClaw.open();
            break;
          }
        }
      }
      void moveBackClaw(int Pos) {
        switch(Pos) {
          case UP: {
            backClaw.close();
            break;
          }
          case DOWN: {
            backClaw.open();
            break;
          }
        }
      }


///////////////   RINGS       ///////////////
  void runIntake(int dir) {
      intake.spin(fwd, 12*dir, volt);
    }
    void stopIntake() {
      intake.stop();
    }




void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  moveFrontClaw(DOWN);
  moveBackClaw(DOWN);
  moveTilter(UP);

  IMU.calibrate();
  while(IMU.isCalibrating()) {
    task::sleep(10);
  }
  setDriveBreak();
}

void autonomous(void) {
  setDriveBreak();
  moveFrontClaw(UP);
  moveTilter(DOWN);
  moveBackClaw(UP);
  wait(100, msec);

  timeDrive(12, 800);
  moveFrontClaw(DOWN);
  wait(300, msec);
  timeDrive(12, 650);
  liftPD(100);
  turnPD(-90); // 90 degree
  drivePD(-100);
  timeDrive(-5, 300);
  moveBackClaw(DOWN);
  moveTilter(UP);
  drivePD(200);
  runIntake(1);
}

double driverRightOutput = 0;
double driverLeftOutput = 0;
double maxSpeed = 12;

void usercontrol(void) {
  setDriveBreak();
  moveFrontClaw(UP);
  moveTilter(DOWN);
  moveBackClaw(UP);
  while (1) {
    //DRIVE
    double leftJoystickY = Master.Axis3.value();   // figure out what values these return
    double rightJoystickY = Master.Axis2.value();

    if(fabs(leftJoystickY) < 5){
      leftJoystickY = 0;
    }
    if(fabs(rightJoystickY) < 5){
      rightJoystickY = 0;
    }

    if (rightJoystickY > maxSpeed) {
      rightJoystickY = maxSpeed;
    } else if (rightJoystickY < -maxSpeed) {
      rightJoystickY = - maxSpeed;
    }

    if (leftJoystickY > maxSpeed) {
      leftJoystickY = maxSpeed;
    } else if (leftJoystickY < -maxSpeed) {
      leftJoystickY = - maxSpeed;
    }

    if (Master.ButtonLeft.pressing()) {
      maxSpeed = 6;
    } else if (Master.ButtonRight.pressing()) {
      maxSpeed = 12;
    }

    rightFrontMotor.spin(fwd, rightJoystickY, volt);
    rightMidMotor.spin(fwd, rightJoystickY, volt);
    rightBackMotor.spin(fwd, rightJoystickY, volt);
    leftFrontMotor.spin(fwd, leftJoystickY, volt);
    leftMidMotor.spin(fwd, leftJoystickY, volt);
    leftBackMotor.spin(fwd, leftJoystickY, volt);

    //FRONT LIFT
    if(Master.ButtonR1.pressing()) {
      liftMotor.spin(fwd, 12, volt);
    } else if(Master.ButtonR2.pressing()){
      liftMotor.spin(reverse, 12, volt);
    } else {
      liftMotor.stop(hold);
    }

    // TILTER
    if(Master.ButtonL1.pressing()) { // when button is pressed, grab mobile goal and then tilt back
      moveBackClaw(DOWN); // make sure if tilter is pulled back claw is grabbed
      task::sleep(300);
      moveTilter(UP);
    } else if (Master.ButtonL2.pressing()) {
      moveTilter(DOWN);
      moveBackClaw(UP); // when tilter is pushed down drop claw
    }

    // FRONT CLAW
    if(Master.ButtonX.pressing()) {
      moveFrontClaw(DOWN);
    } else if (Master.ButtonB.pressing()) {
      moveFrontClaw(UP);
    }

    // BACK CLAW
    if(Master.ButtonUp.pressing()) {
      moveBackClaw(DOWN);
    } else if (Master.ButtonDown.pressing()) {
      moveBackClaw(UP);
    }

    // INTAKE OVERRIDE
    if(Master.ButtonA.pressing()) {
      intake.spin(reverse, 12, volt);
    } else {
      if (liftMotor.rotation(deg) > 50) {
        intake.spin(fwd, 12, volt);
      } else {
        intake.stop();
      }
    }
    
    wait(20, msec);
  }
}

int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
