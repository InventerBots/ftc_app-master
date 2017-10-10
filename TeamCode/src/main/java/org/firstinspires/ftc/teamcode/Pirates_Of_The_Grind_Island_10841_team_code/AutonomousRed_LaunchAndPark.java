package org.firstinspires.ftc.teamcode.Pirates_Of_The_Grind_Island_10841_team_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Luke on 1/15/2017.
 */

/*
 * This Autonomous program starts four tiles over from the corner vortex wall with the intake to the
 * wall. The left corner of the robot must be against the wall, just ot the right of the tile separation
 * line.
 * It starts out by driving 12" forward and turning around to launch two balls. Than it turns back to
 * face the center vortex base and drives 30" hitting the cap NumberOfBallsOnRamp, and backing up 4" turn 54 degrees
 * and finely drive 11" to park fully on the center vortex base.
 *
 * This program will ern us 45 points.
*/
@Autonomous (name = "Red PARK", group = "10841 red")
//@Disabled
public class AutonomousRed_LaunchAndPark extends LinearOpMode {
    Team_10841_Robot robot = new Team_10841_Robot();
    ElapsedTime runtime = new ElapsedTime();

    double turnSpeed = 0.4;
    double driveSpeed = 0.7;
    double secs = 0;
    boolean stalled;
    int stalledTest;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addLine("Robot Initializing ... Please do not touch the robot.");
        telemetry.update();
        robot.init(hardwareMap);
        robot.SetAutonomousMode(true);
        telemetry.addLine("Robot is ready to start");
        telemetry.update();
        robot.ods_Line.enableLed(false);
        robot.ods_Wall.enableLed(false);
        robot.left_color.enableLed(false);
        robot.right_color.enableLed(false);

        waitForStart();

        runtime.reset();  // reset and start a timer
        showData();

        if (opModeIsActive()){
            driveForDistanceInches(12, driveSpeed); // drive 60 inches
            showData();
        }
        if (opModeIsActive()){
            // turn 90 degrees
            swingTurnRight(70, turnSpeed); // turn to face Vortex
            showData();
        }
        if (opModeIsActive()) {
            LaunchParticle(true); // launch two balls
            showData();
        }
        if (opModeIsActive()) {
            LaunchParticle(false);
            showData();
        }

        if (opModeIsActive()) {
            swingTurnRight(-70, -turnSpeed);
        }
        runtime.reset(); // about 11 seconds to here
        while (opModeIsActive() && runtime.seconds()<12) {
            showData();
        }

        if (opModeIsActive()) {
            driveForDistanceInches(30, driveSpeed);
        }

        if (opModeIsActive()) {
            driveForDistanceInches(-4, -driveSpeed);
        }

        if (opModeIsActive()) {
            swingTurnLeft(45, turnSpeed);
            showData();
        }
        // park
        if (opModeIsActive()) {
            driveForDistanceInches(11, driveSpeed/2);
            showData();
        }


        telemetry.addLine("full park complete");
        telemetry.update();
    }

    void showData(){
        telemetry.addData("run time = ", runtime.seconds());
        telemetry.addData("left drive motor speed = ", robot.leftDriveMotor);
        telemetry.addData("right drive motor speed = ", robot.rightDriveMotor);
        telemetry.update();

    }
    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

   Common code to all red and blue autonomous options

   * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    void driveToBeacon(){
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ods_Wall.enableLed(true);

        runtime.reset();
        stalled = false;
        secs =  runtime.seconds();
        int stalledTestRight = robot.rightDriveMotor.getCurrentPosition();
        int stalledTestLeft = robot.leftDriveMotor.getCurrentPosition();

        while ((opModeIsActive() && robot.ods_Wall.getRawLightDetected() < 0.28) && !stalled) {
            if (robot.ods_Wall.getRawLightDetected() < 0.0001){
                robot.leftDriveMotor.setPower(0.2);
                robot.rightDriveMotor.setPower(0.2);
            }else {
                robot.leftDriveMotor.setPower(0.1);
                robot.rightDriveMotor.setPower(0.1);
            }
            telemetry.addData("Stalled ",stalled ? "Robot is Stalled!" : "Normal operation");
            telemetry.addData("driveToBeacon value ", robot.ods_Wall.getRawLightDetected());
            telemetry.addData("driveToBeacon Power ", robot.rightDriveMotor.getPower());
            telemetry.update();

            if ((runtime.seconds() - secs) > 1.0) {
                stalled = (stalledTestRight == robot.rightDriveMotor.getCurrentPosition()) ||
                        (stalledTestLeft == robot.leftDriveMotor.getCurrentPosition());
                stalledTestRight = robot.rightDriveMotor.getCurrentPosition();
                stalledTestLeft = robot.leftDriveMotor.getCurrentPosition();
                secs =  runtime.seconds();
            }

        }
        robot.leftDriveMotor.setPower(0.0);
        robot.rightDriveMotor.setPower(0.0);
        robot.ods_Wall.enableLed(false);

    }

    void stopDrive() {
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveMotor.setPower(0.0);
        robot.rightDriveMotor.setPower(0.0);
    }

    void LaunchParticle(boolean loadball) throws InterruptedException {
        robot.StartCatapultCycle();
        // need to wait until the catapult is no longer pressing the touch sensor
        // Should only be a small fraction of a second
        while (opModeIsActive() && robot.catapultTouchSensor.isPressed()) {
            idle();
        }
        // now wait until the catapult presses the touch sensor to move on
        while (opModeIsActive() && !robot.catapultTouchSensor.isPressed()) {
            idle();
        }
        robot.EndCatapultCycle(loadball);
    }

    //-------------------- Press the red beacon button --------------------//
    boolean areBothSidesOfBeaconRed() {
        /*
        We need to check both color sensors to see if they are both red.  If not, maybe we can come
        back and press the button again.
         */
//        boolean ok = robot.leftBeaconButtonIsRed() && robot.rightBeaconButtonIsRed();
//        if (robot.leftBeaconButtonIsRed() && robot.rightBeaconButtonIsRed()) {
//            ok = true; // means that beacon is solid red
//        } else ok = false; // means that beacon is not solid red
        return robot.leftBeaconButtonIsRed() && robot.rightBeaconButtonIsRed();
    }

    private boolean TheRightSideIsRed() {
        /*
        Check the color sensors to determine if the Right side of the beacon is red.  If it is
        return true;
        */
        if (robot.rightBeaconButtonIsRed()) {
            return true; // if the right side of the beacon is red return true
        } else return false; // other wise return false
    }

    boolean areBothSidesOfBeaconBlue() {
        /*
        We need to check both color sensors to see if they are both red.  If not, maybe we can come
        back and press the button again.
         */
        if (robot.leftBeaconButtonIsBlue() && robot.rightBeaconButtonIsBlue()) {
            return true; // means that beacon is solid red
        } else return false; // means that beacon is not solid red
    }

    /*
     * drives the robot straight until it finds a white line on the floor
      */
    void LocateWhiteLine(double DistanceToDrive,String redORblue) throws InterruptedException {
        int maxTicksToDrive = robot.leftDriveMotor.getCurrentPosition() + robot.setTargetInches(DistanceToDrive);
        boolean found;
        boolean TooFar;
        do {
            robot.setDriveWheelSpeed(turnSpeed, turnSpeed);
            robot.odsValue = robot.ods_Line.getRawLightDetected();
            found = robot.odsValue > 0.5;
            TooFar = robot.leftDriveMotor.getCurrentPosition() > maxTicksToDrive;

            telemetry.addData("Locate White Line raw ",robot.odsValue);
            int curpos = robot.leftDriveMotor.getCurrentPosition();
            telemetry.addData("Cur pos ",curpos);
            telemetry.addData("Target ",maxTicksToDrive);
            telemetry.addData("found ",found);
            telemetry.addData("TooFar ",TooFar);

            telemetry.update();
        }  while (opModeIsActive() && !found && !TooFar);

        if (TooFar) {
            // need to back up and try again.
            if (redORblue == "red") {
                swingTurnLeft(-10, -turnSpeed);
                driveForDistanceInches(-DistanceToDrive, -turnSpeed);
                swingTurnRight(-10, -turnSpeed);
            } else {
                swingTurnRight(-10,-turnSpeed);
                driveForDistanceInches(-DistanceToDrive,-turnSpeed);
                swingTurnLeft(-10,-turnSpeed);
            }
            if (opModeIsActive())
                LocateWhiteLine(DistanceToDrive,redORblue); // try it again
        }
    }

    void driveForDistanceCM(double Centimeters, double setSpeed) throws InterruptedException {
        driveForDistanceInches(Centimeters / 2.54, setSpeed);  // convert centimeters to inches 2.54 cm = 1 inch
    }

    /*
     * Drive the robot in a somewhat straight line for "driveToDistance" inches.  It converts inches
     * to encoder ticks to determine where to stop.
     */
    void driveForDistanceInches(double driveToDistance, double setSpeed) throws InterruptedException {
        int startPosition;
        if (driveToDistance < 0)
            setSpeed = -Math.abs(setSpeed);
        else
            setSpeed = Math.abs(setSpeed);
        robot.ticksToMove = -robot.setTargetInches(driveToDistance);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int Target_Right = robot.rightDriveMotor.getCurrentPosition() - robot.ticksToMove;
        int Target_Left = robot.leftDriveMotor.getCurrentPosition() - robot.ticksToMove;
        startPosition = robot.leftDriveMotor.getCurrentPosition();

//        robot.leftDriveMotor.setTargetPosition(robot.leftDriveMotor.getCurrentPosition() - robot.ticksToMove);
//        robot.rightDriveMotor.setTargetPosition(robot.rightDriveMotor.getCurrentPosition() - robot.ticksToMove);

//        runtime.reset();
//        stalled = false;
//        secs =  runtime.seconds();
//        int stalledTestRight = robot.rightDriveMotor.getCurrentPosition();
//        int stalledTestLeft = robot.leftDriveMotor.getCurrentPosition();

        if (driveToDistance < 0) {
            while (opModeIsActive() && ((robot.leftDriveMotor.getCurrentPosition() > Target_Left) || (robot.rightDriveMotor.getCurrentPosition() > Target_Right) && !stalled)) {
                robot.drivePower = -setDrivePower(startPosition, robot.leftDriveMotor.getCurrentPosition(),
                        Target_Left, robot.slowZone, setSpeed);
                robot.leftDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
                robot.rightDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));

                telemetry.addData("< setspeed",setSpeed);
                telemetry.addData("left pos",robot.leftDriveMotor.getCurrentPosition());
                telemetry.addData("target ",Target_Left);
                telemetry.update();

            }
        } else {
            while (opModeIsActive() && ((robot.leftDriveMotor.getCurrentPosition() < Target_Left) || (robot.rightDriveMotor.getCurrentPosition() < Target_Right) && !stalled)) {
                robot.drivePower = setDrivePower(startPosition, robot.leftDriveMotor.getCurrentPosition(),
                        Target_Left, robot.slowZone, setSpeed);


                robot.leftDriveMotor.setPower(Range.clip(robot.drivePower, -1.0, 1.0));
                robot.rightDriveMotor.setPower(Range.clip(robot.drivePower, -1.0, 1.0));
                telemetry.addData("> setspeed",setSpeed);
                telemetry.addData("left pos",robot.leftDriveMotor.getCurrentPosition());
                telemetry.addData("target ",Target_Left);
                telemetry.update();

            }
        }

        robot.rightDriveMotor.setPower(0);
        robot.leftDriveMotor.setPower(0);
        sleep(100);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double setDrivePower(int startPos, int currentPos, int targetPos, int slowDistance, double targetMaxPower) {
        /*
        We want to ramp up and down the power at the beginning and end of the run.
         */
        double minSpeed = 0.1;

        double drivePower;  // this value will be returned
        int totalTravelDistance = (Math.abs(targetPos - startPos));

        // Decrease the max speed if the ramp up and ramp down distances are shorter then
        // the total travel distance. Keep it on the same vector.
        double maxSpeed;// = robot.calcMaxSpeed(slowDistance, targetPos - startPos, minSpeed, targetMaxPower);
        maxSpeed = targetMaxPower; // for now ignore the value we just calculated.  delete this later

        boolean neg = false;
        if (targetMaxPower < 0) {
            neg = true;
            maxSpeed = -targetMaxPower;
        }

        int DistanceRemaining = Math.abs(targetPos - currentPos);
        int DistanceTraveled = Math.abs(currentPos - startPos);
        double rUp = robot.RampUp(DistanceTraveled, slowDistance/2, minSpeed*2, maxSpeed);
        double rDn = robot.RampDown(DistanceRemaining, slowDistance*3, minSpeed, maxSpeed);

        /*
        if the drive power is neg we need to set it to the max instead of min.
         */
        if (rUp < 0)
            drivePower = Math.max(rUp,rDn);
        else
            drivePower = Math.min(rUp,rDn);

        if (targetPos < currentPos)
            drivePower = -drivePower;

        if (neg)
            drivePower = -drivePower;

        return drivePower;
    }

    /*
     * Use encoder ticks to cause the robot to pivot at the center of the robot. It over shoots the
     * target angle since the robot coasts after the motors are turned off.
     */
    void PointTurnUsingEncoders(int TurnAngle, double setSpeed) throws InterruptedException {
        int TicksToTurn = (int) robot.wheel_angleToTicks(TurnAngle);

        int leftM_target = robot.leftDriveMotor.getCurrentPosition() + TicksToTurn;
        int rightM_target = robot.rightDriveMotor.getCurrentPosition() - TicksToTurn;
        int startPosition = robot.leftDriveMotor.getCurrentPosition();
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDriveMotor.setTargetPosition(leftM_target);
        robot.rightDriveMotor.setTargetPosition(rightM_target);

        boolean done = false;
        while (opModeIsActive() && robot.rightDriveMotor.isBusy() && robot.leftDriveMotor.isBusy()) {
            robot.drivePower = setDrivePower(startPosition, robot.leftDriveMotor.getCurrentPosition(),
                    robot.leftDriveMotor.getTargetPosition(), robot.setTargetInches(3), setSpeed);
            robot.leftDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
            robot.rightDriveMotor.setPower(-Range.clip(robot.drivePower,-1.0,1.0));
        }
        //sleep(100);
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


//    void driveStraightUsingGyros(double Inches, double speed, int directionAdj) {
//        double leftSpeed; //Power to feed the motors
//        double rightSpeed;
//        double StartSpeed = speed;
//        double ClipSpeed = Range.clip(Math.abs(speed * 1.2), -1.0, 1.0);
//        int StartPos = robot.leftDriveMotor.getCurrentPosition();
//        double encoderTarget = robot.setTargetInches(Inches);
//
//        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        double startz = robot.gyro.getIntegratedZValue();
//        double target = (robot.gyro.getIntegratedZValue() + directionAdj);  //Starting direction
//        double startPosition = robot.leftDriveMotor.getCurrentPosition();  //Starting position
//
//        double errorCorrection;
//
//        while (opModeIsActive() && robot.leftDriveMotor.getCurrentPosition() < startPosition + encoderTarget) {  //While we have not passed out intended distance
//            robot.zAccumulated = robot.gyro.getIntegratedZValue();  //Current direction
//     //            speed = setDrivePower(StartPos,robot.leftDriveMotor.getCurrentPosition(),encoderTarget,robot.setTargetInches(9),StartSpeed);

//
//            errorCorrection = Range.clip(((robot.zAccumulated - target) / 150), -0.2, 0.2); // max correction -
//
//            leftSpeed = Range.clip((speed + errorCorrection),-ClipSpeed,ClipSpeed);  //Calculate speed for each side
//            rightSpeed = Range.clip((speed - errorCorrection),-ClipSpeed,ClipSpeed);  //See Gyro Straight video for detailed explanation
//
//            robot.leftDriveMotor.setPower(leftSpeed);
//            robot.rightDriveMotor.setPower(rightSpeed);
//        }
//    }

    /*
     * Use encoder ticks to cause the robot to pivot on one wheel of the robot. It over shoots the
     * target angle less then pivoting on the center  of the robot since only one wheel is moving
     * twice the distance.
     * This will work driving forward or reverse by setting the speed positive(forward) or
     * negative(backward).
     *
     * swingTurnRight - Right Wheel is moving and left is stationary
     * swingTurnLeft - Left Wheel is moving and Right is stationary
     */

    void swingTurnRight(int TurnAngle,double setSpeed){
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int TicksToTurn = (int) Swing_angleToTicks(Math.abs(TurnAngle));

        int leftM_target = robot.leftDriveMotor.getCurrentPosition();
        int rightM_target;
        rightM_target = robot.rightDriveMotor.getCurrentPosition() - TicksToTurn;
        if (setSpeed > 0)
            rightM_target = robot.rightDriveMotor.getCurrentPosition() + TicksToTurn;
        else
            rightM_target = robot.rightDriveMotor.getCurrentPosition() - TicksToTurn;
        int startPosition = robot.rightDriveMotor.getCurrentPosition();

        robot.leftDriveMotor.setTargetPosition(leftM_target);
        robot.rightDriveMotor.setTargetPosition(rightM_target);
        robot.leftDriveMotor.setPower(1);

        runtime.reset();
        stalled = false;
        secs = runtime.seconds();
        stalledTest = robot.rightDriveMotor.getCurrentPosition();

        // this maybe should be changed to checking if we have gone past the target.
        while (opModeIsActive() && robot.rightDriveMotor.isBusy() && !stalled) {
            robot.drivePower = setDrivePower(startPosition, robot.rightDriveMotor.getCurrentPosition(),
                    robot.rightDriveMotor.getTargetPosition(), robot.setTargetInches(3), setSpeed); // ...Inches(3) is slow distance in ticks
            robot.rightDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
            if ((runtime.seconds() - secs) > 1.0) {
                stalled = stalledTest == robot.rightDriveMotor.getCurrentPosition();
                stalledTest = robot.rightDriveMotor.getCurrentPosition();
                secs = runtime.seconds();
            }

        }
        stopDrive();

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    void swingTurnLeft(int TurnAngle,double setSpeed){
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int TicksToTurn = (int) Swing_angleToTicks(Math.abs(TurnAngle));

        int rightM_target = robot.rightDriveMotor.getCurrentPosition();
        int leftM_target;
        int startPosition = robot.leftDriveMotor.getCurrentPosition();
        if (setSpeed > 0)
            leftM_target = startPosition + TicksToTurn;
        else
            leftM_target = startPosition - TicksToTurn;

        robot.leftDriveMotor.setTargetPosition(leftM_target);
        robot.rightDriveMotor.setTargetPosition(rightM_target);
        robot.rightDriveMotor.setPower(1);

        runtime.reset();
        stalled = false;
        secs = runtime.seconds();
        stalledTest = robot.leftDriveMotor.getCurrentPosition();

        do
        {
            robot.drivePower = setDrivePower(startPosition, robot.leftDriveMotor.getCurrentPosition(),
                    robot.leftDriveMotor.getTargetPosition(), robot.setTargetInches(3), setSpeed);  // ...Inches(3) is slow distance in ticks
            robot.leftDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
            if ((runtime.seconds() - secs) > 1.0) {
                stalled = stalledTest == robot.leftDriveMotor.getCurrentPosition();
                stalledTest = robot.leftDriveMotor.getCurrentPosition();
                secs = runtime.seconds();
            }
        } while (opModeIsActive() && robot.leftDriveMotor.isBusy()  && !stalled);

        stopDrive();

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    private void pressTheButton(String color) throws InterruptedException {
        /*
        Now we need to pivot the robot to the left or right depending on which side is red
         */
        boolean pushRightButton;
        boolean getRed = color == "red"; // are we pressing a red button?
        if (getRed) {
            pushRightButton = robot.rightBeaconButtonIsRed();
        } else {
            pushRightButton = robot.rightBeaconButtonIsBlue();
        }

        int TurnAngle = 10;
        telemetry.addData("Color ",getRed);
        telemetry.addData("pushRightButton ",pushRightButton);
        telemetry.update();

        if (pushRightButton) {
            swingTurnLeftButtonCheck(TurnAngle, 0.2); // turn right 7 degrees
        } else if (!pushRightButton){
            swingTurnRightButtonCheck(TurnAngle, 0.2); // turn left 7 degrees
        } else {} // do nothing

    }


    void swingTurnLeftButtonCheck(int TurnAngle,double setSpeed){
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int TicksToTurn = (int) Swing_angleToTicks(Math.abs(TurnAngle));
        boolean ckColor = robot.leftBeaconButtonIsRed();
        boolean cont;

        telemetry.addData("swingTurnLeft - left is red",robot.leftBeaconButtonIsRed());
        telemetry.addData("swingTurnLeft - Right is red",robot.rightBeaconButtonIsRed());
        telemetry.update();

        int rightM_target = robot.rightDriveMotor.getCurrentPosition();
        int leftM_target;
        int startPosition = robot.leftDriveMotor.getCurrentPosition();
        if (setSpeed > 0)
            leftM_target = startPosition + TicksToTurn;
        else
            leftM_target = startPosition - TicksToTurn;

        robot.leftDriveMotor.setTargetPosition(leftM_target);
        robot.rightDriveMotor.setTargetPosition(rightM_target);
        robot.rightDriveMotor.setPower(1);

        do
        {
            robot.drivePower = setDrivePower(startPosition, robot.leftDriveMotor.getCurrentPosition(),
                    robot.leftDriveMotor.getTargetPosition(), robot.setTargetInches(3), setSpeed);  // ...Inches(3) is slow distance in ticks
            robot.leftDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
            telemetry.addData("swingTurnLeft - left is red",robot.leftBeaconButtonIsRed());
            telemetry.addData("swingTurnLeft - Right is red",robot.rightBeaconButtonIsRed());
            telemetry.update();
            cont = ckColor == robot.leftBeaconButtonIsRed();
            if (!cont) {
                robot.rightDriveMotor.setTargetPosition(robot.rightDriveMotor.getCurrentPosition());
            }
        } while (opModeIsActive() && robot.leftDriveMotor.isBusy() && (cont));


        robot.leftDriveMotor.setTargetPosition(startPosition);
        do
        {
            robot.drivePower = setDrivePower(startPosition, robot.leftDriveMotor.getCurrentPosition(),
                    robot.leftDriveMotor.getTargetPosition(), robot.setTargetInches(3), -setSpeed);  // ...Inches(3) is slow distance in ticks
            robot.leftDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
        } while (opModeIsActive() && robot.leftDriveMotor.isBusy() );

        //sleep(100);
        stopDrive();

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    void swingTurnRightButtonCheck(int TurnAngle,double setSpeed){
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int TicksToTurn = (int) Swing_angleToTicks(Math.abs(TurnAngle));
        boolean ckColor = robot.rightBeaconButtonIsRed();
        boolean cont;

        int leftM_target = robot.leftDriveMotor.getCurrentPosition();
        int rightM_target;
        int startPosition = robot.rightDriveMotor.getCurrentPosition();
        if (setSpeed > 0)
            rightM_target = startPosition + TicksToTurn;
        else
            rightM_target = startPosition - TicksToTurn;

        robot.leftDriveMotor.setTargetPosition(leftM_target);
        robot.rightDriveMotor.setTargetPosition(rightM_target);
        robot.leftDriveMotor.setPower(1);

        do {
            robot.drivePower = setDrivePower(startPosition, robot.rightDriveMotor.getCurrentPosition(),
                    robot.rightDriveMotor.getTargetPosition(), robot.setTargetInches(3), setSpeed); // ...Inches(3) is slow distance in ticks
            robot.rightDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));

            telemetry.addData("swingTurnRight - left is red",robot.leftBeaconButtonIsRed());
            telemetry.addData("swingTurnRight - Right is red",robot.rightBeaconButtonIsRed());
            telemetry.update();
            cont = ckColor == robot.rightBeaconButtonIsRed();
            if (!cont) {
                robot.rightDriveMotor.setTargetPosition(robot.rightDriveMotor.getCurrentPosition());
            }
        } while (opModeIsActive() && robot.rightDriveMotor.isBusy() && (cont));
        //sleep(100);

        // it needs to backup to the starting position
        robot.rightDriveMotor.setTargetPosition(startPosition);
        do {
            robot.drivePower = setDrivePower(startPosition, robot.rightDriveMotor.getCurrentPosition(),
                    robot.rightDriveMotor.getTargetPosition(), robot.setTargetInches(3), -setSpeed); // ...Inches(3) is slow distance in ticks
            robot.rightDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
        } while (opModeIsActive() && robot.rightDriveMotor.isBusy() );

        robot.rightDriveMotor.setPower(robot.stop);
        robot.leftDriveMotor.setPower(robot.stop);

        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    double Swing_angleToTicks(double turnAngle) {
        /*
         * return number of encoder ticks per degrees turnAngle
         * set only one wheel to this value, set the other to maintain it's current position
         * 14.45 is the radius of the arc the wheel will travel and 3 is the diameter of
         * the wheels.
         */
        double WheelDiameter = 3;
        double robotDiameter = 14.45*2; // with one wheel turning we need to double the diameter
        double robotCircumference = (robotDiameter * Math.PI);
        double wheelCircumference = (WheelDiameter * Math.PI);
        // return (robot Circumference / Wheel circumference) / 360 = distance travel per degree
        // then multiply it by the turn angle * Ticks per wheel revolution = motor ticks
        return Math.round(((robotCircumference / wheelCircumference / 360) * turnAngle) * robot.driveWheelTicksPerRevolution);
    }


    void SwingTurnUsingEncoders(int TurnAngle, double SetSpeed) throws InterruptedException{
        int TicksToTurn = (int) robot.wheel_angleToTicks(TurnAngle);

        int LeftM_Target = robot.leftDriveMotor.getCurrentPosition() - TicksToTurn;
        int rightM_Target = robot.leftDriveMotor.getCurrentPosition() + TicksToTurn;
        int StartPosition_left = robot.leftDriveMotor.getCurrentPosition();
        int StartPosition_right = robot.rightDriveMotor.getCurrentPosition();
        int StartPosition;
        int CurrentOps;
        int TargetOps;
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftDriveMotor.setTargetPosition(LeftM_Target);
        robot.rightDriveMotor.setTargetPosition(rightM_Target);

        if (TurnAngle > 0) {
            StartPosition = StartPosition_left;
            CurrentOps = robot.leftDriveMotor.getCurrentPosition();
            TargetOps = robot.leftDriveMotor.getTargetPosition();

        } else {
            StartPosition = StartPosition_right;
            CurrentOps = robot.rightDriveMotor.getCurrentPosition();
            TargetOps = robot.rightDriveMotor.getTargetPosition();

        }

        boolean done = false;
        while(opModeIsActive() && robot.rightDriveMotor.isBusy() && robot.leftDriveMotor.isBusy()) {
            robot.drivePower = setDrivePower(StartPosition, CurrentOps, TargetOps, robot.slowZone, (SetSpeed*2));
            if (TurnAngle > 0)
                robot.leftDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
            else
                robot.rightDriveMotor.setPower(-Range.clip(robot.drivePower,-1.0,1.0));
        }
        sleep(100);

        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /*
     * Use encoder ticks to cause the robot to pivot at the center of the robot. It over shoots the
     * target angle since the robot coasts after the motors are turned off.
     */



}
