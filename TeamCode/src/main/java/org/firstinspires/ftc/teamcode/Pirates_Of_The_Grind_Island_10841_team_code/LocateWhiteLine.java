package org.firstinspires.ftc.teamcode.Pirates_Of_The_Grind_Island_10841_team_code;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Luke on 12/4/2016.
 */
@Autonomous(name = "Locate White Line", group = "testing")
@Disabled
public class LocateWhiteLine  extends LinearOpMode {
    Team_10841_Robot robot =new Team_10841_Robot();

    double driveSpeed = 0.7;
    double turnSpeed = 0.2;

    double Floor_odsRawValue;
    double Wall_odsRawValue;
    static double odsReading;

    boolean DriveToBeacon_Complete = false;
    boolean PressBeacon_Complete = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        robot.SetAutonomousMode(true);

        waitForStart();

        while (opModeIsActive()) {
            LocateWhiteLine(5, "red");

//            robot.left_color.enableLed(true);
//            robot.right_color.enableLed(true);
//            telemetry.addData("ODS distance from wall raw reading", "%4.4f", robot.ods_Wall.getRawLightDetected())
//                    .addData("ODS line raw reading", "%4.4f", robot.ods_Wall.getRawLightDetected())
//                    .addData("left color", robot.left_color)
//                    .addData("right color", robot.right_color);
//            telemetry.update();

            //********** start of "drive ot beacon" *********//
//            if (driveToBeacon() && !PressBeacon_Complete && DriveToBeacon_Complete) { // if driveToBeacon has not run...
//                PressBeacon_Complete = false;
//                DriveToBeacon_Complete = true;
      //          driveToBeacon(); // run driveToBeacon
//            } //else DriveToBeacon_Complete = true;
            //********** end of "drive to beacon" start of "press beacon" **********//
//            if (pressBeacon()&& !PressBeacon_Complete && DriveToBeacon_Complete) { // if pressBeacon has not run...
//                PressBeacon_Complete = true;
//                DriveToBeacon_Complete = false;
//                pressBeacon(); // run pressBeacon
//            } //else DriveToBeacon_Complete = false;
            //********** end of "press beacon" start of "line following" **********//

//            telemetry.addData("ods Reading", robot.ods_Line);
//            telemetry.update();

//            lineTracking();
//            if (/*driveToBeacon() &&*/ pressBeacon());
//                else{ sleep(5000); break;}
//
//            waitOneFullHardwareCycle();
        }
    }

    void driveForDistanceInches(double driveToDistance, double setSpeed) throws InterruptedException {
        int startPosition;
        robot.ticksToMove = -robot.setTargetInches(driveToDistance);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        startPosition = robot.leftDriveMotor.getCurrentPosition();

        robot.leftDriveMotor.setTargetPosition(robot.leftDriveMotor.getCurrentPosition() - robot.ticksToMove);
        robot.rightDriveMotor.setTargetPosition(robot.rightDriveMotor.getCurrentPosition() - robot.ticksToMove);

        while (opModeIsActive() && robot.rightDriveMotor.isBusy() && robot.leftDriveMotor.isBusy()) {
            robot.drivePower = setDrivePower(startPosition, robot.leftDriveMotor.getCurrentPosition(),
                    robot.leftDriveMotor.getTargetPosition(), robot.slowZone, setSpeed);

            robot.leftDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
            robot.rightDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
        }
        sleep(100);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private double setDrivePower(int startPos, int currentPos, int targetPos, int slowDistance, double targetMaxPower) {
        /*
        We want to ramp up and down the power at the beginning and end of the run.
         */
        double minSpeed = 0.05;

        double drivePower;  // this value will be returned
        int totalTravelDistance = (Math.abs(targetPos - startPos));

        // Decrease the max speed if the ramp up and ramp down distances are shorter then
        // the total travel distance. Keep it on the same vector.
        double maxSpeed;// = robot.calcMaxSpeed(slowDistance, targetPos - startPos, minSpeed, targetMaxPower);
        maxSpeed = targetMaxPower; // for now ignore the value we just calculated.  delete this later

        int DistanceRemaining = Math.abs(targetPos - currentPos);
        int DistanceTraveled = Math.abs(currentPos - startPos);
        double rUp = robot.RampUp(DistanceTraveled, slowDistance, minSpeed*2, maxSpeed);
        double rDn = robot.RampDown(DistanceRemaining, slowDistance, minSpeed/2, maxSpeed);

        /*
        if the drive power is neg we need to set it to the max instead of min.
         */
        if (rUp < 0)
            drivePower = Math.max(rUp,rDn);
        else
            drivePower = Math.min(rUp,rDn);

//        drivePower = (robot.RampUp(DistanceTraveled, slowDistance, minSpeed, maxSpeed) +
//                robot.RampDown(DistanceRemaining, slowDistance, minSpeed, maxSpeed)) / 2;

        if (targetPos < currentPos)
            drivePower = -drivePower;

        return drivePower;
    }

    void stopDrive() {
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveMotor.setPower(0.0);
        robot.rightDriveMotor.setPower(0.0);
    }

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

        // this maybe should be changed to checking if we have gone past the target.
        while (opModeIsActive() && robot.rightDriveMotor.isBusy()) {
            robot.drivePower = setDrivePower(startPosition, robot.rightDriveMotor.getCurrentPosition(),
                    robot.rightDriveMotor.getTargetPosition(), robot.setTargetInches(3), setSpeed); // ...Inches(3) is slow distance in ticks
            robot.rightDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
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

        do
        {
            robot.drivePower = setDrivePower(startPosition, robot.leftDriveMotor.getCurrentPosition(),
                    robot.leftDriveMotor.getTargetPosition(), robot.setTargetInches(3), setSpeed);  // ...Inches(3) is slow distance in ticks
            robot.leftDriveMotor.setPower(Range.clip(robot.drivePower,-1.0,1.0));
        } while (opModeIsActive() && robot.leftDriveMotor.isBusy());

        stopDrive();

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

    void LocateWhiteLine(double DistanceToDrive,String redORblue) throws InterruptedException {
        int maxTicksToDrive = robot.leftDriveMotor.getCurrentPosition() + robot.setTargetInches(DistanceToDrive);
        boolean found;
        boolean TooFar;
        do {
            robot.setDriveWheelSpeed(0.1, 0.1);
            robot.odsValue = robot.ods_Line.getRawLightDetected();
            telemetry.addData("Locate White Line raw ",robot.odsValue);
            int curpos = robot.leftDriveMotor.getCurrentPosition();
            telemetry.addData("Cur pos ",curpos);
            telemetry.addData("Target ",maxTicksToDrive);
            telemetry.update();
            found = robot.odsValue > 0.5;
            TooFar = robot.leftDriveMotor.getCurrentPosition() > maxTicksToDrive;
        }  while (opModeIsActive() && !found && !TooFar);

        if (TooFar) {
            // need to back up and try again.
            if (redORblue == "red") {
                swingTurnLeft(-5, -turnSpeed);
                driveForDistanceInches(-DistanceToDrive, -driveSpeed);
                swingTurnRight(-5, -turnSpeed);
            } else {
                swingTurnRight(-5,-turnSpeed);
                driveForDistanceInches(-DistanceToDrive,-driveSpeed);
                swingTurnLeft(-5,-turnSpeed);
            }
            if (opModeIsActive())
                LocateWhiteLine(DistanceToDrive,redORblue); // try it again
        }
    }

    void driveToBeacon(){
        if (robot.ods_Wall.getRawLightDetected() < 0.5) {
            robot.leftDriveMotor.setPower(-0.1);
            robot.rightDriveMotor.setPower(-0.1);
        }else {
            robot.leftDriveMotor.setPower(0.0);
            robot.rightDriveMotor.setPower(0.0);
        }
    }

    boolean pressBeacon(){
        if (robot.left_color.blue() > robot.left_color.red() && robot.left_color.blue() > robot.left_color.green()) {
            robot.leftDriveMotor.setPower(-0.1);
            robot.rightDriveMotor.setPower(0.0);
        }else  if (robot.left_color.red() > robot.left_color.blue() && robot.left_color.red() > robot.left_color.green()){
            robot.leftDriveMotor.setPower(0.0);
            robot.rightDriveMotor.setPower(-0.1);
        } else {
            robot.leftDriveMotor.setPower(0.0);
            robot.rightDriveMotor.setPower(0.0);
        }
        if (robot.leftDriveMotor.isBusy() && robot.rightDriveMotor.isBusy()) return false;
        else return true;
    }

    void lineTracking(){

        // Display weather the ODS sees the foam tile or the white line.
        if (robot.ods_Line.getRawLightDetected() > 0.2)
            telemetry.addLine("white line");
        else telemetry.addLine("foam tile");
        telemetry.update();

        while (robot.ods_Wall.getRawLightDetected() < 0.5) // robot is less than ~4cm from beacon
            if (robot.ods_Line.getRawLightDetected() < 0.2) { // ods sees foam
                robot.leftDriveMotor.setPower(-0.01);
                robot.rightDriveMotor.setPower(-0.1);
            } else { // ods sees white line
                robot.leftDriveMotor.setPower(-0.1);
                robot.rightDriveMotor.setPower(-0.01);
            }
    }
}
