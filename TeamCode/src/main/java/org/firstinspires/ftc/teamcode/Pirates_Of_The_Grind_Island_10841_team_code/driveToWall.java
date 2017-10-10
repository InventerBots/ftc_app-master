package org.firstinspires.ftc.teamcode.Pirates_Of_The_Grind_Island_10841_team_code;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Luke on 11/27/2016.
 */

@Autonomous(name = "drive to wall")
@Disabled
public class driveToWall extends LinearOpMode {
    Team_10841_Robot robot = new Team_10841_Robot();

    UltrasonicSensor ultrasonicSensor;
    ModernRoboticsAnalogOpticalDistanceSensor ods;
    ModernRoboticsAnalogOpticalDistanceSensor odsFront;

    int wallDistance;
    int ultrasonicSensorValue;
    int odsValue;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        ultrasonicSensor = hardwareMap.ultrasonicSensor.get("left sonar");
        ods = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods_Line");
        odsFront = (ModernRoboticsAnalogOpticalDistanceSensor) hardwareMap.opticalDistanceSensor.get("ods_Line front");
        wallDistance = 10;//centimeters from the wall to start turn to push the color button
        ultrasonicSensorValue = (int)  ultrasonicSensor.getUltrasonicLevel();

        waitForStart();

        changeBeaconColor("red",false);
        /*
        while (opModeIsActive()) {
            driveUsingUltrasonicSensor(wallDistance);
            telemetry.addData("Ultrasonic Sensor OL = ", ultrasonicSensor.getUltrasonicLevel());
            updateTelemetry(telemetry);
        }
        */
    }
    void setDrivePower(double right, double left) {
        robot.leftDriveMotor.setPower(left);
        robot.rightDriveMotor.setPower(right);
    }

    void getUltrasonicSensorValue() {  // ignores 0 values and sets a valid distance (cm)
        ultrasonicSensorValue = (int) ultrasonicSensor.getUltrasonicLevel();
        while (opModeIsActive() && ultrasonicSensorValue == 0) {
            ultrasonicSensorValue = (int) ultrasonicSensor.getUltrasonicLevel();
        }
    }

    void changeBeaconColor(String TargetColor, Boolean turnLeft) throws InterruptedException {
        /* TargetColor "red" or "blue"
         *   ** First thing to do is find the line.  It should be straight in front of the robot. So move
         *      forward slowly until we find it.
         *   ** Next start turning the requested direction until we are back off the line, then
         *      right until we're back onto the line and repeat until we reach the distance from
         *      the wall as was requested
         *   ** Now check the color sensors to determine the direction the robot needs to turn to
         *      press the correct button to change the beacon color.
         */
        double maxPower = 0.08;

        // Find the line
        odsValue = (int) ods.getRawLightDetected();
        while (opModeIsActive() && (odsValue < 1)){
            setDrivePower(-0.1,-0.1);
            odsValue = (int) ods.getRawLightDetected();
        }

        // we may not need to stop here
        setDrivePower(0.0,0.0);


        if (turnLeft) {
            getUltrasonicSensorValue();
            while (opModeIsActive() &&  (ultrasonicSensorValue > wallDistance)) {
                 if (odsValue > 1) {
                     setDrivePower(-maxPower, -0.0);
                 } else {
                     setDrivePower(-0.0,-maxPower);
                 }
                sleep(25);
                getUltrasonicSensorValue();
                odsValue = (int) ods.getRawLightDetected();
             }
        } else{    // turn right
            while (opModeIsActive() &&  (ultrasonicSensorValue > wallDistance)) {
                if (odsValue > 1) {
                    setDrivePower(-0.0, -maxPower);
                } else {
                    setDrivePower(-maxPower, -0.0);
                }
                getUltrasonicSensorValue();
                odsValue = (int) ods.getRawLightDetected();
            }
        }
        setDrivePower(0.0,0.0);
// Now we need to push the button
    }






    void driveUsingUltrasonicSensor(double distanceToWall){
        boolean done = false;
        ultrasonicSensorValue = (int) ultrasonicSensor.getUltrasonicLevel();
        while (!done) {
            if (ultrasonicSensorValue > distanceToWall) {
                robot.leftDriveMotor.setPower(-0.1);
                robot.rightDriveMotor.setPower(-0.1);
            } else {
                if ( ultrasonicSensorValue <  distanceToWall) {
                    robot.leftDriveMotor.setPower(0.1);
                    robot.rightDriveMotor.setPower(0.1);
                } else {
                    done = true;
                }
            }

            ultrasonicSensorValue = (int) ultrasonicSensor.getUltrasonicLevel();
            telemetry.addData("Ultrasonic Sensor = ", ultrasonicSensorValue);
            updateTelemetry(telemetry);
            while (ultrasonicSensorValue == 0) {
                telemetry.addLine("Ultrasonic returned a zero value");
                ultrasonicSensorValue = (int) ultrasonicSensor.getUltrasonicLevel();
            }
        }
        robot.leftDriveMotor.setPower(0.0);
        robot.rightDriveMotor.setPower(0.0);
    }

}
