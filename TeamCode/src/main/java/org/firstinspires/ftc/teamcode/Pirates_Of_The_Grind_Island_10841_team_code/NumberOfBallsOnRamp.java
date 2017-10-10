package org.firstinspires.ftc.teamcode.Pirates_Of_The_Grind_Island_10841_team_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.I2cController;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

/**
 * Created by Luke on 2/24/2017.
 */

public class NumberOfBallsOnRamp extends LinearOpMode{
    Team_10841_Robot robot;

    protected UltrasonicSensor ultrasonicSensor;

    double ultrasonicValue = 0.0;
    protected double numberOfBalls = 0.0;
    protected final static double ballRampLaugh_CM = 30.0; // amusing that the ramp is 12in long
    protected final static double ballSize_CM = 9.525;

    @Override
    public void runOpMode() throws InterruptedException {
        ultrasonicSensor = hardwareMap.ultrasonicSensor.get("NumberOfBallsOnRamp sonar");
        ultrasonicValue = ultrasonicSensor.getUltrasonicLevel();

        waitForStart();

        if (opModeIsActive()) {

        }
    }
}
