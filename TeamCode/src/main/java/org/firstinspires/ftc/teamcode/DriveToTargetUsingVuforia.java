package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Luke on 8/4/2017.
 */

@Autonomous (name = "drive to target using Vuforia", group = "Vuforia")
public class DriveToTargetUsingVuforia extends LinearOpMode {
    VuforiaDisplayDistanceAndAngleFromTarget vuforia; // declare the vuforia field

    @Override
    public void runOpMode() throws InterruptedException {
        vuforia.initVuforia(); // initialise vuforia

        waitForStart(); // wait for the opmode to start

        while (opModeIsActive()) {
            telemetry.addData("distance from target in mm:", vuforia.mm_ToDrive);
            telemetry.log().add("distance:", vuforia.mm_ToDrive);
            telemetry.addData("degrees to turn:", vuforia.degrees_ToTurn);
            telemetry.log().add("degrees:", vuforia.degrees_ToTurn);
        }
    }
}
