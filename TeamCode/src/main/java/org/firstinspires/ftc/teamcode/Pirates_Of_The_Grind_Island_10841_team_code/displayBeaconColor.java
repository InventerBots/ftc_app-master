package org.firstinspires.ftc.teamcode.Pirates_Of_The_Grind_Island_10841_team_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by Luke on 12/6/2016.
 */

@TeleOp(name = "display beacon color")
@Disabled
public class displayBeaconColor extends LinearOpMode {
    Team_10841_Robot robot =new Team_10841_Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            if (robot.left_color.blue() > robot.left_color.red() && robot.left_color.blue() > robot.left_color.green())
                telemetry.addLine("left color is blue");
            else if (robot.left_color.red() > robot.left_color.blue() && robot.left_color.red() > robot.left_color.green())
                telemetry.addLine("left color is red");
            else telemetry.addLine("left color is green");

            if (robot.right_color.blue() > robot.right_color.red() && robot.right_color.blue() > robot.right_color.green())
                telemetry.addLine("right color is blue");
            else if (robot.right_color.red() > robot.right_color.blue() && robot.right_color.red() > robot.right_color.green())
                telemetry.addLine("right color is red");
            else telemetry.addLine("right color is green");
            telemetry.update();
        }
    }
}
