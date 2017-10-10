package org.firstinspires.ftc.teamcode.Pirates_Of_The_Grind_Island_10841_team_code;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import static java.lang.Thread.enumerate;
import static java.lang.Thread.sleep;

/**
 * Created by Luke on 11/11/2016.
 */

@TeleOp(name = "TeleOp", group = "PGI_10841")
@Disabled
public class twoSpeedTeleOp extends LinearOpMode{
    Team_10841_Robot robot  =new Team_10841_Robot();  //use Team_10841_Robot hardware

    boolean high_speed = false;             // Servo mid position
    boolean right_bumper_state;                // keep track if the button state has changed
    double left;                               // current calculated left motor speed from gamepad1.left_stick
    double right;                              // current calculated right motor speed from gamepad1.left_stick
    double intake;                             // intake motor from gamepad2.left_stick
    int CatapultTicksPerLaunch = (256 * 28);  // gear reduction 256:1 * Motor ticks per revolution
    boolean inStartup = true;
    boolean CatapultButtonPushed = false;
    double turnMultiplier = 0.45;// turn factor to make the robot more controllable when turning


    //start of program
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);    //initialise Team_10841_Robot's hardware map
        right_bumper_state = gamepad1.right_bumper;  // initialize the bumper state

        robot.rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftDriveMotor.setPower(0.0);
        robot.rightDriveMotor.setPower(0.0);
        robot.intake.setPower(0.0);
        // Send telemetry message to signify robot waiting
        showInfo();

        /*wait for start button to be pressed*/
        waitForStart();

        /*loop until stop button is pressed*/
        while (opModeIsActive()) {
            /* -------------- Drive Motor Control --------------------------------------------------
            Left and right drive motors are controlled in this section.  The calculations are to make
            the joystick less responsive to give the operator more control
            */

            // Run wheels in Arcade mode (note: The joystick goes negative when pushed forwards, so negate it)


             /*  --------------- intake motor control ----------------------------------------------
              Could use the second gamepad to control the intake so the driver will not be conserned with
              watching the intake. The left joystick will control if the intake is accepting or rejecting
              the current game particle.  Allows for a second driver.  This is also placed on gamepad1
              for a single operator
             */
            if (inStartup){
                if (!robot.catapultTouchSensor.isPressed())
                    robot.StartCatapultCycle();
                inStartup = false;
            }

            twoSpeed();  // set the game driver joystick in high or low speed.
            intake = (gamepad1.right_stick_y);
            robot.intake.setPower(intake);

            if (gamepad1.left_bumper && robot.catapultIdle) {  // driver initiated NumberOfBallsOnRamp launch
                robot.StartCatapultCycle(); // no longer waits for the touch sensor release
                /*
                 * assume the touch sensor is still pushed so set CatapultButtonPush to true
                 * Later we'll check if it is no longer pushed, then set CatapultButtonPushed to
                 * false so we know it is ok to EndCatapultCycle once the catapult presses the
                 * touch sensor.
                 */
                CatapultButtonPushed = true;
            }
            if (gamepad2.left_bumper && robot.catapultIdle) {  // driver initiated NumberOfBallsOnRamp launch
                robot.StartCatapultCycle(); // no longer waits for the touch sensor release
                /*
                 * assume the touch sensor is still pushed so set CatapultButtonPush to true
                 * Later we'll check if it is no longer pushed, then set CatapultButtonPushed to
                 * false so we know it is ok to EndCatapultCycle once the catapult presses the
                 * touch sensor.
                 */
                CatapultButtonPushed = true;
            }

            /*
             * has the catapult touch sensor been released?
             */
            if (CatapultButtonPushed && !robot.catapultTouchSensor.isPressed())
                CatapultButtonPushed = false;

            /*
             * after the catapult touch sensor has been released, start checking if it is pressed
             * again
             */
            if (!CatapultButtonPushed && robot.catapultTouchSensor.isPressed()) {  // end of catapult cycle, loads the next NumberOfBallsOnRamp
                robot.EndCatapultCycle(true);
            }

            /*
             * x button is to load NumberOfBallsOnRamp if we have fired all the balls and the catapult is ready to
             * fire a NumberOfBallsOnRamp is not in position
             */
            if (gamepad1.x && robot.catapultIdle) {
                robot.loadBall();
            }
            if (gamepad2.x && robot.catapultIdle) {
                robot.loadBall();
            }

            /*
             * red button B is in case we have problems with the catapultTouchSensor switch.
             * This is a manual override to tell the catapult it can reload, hopefully at the
             * bottom of the stroke.
             */
            if (gamepad1.b && !robot.catapultIdle) {
                robot.EndCatapultCycle(true);
            }
            if (gamepad2.b && !robot.catapultIdle) {
                robot.EndCatapultCycle(true);
            }

            /*
             * left trigger raises the NumberOfBallsOnRamp lift, the right trigger lowers the lift
             */
            if (Math.abs(gamepad2.left_stick_y) > 0.001) {
                telemetry.addData("Ball lift Moving ",gamepad2.left_stick_y);
                robot.ballLift(gamepad2.left_stick_y);
            } else {
                telemetry.addLine("Ball lift stopped ");
                robot.ballLift(0);
            }
            showInfo();
            idle();
            }
    }

    public void showInfo() {
        // Send telemetry message to signify robot running;
        telemetry.addData("left", left); // drive motor
        telemetry.addData("right", right); // drive motor
        telemetry.addData("intake", intake);
        telemetry.addData("speed", high_speed ? "Warp Drive Engaged" : "Impulse Power");
        telemetry.addData("Cat sensor",robot.catapultTouchSensor.isPressed() ? "Down" : "Up");
        telemetry.addData("Start Cat Btn Pressed",gamepad1.left_bumper ? "Pressed" : "");
        telemetry.addData("StartCatapultFunction position", robot.catapult.getCurrentPosition());
        updateTelemetry(telemetry);

    }
    public void twoSpeed(){
        left = (-robot.JoystickToMotorVal(gamepad1.left_stick_y)) +
                (robot.JoystickToMotorVal(gamepad1.left_stick_x)* turnMultiplier);  // we divide the x axis so left and right respond less to the joystick
        right = (-robot.JoystickToMotorVal(gamepad1.left_stick_y)) -
                (robot.JoystickToMotorVal(gamepad1.left_stick_x)* turnMultiplier);

        if (gamepad2.right_bumper) { // if in capball lifting mode...
            if (gamepad1.right_bumper) { // if in hyper drive mode, drive at 5% power
                left *= 0.5;
                right *= 0.5;
            } else { // otherwise, drive at 3% power
                left *= 0.3;
                right *= 0.3;
            }
        }
        else
        {
            if (gamepad1.right_bumper) {
                high_speed = true;
                // hyper drive mode - leave the calculations at 100%
            } else {
                high_speed = false;
                left *= robot.normalWarpFactor;
                right *= robot.normalWarpFactor;
            }
        }
        robot.setDriveWheelSpeed(right,left);
//        if(!high_speed) {
//            left *= 0.25;
//            right *= 0.25;
//        }// determine if low speed button has been pressed to switch to low or high speed motor control
//        robot.setDriveWheelSpeed(right,left);
//        if (right_bumper_state =gamepad1.right_bumper) {
//            if (gamepad1.right_bumper) {
//                high_speed = !high_speed;
//            }
//            right_bumper_state =gamepad1.right_bumper;
//        }
    }
//    void StartCatapultCycle() throws InterruptedException {
//        catapultIdle = false;
//        robot.catapult.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.catapult.setPower(1.0);
//        while (robot.catapultTouchSensor .isPressed()){} // do nothing!!
//         sleep(1000);
//    }
//    void EndCatapultCycle() throws InterruptedException {
//        if (!catapultIdle) {
//            robot.catapult.setPower(0.0);
//            loadBall(); // load the NumberOfBallsOnRamp onto the catapult arm
//            catapultIdle = true;
//        }
//    }
//    void loadBall() throws InterruptedException {
//        robot.ballPusher.setPosition(0.0);
//        sleep(500);
//        robot.ballPusher.setPosition(1.0);
//    }

}
