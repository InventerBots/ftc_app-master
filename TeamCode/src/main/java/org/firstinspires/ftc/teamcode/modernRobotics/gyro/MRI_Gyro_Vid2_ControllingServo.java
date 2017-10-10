package org.firstinspires.ftc.teamcode.modernRobotics.gyro;

/*
Modern Robotics Gyro Controlling Servo Example
Updated 11/3/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
Gyro Sensor named "gyro"
Servo named "servo"

For more information, go to http://modernroboticsedu.com/course/view.php?id=4
Support is available by emailing support@modernroboticsinc.com.
*/

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="2 Controlling Servo", group="Gyro Video")
@Disabled
public class MRI_Gyro_Vid2_ControllingServo extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    Servo servo;
    double servoPosition = 127;

    int zAccumulated;  //Total rotation left/right

    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        servo = hardwareMap.servo.get("servo");

        servo.setPosition((servoPosition / 255));
        telemetry.addData("Servo", (servoPosition / 255));
        telemetry.addData("Servo Position", servoPosition);

        sensorGyro = hardwareMap.gyroSensor.get("gyro");  //Point to the gyro in the configuration file
        mrGyro = (ModernRoboticsI2cGyro)sensorGyro;      //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()
        mrGyro.calibrate();  //Calibrate the sensor so it knows where 0 is and what still is. DO NOT MOVE SENSOR WHILE BLUE LIGHT IS SOLID
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        while (mrGyro.isCalibrating()) { //Ensure calibration is complete (usually 2 seconds)
        }
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());

        zAccumulated = mrGyro.getIntegratedZValue();  //Set variables to gyro readings

        servoPosition = (127 + ((zAccumulated * 128) / 90));
        servo.setPosition(Range.clip(servoPosition / 255, 0, 1));

        telemetry.addData("2. accu", String.format("%03d", zAccumulated));  //Display variables to Driver Station Screen
        telemetry.addData("Servo", (servoPosition / 255));
        telemetry.addData("Servo Position", servoPosition);
    }

    @Override
    public void stop() {
    }

}
