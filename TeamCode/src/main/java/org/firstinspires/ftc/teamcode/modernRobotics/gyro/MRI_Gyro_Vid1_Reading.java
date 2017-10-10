package org.firstinspires.ftc.teamcode.modernRobotics.gyro;

/*
Modern Robotics Gyro Read Example
Updated 11/3/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.35
Reuse permitted with credit where credit is due

Configuration:
Gyro Sensor named "gyro"

For more information, go to http://modernroboticsedu.com/course/view.php?id=4
Support is available by emailing support@modernroboticsinc.com.
*/

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="1 Gyro Read", group="Gyro Video")
@Disabled
public class MRI_Gyro_Vid1_Reading extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();

    int zAccumulated;  //Total rotation left/right
    int heading;       //Heading left/right. Integer between 0 and 359
    int xVal, yVal, zVal;  //Momentary rate of rotation in three axis

    GyroSensor sensorGyro;  //General Gyro Sensor allows us to point to the sensor in the configuration file.
    ModernRoboticsI2cGyro mrGyro;  //ModernRoboticsI2cGyro allows us to .getIntegratedZValue()

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

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

        heading = 360 - mrGyro.getHeading();  //Reverse direction of heading to match the integrated value
        if (heading == 360) {
            heading = 0;
        }

        xVal = mrGyro.rawX() / 128;  //Lowest 7 bits is noise
        yVal = mrGyro.rawY() / 128;
        zVal = mrGyro.rawZ() / 128;

        telemetry.addData("1. heading", String.format("%03d", heading));  //Display variables to Driver Station Screen
        telemetry.addData("2. accu", String.format("%03d", zAccumulated));
        telemetry.addData("3. X", String.format("%03d", xVal));
        telemetry.addData("4. Y", String.format("%03d", yVal));
        telemetry.addData("5. Z", String.format("%03d", zVal));
    }

    @Override
    public void stop() {
    }

}
