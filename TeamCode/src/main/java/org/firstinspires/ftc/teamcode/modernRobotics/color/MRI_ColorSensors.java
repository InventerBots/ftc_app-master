/*
Modern Robotics Color Sensor Active & Passive Example using 2 colro sensors
Created 7/25/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 1.6
Reuse permitted with credit where credit is due

Configuration:
Color sensor at default I2C address of 0x3C named "ca"
Color sensor at I2C address of 0x60 named "cb"
Touch sensor named "t" and connected to the port specified by the user in their config file

This program can be run without a battery and Power Destitution Module

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode.modernRobotics.color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp(name = "MRI two color sensors", group = "sensors")
@Disabled
public class MRI_ColorSensors extends LinearOpMode {

    ColorSensor colorSensor1;  //Instance of ColorSensor - for reading color
    ColorSensor colorSensor2;  //Instance of ColorSensor - for reading color
    TouchSensor touch;         //Instance of TouchSensor - for changing color sensor mode

    @Override
    public void runOpMode() throws InterruptedException {
        //the below lines set up the configuration file
        colorSensor1 = hardwareMap.colorSensor.get("ca");
        colorSensor1.setI2cAddress(I2cAddr.create7bit(0x3e)); //Specify I2C Address of color sensor

        colorSensor2 = hardwareMap.colorSensor.get("cb");
        colorSensor2.setI2cAddress(I2cAddr.create7bit(0x3c)); //Specify I2C Address of color sensor

        touch = hardwareMap.touchSensor.get("t");

        boolean touchState = false;  //Tracks the last known state of the touch sensor
        boolean LEDState = true;     //Tracks the mode of the color sensor; Active = true, Passive = false

        waitForStart();  //Wait for the play button to be pressed

        colorSensor1.enableLed(LEDState);  //Set the mode of the LED; Active = true, Passive = false
        colorSensor2.enableLed(LEDState);
        //Active - For measuring reflected light. Cancels out ambient light
        //Passive - For measuring ambient light, eg. the FTC Color Beacon

        while (opModeIsActive()) {   //Main loop of program

            //The below two if() statements ensure that the mode of the color sensor is changed only once each time the touch sensor is pressed.
            //The mode of the color sensor is saved to the sensor's long term memory. Just like flash drives, the long term memory has a life time in the 10s or 100s of thousands of cycles.
            //This seems like a lot but if your program wrote to the long term memory every time though the main loop, it would shorten the life of your sensor.

            if (!touchState && touch.isPressed()) {  //If the touch sensor is just now being pressed (was not pressed last time through the loop but now is)
                touchState = true;                   //Change touch state to true because the touch sensor is now pressed
                LEDState = !LEDState;                //Change the LEDState to the opposite of what it was
                colorSensor1.enableLed(LEDState);    //Set the mode of the color sensor using LEDState
                colorSensor2.enableLed(LEDState);    //Set the mode of the color sensor using LEDState
            }
            if (!touch.isPressed()) {                //If the touch sensor is now pressed
                touchState = false;                  //Set the touchState to false to indicate that the touch sensor was released
            }

            //display values
            telemetry.addData("3 Red1  ", colorSensor1.red());
            telemetry.addData("4 Green1", colorSensor1.green());
            telemetry.addData("5 Blue1 ", colorSensor1.blue());

            telemetry.addData("6 Red2  ", colorSensor2.red());
            telemetry.addData("7 Green2", colorSensor2.green());
            telemetry.addData("8 Blue2 ", colorSensor2.blue());

            waitOneFullHardwareCycle();  //wait for all new data to go from the phone to the controllers and from the controllers to the phone.
        }  //End main loop of program
    }
}