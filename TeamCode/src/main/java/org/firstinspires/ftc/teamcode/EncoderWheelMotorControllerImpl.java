package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Luke on 5/8/2017.
 */
public class EncoderWheelMotorControllerImpl implements EncoderWheelMotorController {
    private boolean enabled = true;

    private int target;

    // return weather or not the PID loop is enabled
    public boolean isEnabled(boolean enabled) {
        return enabled;
    }

    // start of PID
    public int PID(AnalogInput encoder, int target) {
        int encoderPosition = 0;
        target = this.target;
        
        return encoderPosition;
    }
}