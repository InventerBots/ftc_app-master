package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Luke on 3/22/2017.
 */

public class VuforiaDisplayDistanceAndAngleFromTarget {
    LinearOpMode opMode;

    // Declare two variables to use in other programs that use this class
    public double degrees_ToTurn;
    public double mm_ToDrive;

    public void initVuforia() throws InterruptedException {

        opMode.telemetry.addLine("Initialising...");

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        parameters.vuforiaLicenseKey = "ASIz7sT/////AAAAGfb3lVDO4Evbge6HWYydU2BT6A/r9GYtmWexQy+4JDwMW9t8z27eyX13pypPMbQTCl9YpqDukouTV+wJeZhMiAY6Sp+91uqspGGCQafg0Yf0IL40A9E8Py3QVDaY82M2El+PpiaakH8PxIMOrRD3BZwz71SiMNoToXX1ycXqE9bER5TcarUhu3m+ddfQgxR+VHkck/u1N70DK4i9at5bED1fvZ4HfaJegLQpxYFybrDgnM+odzO1mbANVERFklrS/Q1zamsqqjRoB24DuIxMXJR069n+AUN+/sU2IvcmGbkcUmkNkb22QuScaQZMeRXXlIO8TyLFqa0rPf3l2vZ81ejF+EbzhxdEMqA1+vf7Iuww";

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");

        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Gears");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Tools");

        opMode.telemetry.clearAll();
        opMode.telemetry.addLine("Vuforia is ready to start!");
        opMode.telemetry.update();

        opMode.waitForStart();

        beacons.activate();

        while (opMode.opModeIsActive()) {
            for (VuforiaTrackable beacon : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beacon.getListener()).getPose();

                if (pose != null) {
                    VectorF translation = pose.getTranslation();

                    mm_ToDrive = translation.magnitude();
                    degrees_ToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2))) - 183;


//                    opMode.telemetry.addData(beacons.getName() + " distance from target mm", (int) mm_ToDrive) // display the angle from the target in degrees.
//                            .addData(beacon.getName() + " degrees to turn", (int) degrees_ToTurn); // display distance from target in mm.
//                    if (degrees_ToTurn > 175 || degrees_ToTurn < -175)
//                        opMode.telemetry.addLine("straight with target");


                }
            }
            opMode.telemetry.update();
        }
    }
}