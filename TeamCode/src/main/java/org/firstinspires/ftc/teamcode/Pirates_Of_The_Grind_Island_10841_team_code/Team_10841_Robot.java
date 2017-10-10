package org.firstinspires.ftc.teamcode.Pirates_Of_The_Grind_Island_10841_team_code;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Thread.sleep;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * This hardware class is for the 2016-17 Velocity Vortex 10841 robot.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * <p>
 * Motor channel 1:  Left  drive motor a :        "left motor a"
 * Motor channel 2:  Right drive motor a:         "right motor a"
 * Motor channel 3:  Intake motor :               "intake motor"
 * Motor channel 4:  Catapult motor :             "catapult motor"
 */
public class Team_10841_Robot {
    /* calculations parameters
     * gear ratio: motor = 20t wheel = 18t = 1:0.9
     * ticks per motor revolution = 28
     * motor gear reduction = 16:1
     */
    public static final double driveWheelTicksPerRevolution = (0.9 * 28 * 16);
    /*  Wheel diameter is 3 inches
     * the circumference is (3 * PI) inches
     * so ticks per inch is driveWheelTicksPerRevolution / circumference
     */
    public static final double driveWheelTicksPerInch = driveWheelTicksPerRevolution / (Math.PI * 3);

    public boolean catapultIdle = true;               // true = catapult is waiting for the launch command

    // motors
    public DcMotor leftDriveMotor = null;
    public DcMotor rightDriveMotor = null;
    public DcMotor intake = null;
    public DcMotor catapult = null;
    public DcMotor liftmotor = null;
    // servo
    public Servo ballPusher = null;

    // touch sensor
    public TouchSensor catapultTouchSensor;

    // ultrasonic sensors
//    public UltrasonicSensor right_sonar;
//    public UltrasonicSensor left_sonar;

    // optical distance sensors
    public ModernRoboticsAnalogOpticalDistanceSensor ods_Line;
    public ModernRoboticsAnalogOpticalDistanceSensor ods_Wall;

    // color sensors
    public ColorSensor left_color;
    public ColorSensor right_color;

    // gyro sensor
//    public ModernRoboticsI2cGyro gyro;



    double maxDrivePower = 0.4;
    double drivePower;
    double stop = 0.0;
    int zAccumulated;  //Total rotation left/right
    int ticksToMove;
    int slowZone = setTargetInches(6.0);
    int currentRotation;
    int wallDistance;
    double left_sonar_Value;
    double right_sonar_Value;
    double odsValue;
    boolean firstBeaconSuccess;
    boolean secondBeaconSuccess;
//    boolean initGyro = false;

    /* Local Autonomous_color_sensing_OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    public double normalWarpFactor = 0.6;


    /* Constructor */
    public Team_10841_Robot() {
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) throws InterruptedException {
        // save reference to HW Map
        hwMap = ahwMap;


        // Define and Initialize the Motors
        leftDriveMotor = hwMap.dcMotor.get("left drive");
        rightDriveMotor = hwMap.dcMotor.get("right drive");
        intake = hwMap.dcMotor.get("intake motor");
        catapult = hwMap.dcMotor.get("catapult");
        liftmotor = hwMap.dcMotor.get("lift motor");

        // Define and Initialize the Servo

        ballPusher = hwMap.servo.get("NumberOfBallsOnRamp servo");

        // Define and Initialize the Touch Sensor
        catapultTouchSensor = hwMap.touchSensor.get("cat touch");

        // Define and Initialize the Ultrasonic Sensors
//        left_sonar = hwMap.ultrasonicSensor.get("left sonar");
//        right_sonar = hwMap.ultrasonicSensor.get("right sonar");

        // Define and Initialize the Optical Distance Sensor
        ods_Line = (ModernRoboticsAnalogOpticalDistanceSensor) hwMap.opticalDistanceSensor.get("ods line");
        ods_Wall = (ModernRoboticsAnalogOpticalDistanceSensor) hwMap.opticalDistanceSensor.get("ods wall");

        // Define and Initialize the Color Sensors

        left_color = hwMap.colorSensor.get("left color");
        I2cAddr left_color_addr = I2cAddr.create8bit(0x3a);
        left_color.setI2cAddress(left_color_addr);
        left_color.enableLed(false);

        right_color = hwMap.colorSensor.get("right color");
        I2cAddr right_color_addr = I2cAddr.create8bit(0x3c);
        right_color.setI2cAddress(right_color_addr);
        right_color.enableLed(false);

        // Define and Initialize the Gyro Sensor
//        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");
        // get a reference to a Modern Robotics GyroSensor object.
//        gyro.calibrate();

        // make sure the gyro is calibrated.
//        while (gyro.isCalibrating()) {
//            Thread.sleep(50);
//        }

        // Reverse the Left Drive Motor for operator drive mode
        SetAutonomousMode(false);

        // Stop the Motors
        leftDriveMotor.setPower(0.0);
        rightDriveMotor.setPower(0.0);
        intake.setPower(0.0);
        catapult.setPower(0.0);
        liftmotor.setPower(0.0);

        //ballPusher.setPosition(0.5);

        // Set the Run Mode of the drive
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set the Run Mode of the catapult
        catapult.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the Run Mode of the intake motor
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    void ballLift(double power){
        liftmotor.setPower(Range.clip(power,-1.0,1.0));
    }

     /*
    * Change the joystick value to a curve value instead of a linear value
    */

    public double JoystickToMotorVal(double xyVal) {
        return Math.pow((xyVal * 100), 3) / 1000000;
    }

    void SetAutonomousMode(boolean AutonomousMode){
        if(AutonomousMode) {
            leftDriveMotor.setDirection(DcMotor.Direction.FORWARD);
            rightDriveMotor.setDirection(DcMotor.Direction.REVERSE);
        }
        else {
            leftDriveMotor.setDirection(DcMotor.Direction.REVERSE);
            rightDriveMotor.setDirection(DcMotor.Direction.FORWARD);
        }
    }


    void StartCatapultCycle() throws InterruptedException {
        catapultIdle = false;
        catapult.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        catapult.setPower(1.0);
    }

    void EndCatapultCycle(boolean loadball) throws InterruptedException {
        if (!catapultIdle) {
            catapult.setPower(0.0);
            if (loadball)
                loadBall(); // load the NumberOfBallsOnRamp onto the catapult arm
            catapultIdle = true;
        }
    }

    /*
    void EndCatapultCycle(boolean loadball) throws InterruptedException {
        catapult.setPower(0.0); // stop catapult motor
        NeedToLoadBall = 0;
        }
    void loadBall() throws InterruptedException {
        Integer NeedToLoadBall = 0;
        switch (NeedToLoadBall) {
            case 0:{
                ballPusher.setPosition(0.0);
                NeedToLoadBall = 1;
//                set a timer
            }
            case 1:{
//                check the timer and if time
                ballPusher.setPosition(1.0);
                NeedToLoadBall = 2;
            }
            case 2:;
        }


     */

    void loadBall() throws InterruptedException {
        ballPusher.setPosition(0.0);
        sleep(500);
        ballPusher.setPosition(1.0);
        sleep(100);
    }


    void stopDrive() {
        leftDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDriveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDriveMotor.setTargetPosition(leftDriveMotor.getCurrentPosition());
        rightDriveMotor.setTargetPosition(rightDriveMotor.getCurrentPosition());
        leftDriveMotor.setPower(1.0);
        rightDriveMotor.setPower(1.0);
    }



    double RampDown(int distToGo, int slowDistance, double minSpeed, double maxSpeed) {
//        double target = (((double) distToGo / (double) slowDistance) * (maxSpeed - minSpeed)) + minSpeed;
//        if (target < minSpeed) target = minSpeed;
//        if (target > maxSpeed) target = maxSpeed;
//        return target;
         double target = RampUp(distToGo,slowDistance,minSpeed,maxSpeed);
        return target;
    }

    double RampUp(int distTraveled, int slowDistance, double minSpeed, double maxSpeed) {
        double target = (((double) distTraveled / (double) slowDistance) * (Math.abs(maxSpeed) - Math.abs(minSpeed))) + Math.abs(minSpeed);

        if (target < minSpeed) target = minSpeed;
        if (target > maxSpeed) target = maxSpeed;

        if (maxSpeed > 0)
           return target;
        else
            return -target;
    }


    /*
     * set both wheels speeds with one statement.
     */

    void setDriveWheelSpeed(double right, double left) {
        leftDriveMotor.setPower(Range.clip(left,-1.0,1.0));
        rightDriveMotor.setPower(Range.clip(right,-1.0,1.0));
    }


    /*
     * converts centimeters to inches for the driveForDistanceInches routine.
     */
    double wheel_angleToTicks(double turnAngle) {
        /*
         * return number of encoder ticks per degree
         * set left negative of this value set right positive of this value
         * 14.45 is the diameter of the arc that the wheels will travel in and 3 is the diameter of
         * the wheels.
         */
        double robotCircumference = (14.45 * Math.PI);
        double wheelCircumference = (3 * Math.PI);
        return Math.round(((robotCircumference / wheelCircumference / 360) * turnAngle) * driveWheelTicksPerRevolution);
    }

    /*
     * Convert inches to encoder ticks for the wheel motors.
     */
    public int setTargetInches(double distance) {
        /*motor ticks per revolution times distance per revolution*/
        return (int) (distance * driveWheelTicksPerInch);
    }

//    public double getRightUltrasonicSensorValue() {  // ignores 0 values and sets a valid distance (cm)
//        double ultrasonicSensorValue = (int) right_sonar.getUltrasonicLevel();
//        int I = 0;
//        while (I < 10 && ultrasonicSensorValue == 0) {
//            I++;
//            ultrasonicSensorValue = (int) right_sonar.getUltrasonicLevel();
//        }  // Hopefully we get a non zero value in 10 tries
//        return ultrasonicSensorValue;
//    }
// color censors are backward
    public boolean leftBeaconButtonIsRed() {
        /*
        Check the right color sensor to determine if the Right side of the beacon is red.  If it is
        return true;
        */
        boolean isred;
        if (right_color.red() > right_color.blue() && right_color.red() > left_color.green()) {
            isred = true; // return true if the right color sensor "sees" red
        } else isred = false; // otherwise, return false
        return isred;
    }

    public boolean rightBeaconButtonIsRed() {
        /*
        Check the left color sensor to determine if the Left side of the beacon is red.  If it is
        return true;
        */
        boolean isred;
        if (left_color.red() > left_color.blue() && left_color.red() > left_color.green()) {
            isred = true; // return true if the left color sensor "sees" red
        } else isred = false; // otherwise, return false
        return isred;
    }


    public boolean leftBeaconButtonIsBlue() {
        /*
        Check the right color sensor to determine if the Right side of the beacon is red.  If it is
        return true;
        */
        boolean isblue;
        if (right_color.blue() > right_color.red() && right_color.blue() > right_color.green()) {
            isblue = true; // return true if the right color sensor "sees" red
        } else isblue = false; // otherwise, return false
        return isblue;
    }

    public boolean rightBeaconButtonIsBlue() {
        /*
        Check the left color sensor to determine if the Left side of the beacon is red.  If it is
        return true;
        */
        boolean isblue;
        if (left_color.blue() > left_color.red() && left_color.blue() > left_color.green()) {
            isblue = true; // return true if the left color sensor "sees" red
        } else isblue = false; // otherwise, return false
        return isblue;
    }
}

