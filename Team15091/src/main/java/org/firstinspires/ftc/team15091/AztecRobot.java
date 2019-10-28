package org.firstinspires.ftc.team15091;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class AztecRobot {
    HardwareMap hwMap = null;

    DcMotor motorFL = null;
    DcMotor motorRL = null;
    DcMotor motorFR = null;
    DcMotor motorRR = null;
    DcMotor motorArm = null;
    DcMotor motorWinch = null;
    AnalogInput sensorArm = null;
    BNO055IMU imu;
    Servo servoHook = null, servoHand = null, servoWrist = null;
    DistanceSensor sensorRange = null;

    private static final double COUNTS_PER_MOTOR_REV = 288d;    // eg: Core Hex Motor Encoder
    private static final double DRIVE_GEAR_REDUCTION = 1d;     // This is < 1.0 if geared UP, eg. 26d/10d
    private static final double WHEEL_DIAMETER_INCHES = 4d;     // For figuring circumference
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265359d);
    static final double ARM_MAX = 2.8d;
    static final double ARM_MIN = 0.5d;

    boolean hookDown = false;
    boolean turnClaw = false;
    boolean openClaw = false;
    private int beepSoundID;

    AztecRobot() {

    }

    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        motorFL = hwMap.dcMotor.get("motor_0");
        motorRL = hwMap.dcMotor.get("motor_1");
        motorFR = hwMap.dcMotor.get("motor_2");
        motorRR = hwMap.dcMotor.get("motor_3");
        motorArm = hwMap.dcMotor.get("motor_arm");
        motorWinch = hwMap.dcMotor.get("motor_winch");
        sensorArm = hwMap.analogInput.get("sensor_arm");
        servoHook = hwMap.servo.get("servo_hook");
        servoHand = hwMap.servo.get("servo_hand");
        servoWrist = hwMap.servo.get("servo_wrist");
        sensorRange = hwMap.get(DistanceSensor.class, "sensor_range");

        motorArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRR.setDirection(DcMotorSimple.Direction.REVERSE);

        motorWinch.setDirection(DcMotorSimple.Direction.REVERSE);
        motorWinch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorWinch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        setDrivePower(0, 0, 0, 0);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            Thread.yield();
        }

        hookDown = false;
        beepSoundID = hwMap.appContext.getResources().getIdentifier("beep", "raw", hwMap.appContext.getPackageName());
    }

    final void beep() {
        new Thread() {
            public void run() {
                SoundPlayer.getInstance().startPlaying(hwMap.appContext, beepSoundID);
            }
        }.start();
    }

    /**
     * set drive motor power
     *
     * @param powerFL front left
     * @param powerFR front right
     * @param powerRL rear left
     * @param powerRR rear right
     */
    public void setDrivePower(double powerFL, double powerFR, double powerRL, double powerRR) {
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorRL.setPower(powerRL);
        motorRR.setPower(powerRR);
    }

    void setDriveMode(DcMotor.RunMode runMode) {
        motorFL.setMode(runMode);
        motorFR.setMode(runMode);
        motorRL.setMode(runMode);
        motorRR.setMode(runMode);
    }

    void setDriveTarget(double distance, boolean moveSideway) {
        // Determine new target position, and pass to motor controller
        int moveCounts = (int) (distance * COUNTS_PER_INCH);

        int dirFL = moveSideway ? -1 : 1;
        int dirFR = moveSideway ? 1 : 1;
        int dirRL = moveSideway ? 1 : 1;
        int dirRR = moveSideway ? -1 : 1;

        int newFLTarget = motorFL.getCurrentPosition() + moveCounts * dirFL;
        int newFRTarget = motorFR.getCurrentPosition() + moveCounts * dirFR;
        int newRLTarget = motorRL.getCurrentPosition() + moveCounts * dirRL;
        int newRRTarget = motorRR.getCurrentPosition() + moveCounts * dirRR;

        // Set Target and Turn On RUN_TO_POSITION
        motorFL.setTargetPosition(newFLTarget);
        motorFR.setTargetPosition(newFRTarget);
        motorRL.setTargetPosition(newRLTarget);
        motorRR.setTargetPosition(newRRTarget);
    }

    void resetDrive() {
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motorFL.setZeroPowerBehavior(zeroPowerBehavior);
        motorFR.setZeroPowerBehavior(zeroPowerBehavior);
        motorRL.setZeroPowerBehavior(zeroPowerBehavior);
        motorRR.setZeroPowerBehavior(zeroPowerBehavior);
    }

    boolean isDriveBusy() {
        return motorFL.isBusy() && motorFR.isBusy() && motorRL.isBusy() && motorRR.isBusy();
    }

    /**
     * move main arm
     *
     * @param powerArm positive power move arm back, negative move arm out
     */
    void setArmPower(double powerArm) {
        double currentAngle = getArmAngle();
        if (powerArm > 0d && currentAngle < ARM_MAX) { //move arm back
            double gap = Math.abs(ARM_MAX - currentAngle);
            double error = Range.scale(gap, 0d, 2d, 0d, 1d);
            double suggestedPower = powerArm * error;
            double finalPower = Math.min(suggestedPower, powerArm);
            motorArm.setPower(finalPower);
        } else if (powerArm < 0d && currentAngle > ARM_MIN) { //move arm out
            double gap = Math.abs(currentAngle - ARM_MIN);
            double error = Range.clip(gap, 0d, 1d);
            double suggestedPower = powerArm * error;
            double finalPower = Math.max(suggestedPower, powerArm);
            motorArm.setPower(finalPower);
        } else {
            motorArm.setPower(0d);
        }
    }

    double getArmAngle() {
        return sensorArm.getVoltage();
    }

    double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (double) AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    void modifyHook() {
        hookDown = !hookDown;
        double hookPosition = hookDown ? 1 : 0;
        servoHook.setPosition(hookPosition);
    }

    void openClaw() {
        openClaw = !openClaw;
        double clawPosition = openClaw ? 0.4 : 0;
        servoHand.setPosition(clawPosition);
    }

    void turnClaw() {
        turnClaw = !turnClaw;
        double wristPosition = turnClaw ? 1 : 0;
        servoWrist.setPosition(wristPosition);
    }
}
