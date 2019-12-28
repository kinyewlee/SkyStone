package org.firstinspires.ftc.teamaztec;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware_Aztec {
    DcMotor motorFL, motorFR, motorRL, motorRR;
    Servo servoArm, servoWinch, servoHand, servoWrist;
    BNO055IMU imu;

    static final double ARM_MAX = 2.48d;
    static final double ARM_MIN = 0.57d;

    boolean turnClaw = false;
    boolean openClaw = false;

    Hardware_Aztec(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        motorFL = hardwareMap.dcMotor.get("motor_1");
        motorRL = hardwareMap.dcMotor.get("motor_0");
        motorFR = hardwareMap.dcMotor.get("motor_3");
        motorRR = hardwareMap.dcMotor.get("motor_2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.FORWARD);
        motorRR.setDirection(DcMotorSimple.Direction.REVERSE);

        servoArm = hardwareMap.servo.get("servo_arm");
        servoWinch = hardwareMap.servo.get("servo_winch");
        servoHand= hardwareMap.servo.get("servo_hand");
        servoWrist= hardwareMap.servo.get("servo_wrist");

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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while (!imu.isGyroCalibrated()) {
            Thread.yield();
        }
    }

    public void setDrivePower(double powerFL, double powerFR, double powerRL, double powerRR) {
        motorFL.setPower(powerFL);
        motorRR.setPower(powerRR);
        motorRL.setPower(powerRL);
        motorFR.setPower(powerFR);
    }

    double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (double) AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    void openClaw() {
        openClaw = !openClaw;
        double clawPosition = openClaw ? 0d : 0.55d;
        servoHand.setPosition(clawPosition);
    }

    void turnClaw() {
        turnClaw = !turnClaw;
        double wristPosition = turnClaw ? 1d : 0d;
        servoWrist.setPosition(wristPosition);
    }
}
