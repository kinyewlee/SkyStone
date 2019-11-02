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
    DcMotor leftDrive, rightDrive, motorArm;
    AnalogInput sensorArm;
    Servo servoWrist, servoHand, servoClamp;
    BNO055IMU imu;

    static final double ARM_MAX = 2.48d;
    static final double ARM_MIN = 0.57d;

    boolean turnClaw = false;
    boolean openClaw = false;

    Hardware_Aztec(HardwareMap hardwareMap) {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.dcMotor.get("left_drive");
        rightDrive = hardwareMap.dcMotor.get("right_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        motorArm = hardwareMap.dcMotor.get("motor_arm");
        motorArm.setDirection(DcMotor.Direction.REVERSE);
        sensorArm = hardwareMap.analogInput.get("sensor_arm");
        servoWrist = hardwareMap.servo.get("servo_wrist");
        servoHand = hardwareMap.servo.get("servo_hand");
        servoClamp = hardwareMap.servo.get("servo_clamp");

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

    void setArmPower(double powerArm) {
        double currentAngle = getArmAngle();
        if (powerArm < 0d && currentAngle < ARM_MAX) { //move arm out
            double gap = Math.abs(ARM_MAX - currentAngle);
            double error = Range.scale(gap, 0d, 1.9d, 0d, 1d);
            double suggestedPower = powerArm * error;
            double finalPower = Math.min(suggestedPower, powerArm);
            motorArm.setPower(finalPower);
        } else if (powerArm > 0d && currentAngle > ARM_MIN) { //move arm back
            double gap = Math.abs(currentAngle - ARM_MIN);
            double error = Range.clip(gap, 0d, 1d);
            double suggestedPower = powerArm * error;
            double finalPower = Math.max(suggestedPower, powerArm);
            motorArm.setPower(finalPower);
        } else {
            motorArm.setPower(0d);
        }


    }

    void setPower(double leftPower, double rightPower) {
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }

    double getArmAngle() {
        return sensorArm.getVoltage();
    }

    double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (double) AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.firstAngle);
    }

    void turnClaw() {
        turnClaw = !turnClaw;
        double wristPosition = turnClaw ? 1d : 0d;
        servoHand.setPosition(wristPosition);
    }

    void toggleClaw() {
        openClaw = !openClaw;
        double clawPosition = openClaw ? 0.52d : 0d;
        servoClamp.setPosition(clawPosition);
    }
}
