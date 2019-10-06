package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

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
    AnalogInput sensorArm = null;
    BNO055IMU imu;

    static final double ARM_MAX = 3d;
    static final double ARM_MIN = 0.5d;

    AztecRobot() {
    }

    void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        motorFL = hwMap.dcMotor.get("motor_0");
        motorRL = hwMap.dcMotor.get("motor_1");
        motorFR = hwMap.dcMotor.get("motor_3");
        motorRR = hwMap.dcMotor.get("motor_2");
        motorArm = hwMap.dcMotor.get("motor_arm");
        sensorArm = hwMap.analogInput.get("sensor_arm");

        motorFL.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorRR.setDirection(DcMotorSimple.Direction.FORWARD);

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
    }

    void setDrivePower(double powerFL, double powerFR, double powerRL, double powerRR) {
        motorFL.setPower(powerFL);
        motorFR.setPower(powerFR);
        motorRL.setPower(powerRL);
        motorRR.setPower(powerRR);
    }

    void setArmPower(double powerArm) {
        if (powerArm > 0d && getArmAngle() < ARM_MAX) {
            double gap = Math.abs(ARM_MAX - getArmAngle());
            double error = Range.clip(gap, 0d, 1d);
            motorArm.setPower(powerArm * error);
        } else if (powerArm < 0d && getArmAngle() > ARM_MIN) {
            double gap = Math.abs(getArmAngle() - ARM_MIN);
            double error = Range.clip(gap, 0d, 1d);
            motorArm.setPower(powerArm * error);
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
}
