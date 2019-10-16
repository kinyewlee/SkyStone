package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

abstract class Autonomous_Base extends LinearOpMode {
    private static final double P_TURN_COEFF = 0.09d;     // Larger is more responsive, but also less stable
    private static final double HEADING_THRESHOLD = 1d;      // As tight as we can make it with an integer gyro
    private static final double P_DRIVE_COEFF = 0.16d;     // Larger is more responsive, but also less stable
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    private static final String VUFORIA_KEY =
            "AYuhzJv/////AAABmeg8aNmI7008toUzc6hhMIgtAFhnIF3mx6avMFksv/Jm2qTivu8ialyM7onEEB2F7JVsgl3MXkaV1ZooYDc3fvW6+c3motqWyDpAWC9Z8PJxUHcn+6o+iMkwSyPNYQYQLohALfEcQ0I/HmCxRj7SraSmcssIzSbvdnvMHspynndi47Tg+Ie66vc2EPVhSf9oMOGfIY9KqzhfIfY2QhG58tFYPusqypoqWW6gKBe4NkdcVMIh0HvirCGID/zdeJ9xyp8EoCN3JwR8v9IwGzZHdz0vDVOcnmcJaiYuiAXtpuQklPJGclN93y/W9UBVciFQsxmdcxD4jjsx9XGHANHHCUiyeFs1Y6SI8/0JcDeeMCK9\";";
    protected VuforiaLocalizer vuforia = null;
    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    protected TFObjectDetector tfod;

    protected AztecRobot robot;

    final void gyroTurn(double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            idle();
        }
    }

    final void gyroSlide(double speed,
                         double distance,
                         IRobotDriver robotDriver) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Set Target and Turn On RUN_TO_POSITION
            robot.setDriveTarget(distance, true);
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            double speedToSet = Math.min(0.2d, speed);
            robot.setDrivePower(speedToSet, speedToSet, speedToSet, speedToSet);
            ElapsedTime driveTime = new ElapsedTime();

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    robot.isDriveBusy()) {
                speedToSet = 0.2d + (driveTime.milliseconds() / 1000d);
                speedToSet = Range.clip(speedToSet, 0d, 1d);
                speedToSet = Math.min(speedToSet, speed);
                robot.setDrivePower(speedToSet, speedToSet, speedToSet, speedToSet);
                if (robotDriver.applyBrake()) {
                    break;
                }
            }

            // Stop all motion;
            robot.setDrivePower(0d, 0d, 0d, 0d);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    final void gyroDrive(double speed,
                         double distance,
                         double angle) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Set Target and Turn On RUN_TO_POSITION
            robot.setDriveTarget(distance, false);
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = 0.2d;
            robot.setDrivePower(speed, speed, speed, speed);
            ElapsedTime driveTime = new ElapsedTime();

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    robot.isDriveBusy()) {
                speed = 0.2d + (driveTime.milliseconds() / 1200d);
                speed = Range.clip(speed, 0d, 1d);

                // adjust relative speed based on heading error.
                double error = getError(angle);
                double steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                double speedFL, speedFR, speedRL, speedRR;

                speedFL = speedRL = speed - steer;
                speedFR = speedRR = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(speedFL), Math.abs(speedFR));
                if (max > 1.0) {
                    speedFL /= max;
                    speedRL /= max;
                    speedFR /= max;
                    speedRR /= max;
                }

                robot.setDrivePower(speedFL, speedFR, speedRL, speedRR);
            }

            // Stop all motion;
            robot.setDrivePower(0d, 0d, 0d, 0d);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private boolean onHeading(double speed, double angle, double PCoeff) {
        double error;
        double steer;
        boolean onTarget = false;
        double speedFL, speedFR, speedRL, speedRR;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            speedFL = speedFR = speedRL = speedRR = 0.0;
            onTarget = true;
        } else {
            steer = getSteer(error, PCoeff);
            speedFR = speedRR = speed * steer;
            speedFL = speedRL = speed * -steer;
        }

        // Send desired speeds to motors.
        robot.setDrivePower(speedFL, speedFR, speedRL, speedRR);

        return onTarget;
    }

    private double getError(double targetAngle) {
        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.getHeading();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    private double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    protected void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    protected void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.6;
        tfodParameters.useObjectTracker = false;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}