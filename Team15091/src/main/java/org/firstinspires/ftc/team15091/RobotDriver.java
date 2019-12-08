package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

public class RobotDriver {
    private static final double P_TURN_COEFF = 0.08d;     // Larger is more responsive, but also less stable
    private static final double P_SLIDE_COEFF = 0.07d;     // Larger is more responsive, but also less stable
    private static final double HEADING_THRESHOLD = 1d;      // As tight as we can make it with an integer gyro
    private static final double P_DRIVE_COEFF = 0.09d;     // Larger is more responsive, but also less stable
    private AztecRobot robot;
    private ElapsedTime runtime = new ElapsedTime();
    LinearOpMode opMode;

    public RobotDriver(AztecRobot arobot, LinearOpMode opMode) {
        robot = arobot;
        this.opMode = opMode;
    }

    public final void setHook(HookPosition hookPosition) {
        switch (hookPosition) {
            case UP:
                robot.servoHook.setPosition(0d);
                break;
            case DOWN:
                robot.servoHook.setPosition(1d);
                break;
            case MIDDLE:
                robot.servoHook.setPosition(0.55d);
                break;
        }

    }

    /**
     * Turn robot
     *
     * @param speed
     * @param angle
     */
    final void gyroTurn(double speed, double angle, double timeoutS) {

        runtime.reset();
        // keep looping while we are still active, and not on heading.
        while (opMode.opModeIsActive() &&
                runtime.seconds() < timeoutS &&
                !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
        }

        // Stop all motion;
        robot.setDrivePower(0d, 0d, 0d, 0d);
    }

    final void gyroSlide(double speed,
                         double distance,
                         double angle,
                         double timeoutS,
                         IObjectDetector objectDetector) {

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            // Set Target and Turn On RUN_TO_POSITION
            robot.setDriveTarget(distance, true);
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            double counter = 0d;
            runtime.reset();
            double powerToSet = Math.min(speed, 0.5d);
            robot.setDrivePower(powerToSet, powerToSet, powerToSet, powerToSet);
            ElapsedTime driveTime = new ElapsedTime();

            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    robot.isDriveBusy()) {

//                // adjust relative speed based on heading error.
//                double error = getError(angle);
//                double steer = getSteer(error, P_SLIDE_COEFF);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    steer *= -1.0;
//
//                double speedFL, speedFR, speedRL, speedRR;
//
//                speedFL = powerToSet + steer;
//                speedFR = powerToSet + steer;
//                speedRL = powerToSet - steer;
//                speedRR = powerToSet - steer;
//
//                double max = Math.max(Math.abs(speedFL), Math.abs(speedFR));
//                if (max > 1d) {
//                    speedFL /= max;
//                    speedRL /= max;
//                    speedFR /= max;
//                    speedRR /= max;
//                }

//                robot.setDrivePower(speedFL, speedFR, speedRL, speedRR);

                robot.setDrivePower(powerToSet, powerToSet, powerToSet, powerToSet);
                if (objectDetector != null && objectDetector.objectDetected()) {
                    robot.beep();
                    break;
                }

                if (runtime.milliseconds() > counter && powerToSet < speed) {
                    powerToSet += 0.01d;
                    counter += 20d;
                }
            }

            // Stop all motion;
            robot.setDrivePower(0d, 0d, 0d, 0d);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    final boolean gyroDrive(double speed,
                            double distance,
                            double angle,
                            double timeoutS,
                            IObjectDetector objectDetector) {

        boolean successful = true;
        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            // Set Target and Turn On RUN_TO_POSITION
            robot.setDriveTarget(distance, false);
            robot.setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.setDrivePower(speed, speed, speed, speed);
            // keep looping while we are still active, and BOTH motors are running.
            while (opMode.opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    robot.isDriveBusy()) {

                // adjust relative speed based on heading error.
                double error = getError(angle);
                double steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0d)
                    steer *= -1d;

                double speedFL, speedFR, speedRL, speedRR;

                speedFL = speedRL = speed - steer;
                speedFR = speedRR = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                double max = Math.max(Math.abs(speedFL), Math.abs(speedFR));
                if (max > 1d) {
                    speedFL /= max;
                    speedRL /= max;
                    speedFR /= max;
                    speedRR /= max;
                }
                if (objectDetector != null && objectDetector.objectDetected()) {
                    robot.beep();
                    successful = false;
                    break;
                }
                robot.setDrivePower(speedFL, speedFR, speedRL, speedRR);
            }

            // Stop all motion;
            robot.setDrivePower(0d, 0d, 0d, 0d);

            // Turn off RUN_TO_POSITION
            robot.setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        return successful;
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
        return Range.clip(error * PCoeff, -1d, 1d);
    }

    void moveArm(double armAngle, double timeoutS) {
        runtime.reset();
        if (robot.getArmAngle() < armAngle) {
            robot.setArmPower(1d);
            while (opMode.opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    robot.getArmAngle() < armAngle) {
                robot.setArmPower(1d);
            }
        } else if (robot.getArmAngle() > armAngle) {
            robot.setArmPower(-1d);
            while (opMode.opModeIsActive() &&
                    runtime.seconds() < timeoutS &&
                    robot.getArmAngle() > armAngle) {
                robot.setArmPower(-1d);
            }
        }
        robot.setArmPower(0d);
    }

    void setArmAngle(double armAngle, double timeoutS) {
        new Thread(() -> {
            moveArm(armAngle, timeoutS);
        }).start();
    }

    void setClaw(ClawPosition clawPosition) {
        switch (clawPosition) {
            case CLOSED:
                robot.servoHand.setPosition(0d);
                break;
            case SIDE:
                robot.servoWrist.setPosition(0d);
                break;
            case FRONT:
                robot.servoWrist.setPosition(1d);
                break;
            case OPENED:
                robot.servoHand.setPosition(0.6d);
                break;
        }
    }
}