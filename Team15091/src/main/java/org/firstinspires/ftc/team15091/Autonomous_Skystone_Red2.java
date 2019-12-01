package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;


@Autonomous(name = "Skystone: Red 2", group = "Skystone")
@Disabled
public class Autonomous_Skystone_Red2 extends LinearOpMode {

    int GetSkystonePosition(List<Recognition> skytoneRecognitions) {
        int position = 1;
        float minWidth = 1000L;

        for (Recognition recognition : skytoneRecognitions) {
            //if (recognition.getWidth() < minWidth) {
            //  minWidth = recognition.getWidth();
            float recognitionLeft = Math.max(recognition.getLeft(), 0L);
            float recognitionWidth = recognition.getRight() - recognitionLeft;
            float recognitionMid = recognitionLeft + (recognitionWidth / 2L);
            double angle = recognition.estimateAngleToObject(AngleUnit.DEGREES);

            if (recognitionMid > 530L || angle > 3.8d) {
                position = 2;
            } else if (recognitionMid < 340L || angle < -4d) {
                position = 0;
            } else if (angle < 20L) {
                position = 1;
            }

            telemetry.addData("Skystone", "mid (%.3f) ang (%.3f) %d", recognitionMid, angle, position);
            //}
        }
        return position;
    }

    @Override
    public void runOpMode() {
        final AztecRobot robot = new AztecRobot(hardwareMap);
        robot.resetDrive();

        final RobotDriver robotDriver = new RobotDriver(robot, this);
        final SkystoneDetector skystoneDetector = new SkystoneDetector(this, 0L, 1000L);
        final DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 29.3d, 100d);
        final ColorDetector colorDetector = new ColorDetector(robot.sensorColor);
        TouchDetector touchDetector = new TouchDetector(robot.digitalRight);

        int firstSkystoneLocation = 1, secondSkystoneLocation = 4;

        robot.beep();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.addData("Distance", distanceDetector.objectDetected());
            telemetry.addData("Heading", "%.4f", robot.getHeading());
            telemetry.addData("Color", colorDetector.objectDetected());
            telemetry.update();
            idle();
        }

        new Thread(() -> {
            while (opModeIsActive()) {
                telemetry.addData("Skystone", "mid (%.3f) ang (%.3f)", skystoneDetector.visibleMidpoint, skystoneDetector.visibleAngle);
                telemetry.addData("Distance", distanceDetector.objectDetected());
                telemetry.addData("Heading", "%.4f", robot.getHeading());
                telemetry.addData("Color", colorDetector.objectDetected());
                telemetry.addData("Arm", "%.4f", robot.getArmAngle());
                telemetry.update();
            }
        }).start();

        if (opModeIsActive()) {
            //open claw for pick up
            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.setArmAngle(0.8d, 2d);

            //move forward to start scan first skystone
            robotDriver.gyroDrive(1d, 12d, 0d, 1.5d, null);

            skystoneDetector.objectDetected();
            if (skystoneDetector.visibleMidpoint < 200L || skystoneDetector.visibleAngle < -2d) {
                firstSkystoneLocation = 2;
            } else if (skystoneDetector.visibleMidpoint > 500L || skystoneDetector.visibleAngle > 9d) {
                firstSkystoneLocation = 0;
            }

            robotDriver.gyroDrive(1d, 8d, 0d, 1.5d, null);

            if (firstSkystoneLocation == 0) {
                firstSkystoneLocation = 0;
                robotDriver.gyroSlide(0.6d, -10.7d, 0d, 1.6d, null);
                robotDriver.gyroTurn(0.7d, 0d, 0.6d);

            } else if (firstSkystoneLocation == 2) {
                firstSkystoneLocation = 2;
                robotDriver.gyroSlide(0.6d, 10.7d, 0d, 1.6d, null);
                robotDriver.gyroTurn(0.7d, 0d, 0.6d);
            }

            //adjust to pickup first skystone in front
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 5d, 0d, 1.5d, distanceDetector);
            sleep(30L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 5d, 0d, 1.5d, distanceDetector);

            //pick up skystone
            robotDriver.setArmAngle(0.4d, 2d);
            sleep(1100L);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(900L);

            //move arm back up while turning
            new Thread(() -> {
                robotDriver.moveArm(0.9d, 1d);
                robotDriver.moveArm(0.55d, 1d);
            }).start();
            robotDriver.gyroTurn(0.85d, 270d, 1.4d);

            //deliver first skystone under skybridge
            double distanceToSkybridge = 42.7d + ((firstSkystoneLocation) * 8.9d);
            robotDriver.gyroDrive(1d, distanceToSkybridge, 270d, 3.8d, null);

            //release skystone
            robotDriver.setClaw(ClawPosition.SIDE);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.setArmAngle(0.5d, 1d);

            //go back to look for 2nd skystone
            robotDriver.gyroDrive(1d, -distanceToSkybridge, 270d, 3.6d, null);


            //get ready to pick up
            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.setArmAngle(0.7d, 2d);

            //turn to face skystone side
            robotDriver.gyroTurn(0.75d, 0d, 1.5d);

            //find 2nd skystone
            boolean skystoneFound = false;

            //if the first skytone is on position #3, move directly to the end
            if (firstSkystoneLocation == 2) {

                robotDriver.setArmAngle(0.5d, 1d);

                skystoneDetector.reset();
                robotDriver.gyroSlide(0.6d, 28d, 0d, 2d, touchDetector);
                robotDriver.gyroDrive(0.6d, -2d, 15d, 0.2d, null);
                robotDriver.gyroTurn(0.7d, 12d, 0.5d);
                secondSkystoneLocation = 4;
            } else {
                //find second skystone from 3 to 5
                for (int i = firstSkystoneLocation; i < 4; i++) {
                    skystoneDetector.reset();
                    robotDriver.gyroSlide(0.6d, 10.82d, 0d, 1.6d, null);
                    robotDriver.gyroTurn(0.7d, 0d, 0.5d);
                    for (int j = 0; j < 10; j++) {
                        if (skystoneDetector.objectDetected()) {
                            secondSkystoneLocation = i;
                            skystoneFound = true;
                            break;
                        }
                        sleep(10L);
                    }
                    if (skystoneFound) break;
                }
            }

            //adjust to pickup 2nd skystone in front
            if (distanceDetector.getCurrentDistance() > 26d) {
                distanceDetector.reset();
                robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
                sleep(30L);
                distanceDetector.reset();
                robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            } else if (distanceDetector.getCurrentDistance() < 22d) {
                //move back if too close
                robotDriver.gyroDrive(0.6d, -2d, 0d, 0.2d, null);
            }

            //pick up 2nd skystone
            robotDriver.setArmAngle(0.4d, 2d);
            sleep(1000L);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(900L);

            //move arm back
            new Thread() {
                public void run() {
                    robotDriver.moveArm(0.9d, 1d);
                    robotDriver.moveArm(0.6d, 1d);
                }
            }.start();

            //deliver 2nd skystone
            robotDriver.gyroTurn(1d, 300d, 0.6d);

            distanceToSkybridge = 57.3d + ((secondSkystoneLocation) * 8.95d);
            robot.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            double skybrigdeAngle = secondSkystoneLocation == 4 ? 267d : 269d;
            robotDriver.gyroDrive(1d, distanceToSkybridge, skybrigdeAngle, 6d, null);

            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.setArmAngle(0.5d, 1d);

            //move back to park on line
            robotDriver.gyroDrive(1d, -14d, 270d, 1d, colorDetector);
        }
    }
}
