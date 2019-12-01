package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;


@Autonomous(name = "Skystone: Blue 2", group = "Skystone")
@Disabled
public class Autonomous_Skystone_Blue2 extends LinearOpMode {

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
        ElapsedTime timer = new ElapsedTime();
        final AztecRobot robot = new AztecRobot(hardwareMap);
        robot.resetDrive();

        final RobotDriver robotDriver = new RobotDriver(robot, this);
        final SkystoneDetector skystoneDetector = new SkystoneDetector(this, 0L, 1000L);
        final DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 29.3d, 100d);
        final ColorDetector colorDetector = new ColorDetector(robot.sensorColor);
        TouchDetector touchDetector = new TouchDetector(robot.digitalRight);

        int firstSkystoneLocation = 1, secondSkystoneLocation = 4;

        robot.beep();
        int scanCounter = 0;
        timer.reset();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.addData(">", "Press Play to start op mode");
            if (timer.milliseconds() > 1000L) {
                switch (scanCounter) {
                    case 0:
                        skystoneDetector.tfod.setClippingMargins(170, 0, 450, 0);
                        break;
                    case 1:
                        skystoneDetector.tfod.setClippingMargins(320, 0, 290, 0);
                        break;
                    case 2:
                        skystoneDetector.tfod.setClippingMargins(490, 0, 125, 0);
                        break;
                }

                if (scanCounter < 2) {
                    scanCounter++;
                } else {
                    scanCounter = 0;
                }

                timer.reset();
            }

            if (skystoneDetector.objectDetected()) {
                firstSkystoneLocation = GetSkystonePosition(skystoneDetector.skytoneRecognitions);
            }

            telemetry.addData("Skystone", "%d", firstSkystoneLocation);
            telemetry.addData("Distance", distanceDetector.objectDetected());
            telemetry.addData("Heading", "%.4f", robot.getHeading());
            telemetry.addData("Color", colorDetector.objectDetected());
            telemetry.update();
            idle();
        }

        skystoneDetector.tfod.setClippingMargins(0, 0, 0, 0);

        new Thread(() -> {
            while (opModeIsActive()) {
                telemetry.addData("Skystone", "mid (%.3f)", skystoneDetector.visibleMidpoint);
                telemetry.addData("Distance", distanceDetector.objectDetected());
                telemetry.addData("Heading", "%.4f", robot.getHeading());
                telemetry.addData("Color", colorDetector.objectDetected());
                telemetry.addData("Arm", "%.4f", robot.getArmAngle());
                telemetry.update();
            }
        }).start();

        if (opModeIsActive()) {
            //skystoneDetector.objectDetected();

            //open claw for pick up
            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.setArmAngle(0.8d, 2d);

            //move forward to start scan first skystone
            robotDriver.gyroDrive(1d, 20d, 0d, 1.5d, null);

            sleep(20L);

            /* if mid is < 300 then 1st
             * > 400 then 3rd
              else is 2nd */

            if (firstSkystoneLocation == 0) {
                firstSkystoneLocation = 0;
                robotDriver.gyroSlide(0.6d, 10.7d, 0d, 1.6d, null);
                robotDriver.gyroTurn(0.7d, 0d, 0.6d);

            } else if (firstSkystoneLocation == 2) {
                firstSkystoneLocation = 2;
                robotDriver.gyroSlide(0.6d, -10.7d, 0d, 1.6d, null);
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

            //move arm back
            new Thread(() -> {
                robotDriver.moveArm(0.85d, 1d);
                robotDriver.moveArm(0.55d, 1d);
            }).start();
            robotDriver.gyroTurn(0.85d, 90d, 1.4d);

            //deliver first skystone under skybridge
            double distanceToSkybridge = 44d + ((firstSkystoneLocation) * 8.9d);
            robotDriver.gyroDrive(1d, distanceToSkybridge, 90d, 3.8d, null);

            //release skystone
            robotDriver.setClaw(ClawPosition.SIDE);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.setArmAngle(0.5d, 1d);

            //go back to look for 2nd skystone
            robotDriver.gyroDrive(1d, -distanceToSkybridge, 90d, 3.7d, null);

            //pick up 2nd skystone
            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.setArmAngle(0.7d, 2d);

            robotDriver.gyroTurn(0.8d, 0d, 1.6d);

            //find second skystone
            boolean skystoneFound = false;
            if (firstSkystoneLocation == 2) {
                robotDriver.setArmAngle(0.5d, 1d);

                skystoneDetector.reset();
                robotDriver.gyroSlide(0.6d, -33.2d, 0d, 2d, touchDetector);
                secondSkystoneLocation = 4;
            } else {
                //find second skystone from 3 to 5
                for (int i = firstSkystoneLocation; i < 4; i++) {
                    skystoneDetector.reset();
                    robotDriver.gyroSlide(0.6d, -11d, 0d, 1.6d, null);
                    robotDriver.gyroTurn(0.7d, 0d, 0.5d);
                    //try to confirm visual 10 times
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
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(30L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);

            robotDriver.setArmAngle(0.4d, 2d);
            sleep(1000L);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(900L);

            //move arm back
            new Thread(() -> {
                robotDriver.moveArm(0.8d, 1d);
                robotDriver.moveArm(0.55d, 1d);
            }).start();
            robotDriver.gyroTurn(1d, 70d, 0.6d);

            //deliver 2nd skystone
            distanceToSkybridge = 54.6d + ((secondSkystoneLocation) * 9d);
            robot.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robotDriver.gyroDrive(1d, distanceToSkybridge, 90d, 5.5d, null);

            //release skystone
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.setArmAngle(0.5d, 1d);

            //park on the line
            robotDriver.gyroDrive(1d, -14d, 90d, 1d, colorDetector);
        }
    }
}
