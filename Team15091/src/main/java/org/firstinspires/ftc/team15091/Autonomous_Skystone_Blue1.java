package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Skystone: Blue 1", group = "Skystone")
public class Autonomous_Skystone_Blue1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        final AztecRobot robot = new AztecRobot(hardwareMap);
        robot.resetDrive();

        final RobotDriver robotDriver = new RobotDriver(robot, this);
        final SkystoneDetector skystoneDetector = new SkystoneDetector(this, 180L, 500L);
        final DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 29.3d, 100d);
        final ColorDetector colorDetector = new ColorDetector(robot.sensorColor);
        TouchDetector touchDetector = new TouchDetector(robot.digitalRight);

        int firstSkystoneLocation = 2, secondSkystoneLocation = 4;

        robot.beep();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.addData(">", "Press Play to start op mode");
            skystoneDetector.reset();
            skystoneDetector.objectDetected();
            telemetry.addData("Skystone", "mid (%.3f)", skystoneDetector.visibleMidpoint);
            telemetry.addData("Distance", distanceDetector.objectDetected());
            telemetry.addData("Heading", "%.4f", robot.getHeading());
            telemetry.addData("Color", colorDetector.objectDetected());
            telemetry.update();
            idle();
        }

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
            //open claw for pick up
            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.setArmAngle(0.8d, 2d);

            //move forward to start scan first skystone
            robotDriver.gyroDrive(1d, 20d, 0d, 1.5d, null);

            sleep(20L);

            //start looking for first skystone
            boolean skystoneFound = false;
            //find first skystone from 1 to 3
            for (int i = 0; i < 3; i++) {
                skystoneDetector.reset();
                robotDriver.gyroSlide(0.6d, i == 0 ? -14d : -11d, 0d, 1.6d, null);
                robotDriver.gyroTurn(0.7d, 0d, 0.6d);
                for (int j = 0; j < 10; j++) {
                    if (skystoneDetector.objectDetected()) {
                        firstSkystoneLocation = i;
                        skystoneFound = true;
                        break;
                    }
                    sleep(10L);
                }
                if (skystoneFound) break;
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
            skystoneFound = false;
            if (firstSkystoneLocation == 2) {
                robotDriver.setArmAngle(0.5d, 1d);

                skystoneDetector.reset();
                robotDriver.gyroSlide(0.6d, -33d, 0d, 2d, touchDetector);
                //robotDriver.gyroTurn(0.7d, 2d, 0.5d);
                secondSkystoneLocation = 4;
            } else {
                //find second skystone from 3 to 5
                for (int i = firstSkystoneLocation; i < 4; i++) {
                    skystoneDetector.reset();
                    robotDriver.gyroSlide(0.6d, -11.2d, 0d, 1.6d, null);
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

            final double armLiftAngle = firstSkystoneLocation == 2 ? 1d : 0.85d;
            //move arm back
            new Thread(() -> {
                robotDriver.moveArm(armLiftAngle, 1.1d);
                robotDriver.moveArm(0.6d, 1d);
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

        skystoneDetector.dispose();
    }
}
