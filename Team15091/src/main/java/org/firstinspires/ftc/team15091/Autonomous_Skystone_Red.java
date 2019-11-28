package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Skystone: Red", group = "Skystone")
public class Autonomous_Skystone_Red extends LinearOpMode {
    @Override
    public void runOpMode() {
        final AztecRobot robot = new AztecRobot(hardwareMap);
        robot.resetDrive();

        final RobotDriver robotDriver = new RobotDriver(robot, this);
        final SkystoneDetector skystoneDetector = new SkystoneDetector(this, 180L, 550L);
        final DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 29.3d, 100d);
        final ColorDetector colorDetector = new ColorDetector(robot.sensorColor);
        TouchDetector touchDetector = new TouchDetector(robot.digitalLeft);

        int firstSkystoneLocation = 2, secondSkystoneLocation = 4;

        robot.beep();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.addData(">", "Press Play to start op mode");
            skystoneDetector.reset();
            skystoneDetector.objectDetected();
            telemetry.addData("Skystone:", "mid (%.3f)", skystoneDetector.visibleMidpoint);
            telemetry.addData("Distance:", distanceDetector.objectDetected());
            telemetry.addData("Heading: ", "%.4f", robot.getHeading());
            telemetry.addData("Color:", colorDetector.objectDetected());
            telemetry.update();
            idle();
        }

        new Thread() {
            public void run() {
                while (opModeIsActive()) {
                    telemetry.addData("Skystone:", "mid (%.3f)", skystoneDetector.visibleMidpoint);
                    telemetry.addData("Distance:", "%s (%.3f)", distanceDetector.objectDetected(), distanceDetector.getCurrentDistance());
                    telemetry.addData("Heading: ", "%.4f", robot.getHeading());
                    telemetry.addData("Color:", colorDetector.objectDetected());
                    telemetry.update();
                }
            }
        }.start();

        if (opModeIsActive()) {
            new Thread() {
                public void run() {
                    //open claw for pick up
                    robotDriver.setClaw(ClawPosition.FRONT);
                    robotDriver.setClaw(ClawPosition.OPENED);

                    //lower arm to get ready to pickup
                    robotDriver.moveArm(0.8d, 2d);
                }
            }.start();

            //move forward to start scan first skystone
            robotDriver.gyroDrive(1d, 21.3d, 0d, 1.5d, null);

            sleep(20L);

            //start looking for first skystone
            boolean skystoneFound = false;
            //find first skystone from 1 to 3
            for (int i = 0; i < 3; i++) {
                skystoneDetector.reset();
                robotDriver.gyroSlide(0.6d, i == 0 ? 16d : 12.2d, 0d, 1.6d, null);
                robotDriver.gyroTurn(0.7d, 0d, 0.5d);
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
            new Thread() {
                public void run() {
                    robotDriver.moveArm(0.4d, 2d);
                }
            }.start();
            sleep(1100L);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(900L);

            //move arm back up while turning
            new Thread() {
                public void run() {
                    robotDriver.moveArm(0.9d, 1d);
                    robotDriver.moveArm(0.55d, 1d);
                }
            }.start();
            robotDriver.gyroTurn(0.85d, 270d, 1.4d);

            //deliver first skystone under skybridge
            double distanceToSkybridge = 42.7d + ((firstSkystoneLocation) * 8.9d);
            robotDriver.gyroDrive(1d, distanceToSkybridge, 270d, 3.8d, null);

            //release skystone
            new Thread() {
                public void run() {
                    robotDriver.setClaw(ClawPosition.SIDE);
                    robotDriver.setClaw(ClawPosition.OPENED);
                    robotDriver.moveArm(0.5d, 1d);
                }
            }.start();

            //go back to look for 2nd skystone
            robotDriver.gyroDrive(1d, -distanceToSkybridge, 270d, 3.5d, null);

            new Thread() {
                public void run() {
                    //get ready to pick up
                    robotDriver.setClaw(ClawPosition.FRONT);
                    robotDriver.setClaw(ClawPosition.OPENED);
                    robotDriver.moveArm(0.7d, 2d);
                }
            }.start();

            //turn to face skystone side
            robotDriver.gyroTurn(0.8d, 0d, 1.5d);

            //find 2nd skystone
            skystoneFound = false;

            //if the first skytone is on position #3, move directly to the end
            if (firstSkystoneLocation == 2) {

                new Thread() {
                    public void run() {
                        robotDriver.moveArm(0.5d, 1d);
                    }
                }.start();

                skystoneDetector.reset();
                robotDriver.gyroSlide(0.6d, 31d, 0d, 2d, touchDetector);
                robotDriver.gyroDrive(0.6d, -2d, 15d, 0.2d, null);
                robotDriver.gyroTurn(0.8d, 15d, 0.5d);
                secondSkystoneLocation = 4;
            } else {
                //find second skystone from 3 to 5
                for (int i = firstSkystoneLocation; i < 4; i++) {
                    skystoneDetector.reset();
                    robotDriver.gyroSlide(0.6d, 10.84d, 0d, 1.6d, null);
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
            new Thread() {
                public void run() {
                    robotDriver.moveArm(0.4d, 2d);
                }
            }.start();
            sleep(1000L);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(900L);

            //move arm back
            new Thread() {
                public void run() {
                    robotDriver.moveArm(0.8d, 1d);
                    robotDriver.moveArm(0.55d, 1d);
                }
            }.start();

            //deliver 2nd skystone
            robotDriver.gyroTurn(1d, 300d, 0.6d);

            distanceToSkybridge = 57.4d + ((secondSkystoneLocation) * 8.95d);
            robot.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            double skybrigdeAngle = secondSkystoneLocation == 4 ? 267d : 269d;
            robotDriver.gyroDrive(1d, distanceToSkybridge, skybrigdeAngle, 6d, null);

            new Thread() {
                public void run() {
                    //release skystone
                    robotDriver.setClaw(ClawPosition.OPENED);
                    robotDriver.moveArm(0.5d, 1d);
                }
            }.start();

            //move back to park on line
            robotDriver.gyroDrive(1d, -12d, 270d, 1d, colorDetector);
        }
    }
}
