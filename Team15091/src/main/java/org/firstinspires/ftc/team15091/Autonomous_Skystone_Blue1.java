package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Skystone: Blue", group = "Skystone")
public class Autonomous_Skystone_Blue1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        final AztecRobot robot = new AztecRobot(hardwareMap);
        robot.resetDrive();

        final RobotDriver robotDriver = new RobotDriver(robot, this);
        final SkystoneDetector skystoneDetector = new SkystoneDetector(this);
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
                    telemetry.addData("Distance:", distanceDetector.objectDetected());
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
                    robotDriver.moveArm(0.8d, 2d);
                }
            }.start();

            //move forward to start scan first skystone
            robotDriver.gyroDrive(1d, 20d, 0d, 1.5d, null);

            sleep(20L);

            //start looking for first skystone
            boolean skystoneFound = false;
            //find first skystone from 1 to 3
            for (int i = 0; i < 3; i++) {
                skystoneDetector.reset();
                robotDriver.gyroSlide(0.6d, i == 0 ? -14d : -11d, 0d, 1.6d, null);
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

            //move arm back
            new Thread() {
                public void run() {
                    robotDriver.moveArm(0.8d, 1d);
                    robotDriver.moveArm(0.55d, 1d);
                }
            }.start();
            robotDriver.gyroTurn(0.85d, 90d, 1.4d);

            //deliver first skystone under skybridge
            double distanceToSkybridge = 46d + ((firstSkystoneLocation) * 8.9d);
            robotDriver.gyroDrive(1d, distanceToSkybridge, 90d, 3.8d, null);

            new Thread() {
                public void run() {
                    //release skystone
                    robotDriver.setClaw(ClawPosition.SIDE);
                    robotDriver.setClaw(ClawPosition.OPENED);
                    robotDriver.moveArm(0.5d, 1d);
                }
            }.start();

            robotDriver.gyroDrive(1d, -distanceToSkybridge, 90d, 3.7d, null);

            new Thread() {
                public void run() {
                    //pick up 2nd skystone
                    robotDriver.setClaw(ClawPosition.FRONT);
                    robotDriver.setClaw(ClawPosition.OPENED);
                    robotDriver.moveArm(0.7d, 2d);
                }
            }.start();

            robotDriver.gyroTurn(0.8d, 0d, 1.6d);

            //find second skystone
            skystoneFound = false;
            if (firstSkystoneLocation == 2) {

                new Thread() {
                    public void run() {
                        robotDriver.moveArm(0.5d, 1d);
                    }
                }.start();

                skystoneDetector.reset();
                robotDriver.gyroSlide(0.6d, -33d, 0d, 2d, touchDetector);
                secondSkystoneLocation = 4;
            } else {
                //find second skystone from 3 to 5
                for (int i = firstSkystoneLocation; i < 4; i++) {
                    skystoneDetector.reset();
                    robotDriver.gyroSlide(0.6d, -11.2d, 0d, 1.6d, null);
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
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(30L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);

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
            robotDriver.gyroTurn(1d, 70d, 0.6d);

            distanceToSkybridge = 56d + ((secondSkystoneLocation) * 9d);
            robot.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robotDriver.gyroDrive(1d, distanceToSkybridge, 90d, 5.5d, null);

            new Thread() {
                public void run() {
                    //release skystone
                    robotDriver.setClaw(ClawPosition.OPENED);
                    robotDriver.moveArm(0.5d, 1d);
                }
            }.start();

            robotDriver.gyroDrive(1d, -12d, 90d, 1d, colorDetector);
        }
    }
}