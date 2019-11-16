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
        final SkystoneDetector skystoneDetector = new SkystoneDetector(this);
        final DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 29.5d, 100d);
        final ColorDetector colorDetector = new ColorDetector(robot.sensorColor);
        TouchDetector touchDetector = new TouchDetector(robot.digitalLeft);

        int firstSkystoneLocation = 2, secondSkystoneLocation = 4;

        robot.beep();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.addData(">", "Press Play to start op mode");
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
            robotDriver.gyroDrive(1d, 21.3d, 0d, 1.5d, null);

            sleep(20L);

            //start looking for first skystone
            boolean skystoneFound = false;
            //find first skystone from 1 to 3
            for (int i = 0; i < 3; i++) {
                skystoneDetector.reset();
                robotDriver.gyroSlide(0.5d, i == 0 ? 16d : 12.4d, 0d, 1.6d, null);
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
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1.5d, distanceDetector);
            sleep(30L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1.5d, distanceDetector);
            sleep(25L);

            //lower arm
            robotDriver.moveArm(0.4d, 1.7d);

            //close claw
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(260L);

            //move arm back
            new Thread() {
                public void run() {
                    robotDriver.moveArm(0.9d, 1d);
                    robotDriver.moveArm(0.55d, 1d);
                }
            }.start();
            robotDriver.gyroTurn(0.85d, 270d, 1.4d);

            //deliver first skystone under skybridge
            double distanceToSkybridge = 44d + ((firstSkystoneLocation) * 8.9d);
            robotDriver.gyroDrive(1d, distanceToSkybridge, 270d, 3.8d, null);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.gyroDrive(1d, -distanceToSkybridge, 270d, 3.7d, null);

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
                skystoneDetector.reset();
                robotDriver.gyroSlide(0.6d, 33d, 5d, 2d, touchDetector);
                robotDriver.gyroTurn(0.8d, 15d, 0.7d);
                secondSkystoneLocation = 3;
            }
            else {
                //find second skystone from 3 to 5
                for (int i = firstSkystoneLocation; i < 5; i++) {
                    skystoneDetector.reset();
                    robotDriver.gyroSlide(0.5d, 12.1d, 0d, 1.65d, null);
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
            sleep(1200L);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(900L);

            //move arm back
            new Thread() {
                public void run() {
                    robotDriver.moveArm(0.9d, 1d);
                    robotDriver.moveArm(0.55d, 1d);
                }
            }.start();
            robotDriver.gyroTurn(1d, 270d, 0.8d);

            distanceToSkybridge = 53d + ((secondSkystoneLocation) * 8.9d);
            robotDriver.gyroSlide(1d, 4.7d, 270d, 0.8d, null);
            robotDriver.gyroDrive(1d, distanceToSkybridge, 270d, 7d, null);

            robotDriver.setClaw(ClawPosition.OPENED);
            robot.setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            robotDriver.gyroDrive(1d, -12d, 270d, 1d, colorDetector);
        }
    }
}
