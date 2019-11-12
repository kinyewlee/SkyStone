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
        final DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 30d, 100d);
        final ColorDetector colorDetector = new ColorDetector(robot.sensorColor);
        int firstSkystoneLocation = 3, secondSkystoneLocation = -1;

        robot.beep();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            telemetry.addData(">", "Press Play to start op mode");
            telemetry.addData("Heading: ", "%.4f", robot.getHeading());
            telemetry.addData("Distance:", distanceDetector.objectDetected());
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
            robotDriver.gyroDrive(1d, 20.7d, 0d, 1.5d, null);

            sleep(30L);

            //start looking for first skystone
            skystoneDetector.reset();
            //find first skystone from 1 to 3
            for (int i = 0; i < 3; i++) {
                robotDriver.gyroSlide(0.6d, i == 0 ? 15.1d : 12.4d, -0.5d, 1.1d, null);
                //robotDriver.gyroTurn(0.7d, 0d, 0.5d);
                boolean skystoneFound = false;
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
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(30L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(25L);

            //lower arm
            robotDriver.moveArm(0.4d, 2d);

            //close claw
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(260L);

            //move arm back
            new Thread() {
                public void run() {
                    robotDriver.moveArm(2.85d, 2.5d);
                }
            }.start();
            sleep(490L);

            //turn it side
            robotDriver.setClaw(ClawPosition.SIDE);

            //deliver first skystone under skybridge
            double distanceToSkybridge = 64d + ((firstSkystoneLocation) * 9d);
            robotDriver.gyroSlide(1d, -distanceToSkybridge, 0d, 4d, null);

            //release first skystone
            new Thread() {
                public void run() {
                    robotDriver.moveArm(1.3d, 2d);
                    robotDriver.moveArm(2.85d, 1.5d);
                }
            }.start();
            sleep(600L);
            robotDriver.setClaw(ClawPosition.OPENED);
            sleep(200L);

            //return to get 2nd skystone
            robotDriver.gyroSlide(1d, distanceToSkybridge - 7d, 2d, 4d, null);
            robotDriver.gyroTurn(0.75d, 0d, 0.7d);

            new Thread() {
                public void run() {
                    //pick up 2nd skystone
                    robotDriver.setClaw(ClawPosition.FRONT);
                    robotDriver.setClaw(ClawPosition.OPENED);
                    robotDriver.moveArm(0.7d, 2d);
                }
            }.start();

            secondSkystoneLocation = firstSkystoneLocation;
            //find second skystone from 3 to 5
            for (int i = firstSkystoneLocation; i < 5; i++) {
                robotDriver.gyroSlide(0.6d, 10.75d, -0.8d, 1d, null);
                boolean skystoneFound = false;
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

            //adjust to pickup 2nd skystone in front
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(30L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(25L);

            robotDriver.moveArm(0.4d, 1.8d);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(280L);

            //move arm back
            new Thread() {
                public void run() {
                    robotDriver.moveArm(2.85d, 3d);
                }
            }.start();
            sleep(150L);

            //turn it side
            robotDriver.setClaw(ClawPosition.SIDE);

            distanceToSkybridge = 60d + (secondSkystoneLocation * 8d);
            //turn and park under skybridge
            robotDriver.gyroTurn(0.7d, 270d, 1.9d);
            robotDriver.gyroSlide(1d, 6d, 270d, 1d, null);
            robotDriver.gyroDrive(1d, distanceToSkybridge, 270d, 7d, null);
        }
    }
}
