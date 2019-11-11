package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Skystone: Blue", group = "Skystone")
public class Autonomous_Skystone_Blue extends LinearOpMode {
    @Override
    public void runOpMode() {
        final AztecRobot robot = new AztecRobot(hardwareMap);

        final RobotDriver robotDriver = new RobotDriver(robot, this);
        final SkystoneDetector skystoneDetector = new SkystoneDetector(this);
        DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 30d, 100d);
        ColorDetector colorDetector = new ColorDetector(robot.sensorColor);
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
                    telemetry.addData("Skystone:", "pos (%.3f)", skystoneDetector.stonePosition);
                    telemetry.addData("Heading: ", "%.4f", robot.getHeading());
                    telemetry.update();
                }
            }
        }.start();

        if (opModeIsActive()) {
            //move forward to start scan skystone
            robotDriver.gyroDrive(0.8d, 16.5d, 0d, 1d, null);
            sleep(30L);

            //start looking for first skystone
            skystoneDetector.reset();
            int firstSkystoneLocation = 0;
            //find first skystone from 1 to 3
            for (int i = 0; i < 3; i++) {
                robotDriver.gyroSlide(0.5d, -11.5d, 0d, 1.5d, null);
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

            //adjust to pickup skystone in front
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(35L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(35L);

            //open claw for pick up
            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);

            //lower arm
            robotDriver.moveArm(0.4d, 2.5d);

            //close claw
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(250L);

            //move arm back
            new Thread() {
                public void run() {
                    robotDriver.moveArm(2.85d, 2.5d);
                }
            }.start();
            sleep(490L);

            //drag turn it side
            robotDriver.setClaw(ClawPosition.SIDE);

            double distanceToSkybridge = 55d + ((firstSkystoneLocation) * 8d);
            //deliver under skybridge
            robotDriver.gyroSlide(0.85d, distanceToSkybridge, 0d, 4d, null);

            new Thread() {
                public void run() {
                    robotDriver.moveArm(2.45d, 1d);
                    robotDriver.moveArm(2.85d, 1d);
                }
            }.start();
            sleep(50L);
            robotDriver.setClaw(ClawPosition.OPENED);

            //return to get 2nd skystone
            robotDriver.gyroSlide(0.78d, -distanceToSkybridge - 3.8d, 0d, 4d, null);
            robotDriver.gyroTurn(0.7d, 0d, 1d);
            sleep(30L);

            int secondSkystoneLocation = 0;
            //find second skystone from 3 to 5
            for (int i = firstSkystoneLocation; i < 5; i++) {
                robotDriver.gyroSlide(0.5d, -10.4d, 0d, 1d, null);
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
            sleep(35L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(35L);

            //pick up 2nd skystone
            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.moveArm(0.4d, 2.5d);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(270L);

            //move arm back
            new Thread() {
                public void run() {
                    robotDriver.moveArm(2.85d, 3d);
                }
            }.start();
            sleep(150L);

            //turn it side
            robotDriver.setClaw(ClawPosition.SIDE);

            distanceToSkybridge = 48d + (secondSkystoneLocation * 8d);
            //turn and park under skybridge
            robotDriver.gyroTurn(0.7d, -90d, 2d);
            robotDriver.gyroDrive(0.81d, -distanceToSkybridge, -90d, 6d, colorDetector);
        }
    }
}
