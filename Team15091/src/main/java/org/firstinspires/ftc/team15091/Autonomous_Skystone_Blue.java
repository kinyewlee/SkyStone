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


        telemetry.addData(">", "Press Play to start op mode");
        Telemetry.Item headingItem = telemetry.addData("Heading: ", "%.4f", robot.getHeading());
        robot.beep();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            headingItem.setValue("%.4f", robot.getHeading());
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
            //move forward
            robotDriver.gyroDrive(0.8d, 17d, 0d, 1d, null);

            //slide right to find skystone
            skystoneDetector.reset();
            robotDriver.gyroSlide(0.48d, -60d, 0d, 3d, skystoneDetector);
            skystoneDetector.objectDetected();
            if (skystoneDetector.stonePosition > 150f) {
                robotDriver.gyroSlide(0.6d, 3d, 0d, 0.5d, null);
            }

            //adjust to pickup skystone in front
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(100L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(100L);

            //open claw for pick up
            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);

            //lower arm
            robotDriver.moveArm(0.4d, 2.5d);

            //close claw
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(300L);

            //drag skystone back and turn it side
            robotDriver.gyroDrive(0.8d, -8d, 0d, 2d, null);
            robotDriver.setClaw(ClawPosition.SIDE);
            sleep(100L);

            //move arm back
            new Thread() {
                public void run() {
                    robotDriver.moveArm(2.85d, 2.5d);
                }
            }.start();
            sleep(1600L);

            //deliver under skybridge
            robotDriver.gyroSlide(0.9d, 60d, 0d, 3d, colorDetector);
            robotDriver.gyroSlide(1d, 10d, 0d, 1d, null);
            robotDriver.setClaw(ClawPosition.OPENED);

            new Thread() {
                public void run() {
                    robotDriver.moveArm(2.6d, 1d);
                    robotDriver.moveArm(2.85d, 1d);
                }
            }.start();

            robotDriver.gyroDrive(0.8d, 8d, 0d, 2d, null);
            robotDriver.gyroSlide(0.9d, -30d, 0d, 1.5d, null);

            //start scanning 2nd skystone
            skystoneDetector.reset();
            robotDriver.gyroSlide(0.55d, -220d, 0d, 5d, skystoneDetector);

            sleep(100L);
            skystoneDetector.objectDetected();
            if (skystoneDetector.stonePosition > 150f) {
                robotDriver.gyroSlide(0.6d, 4d, 0d, 0.5d, null);
            }

            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(100L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);

            //pick up 2nd skystone
            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.moveArm(0.4d, 2.5d);
            sleep(100L);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(300L);
            robotDriver.gyroDrive(0.8d, -8d, 0d, 2d, null);
            robotDriver.setClaw(ClawPosition.SIDE);
            sleep(100L);

            new Thread() {
                public void run() {
                    robotDriver.moveArm(2.85d, 2.5d);
                }
            }.start();

            //turn and park under skybridge
            robotDriver.gyroTurn(0.6, -90d, 2d);
            robotDriver.gyroDrive(0.75d, -70d, -90d, 3d, colorDetector);

            // robotDriver.gyroDrive(1d, -15d, -90d, 1d, null);

            //robotDriver.moveArm(0.4d, 2d);
            //robotDriver.setClaw(ClawPosition.OPENED);
            //robotDriver.moveArm(0.5d, 1d);
            //robotDriver.gyroDrive(1d, -24d, 90d, 3d, colorDetector);

        }
    }
}
