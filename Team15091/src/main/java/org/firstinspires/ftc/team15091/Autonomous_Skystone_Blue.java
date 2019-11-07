package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Skystone: Blue", group = "Skystone")
public class Autonomous_Skystone_Blue extends LinearOpMode {
    @Override
    public void runOpMode() {
        AztecRobot robot = new AztecRobot(hardwareMap);

        RobotDriver robotDriver = new RobotDriver(robot, this);
        SkystoneDetector skystoneDetector = new SkystoneDetector(this);
        DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 30d, 100d);

        telemetry.addData(">", "Press Play to start op mode");
        Telemetry.Item headingItem = telemetry.addData("Heading: ", "%.4f", robot.getHeading());
        robot.beep();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            headingItem.setValue("%.4f", robot.getHeading());
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
            //robotDriver.setHook(HookPosition.MIDDLE);
            skystoneDetector.reset();
            robotDriver.gyroSlide(0.5d, -60d, 0d, 3d, skystoneDetector);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(100L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);

            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.moveArm(0.4d, 2d);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(300L);
            robotDriver.gyroDrive(0.8d, -8d, 0d, 2d, null);
            robotDriver.setClaw(ClawPosition.SIDE);
            sleep(100L);

            robotDriver.moveArm(2.85d, 2d);

            robotDriver.gyroSlide(0.8d, 24d, 0d, 3d, null);
            robotDriver.setClaw(ClawPosition.OPENED);

            new Thread() {
                public void run() {
                    robotDriver.moveArm(2.6d, 1d);
                    robotDriver.moveArm(2.85d, 1d);
                }
            }.start();

            robotDriver.gyroDrive(0.8d, 8d, 0d, 2d, null);
            skystoneDetector.reset();
            robotDriver.gyroSlide(0.55d, -80d, 0d, 3.5d, skystoneDetector);
            distanceDetector.reset();

            sleep(100L);
            skystoneDetector.objectDetected();
            if (skystoneDetector.stonePosition > 150f) {
                robotDriver.gyroSlide(0.6d, 2d, 0d, 0.5d, null);
            }

            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);
            sleep(100L);
            distanceDetector.reset();
            robotDriver.gyroDrive(0.3d, 4d, 0d, 1d, distanceDetector);

            robotDriver.setClaw(ClawPosition.FRONT);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.moveArm(0.4d, 2d);
            sleep(100L);
            robotDriver.setClaw(ClawPosition.CLOSED);
            sleep(300L);
            robotDriver.gyroDrive(0.8d, -8d, 0d, 2d, null);
            robotDriver.setClaw(ClawPosition.SIDE);
            sleep(100L);

            new Thread() {
                public void run() {
                    robotDriver.moveArm(2.85d, 2d);
                }
            }.start();

            robotDriver.gyroTurn(0.5, 90d, 2d);
            robotDriver.gyroDrive(1d, 24d, 90d, 3d, null);

            robotDriver.moveArm(0.4d, 2d);
            robotDriver.setClaw(ClawPosition.OPENED);
            robotDriver.moveArm(0.5d, 1d);
            robotDriver.gyroDrive(1d, -24d, 90d, 3d, null);

        }
    }
}
