package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Skystone: Red Foundation", group = "Skystone")
public class Autonomous_Skystone_Red_Foundation2 extends LinearOpMode {
    @Override
    public void runOpMode() {
        AztecRobot robot = new AztecRobot(hardwareMap);
        robot.resetDrive();

        RobotDriver robotDriver = new RobotDriver(robot, this);
        DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 18d, 90d);
        ColorDetector colorDetector = new ColorDetector(robot.sensorColor);
        TouchDetector touchDetector = new TouchDetector(robot.digitalLeft);
        telemetry.addData(">", "Press Play to start op mode");
        Telemetry.Item headingItem = telemetry.addData("Heading: ", "%.4f", robot.getHeading());
        Telemetry.Item detectorItem = telemetry.addData("Detected: ", distanceDetector.objectDetected());
        robot.beep();

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        while (!(isStarted() || isStopRequested())) {
            headingItem.setValue("%.4f", robot.getHeading());
            detectorItem.setValue(distanceDetector.objectDetected());
            telemetry.update();
            idle();
        }

        if (opModeIsActive()) {
            robotDriver.setHook(HookPosition.MIDDLE);
            robotDriver.holdArm();
            //robotDriver.setArmAngle(2.94d, 1d);

            //move to foundation
            robotDriver.gyroDrive(0.9d, -24d, 0d, 2d, null);
            robotDriver.gyroSlide(0.9d, 20d, 0d, 2d, null);
            robotDriver.gyroDrive(0.2d, -17d, 0d, 3d, null);

            //attach to foundation
            robotDriver.setHook(HookPosition.DOWN);
            sleep(5000L);

            //move foundation to building zone
            robotDriver.gyroDrive(0.9d, 24.5d, 0d, 2d, null);
            robotDriver.gyroSlide(0.9d, -20d, 0d, 2d, null);
            robotDriver.gyroTurn(0.9d, 270d, 2d);
            robotDriver.gyroTurn(0.9d, 270d, 2d);
            robotDriver.gyroTurn(0.9d, 270d, 2d);
            robotDriver.gyroDrive(0.9d, -10d, 270d, 2d, null);

            //detach foundation
            robotDriver.setHook(HookPosition.MIDDLE);
            sleep(200L);

            //move under skybridge
            robotDriver.gyroSlide(0.9d, 28d, 270d, 4d, touchDetector);
            sleep(2000L);

            //if another robot blocking under skybridge stop
            boolean stepFinish = robotDriver.gyroDrive(0.4d, 36d, 270d, 4d, distanceDetector);
            if (!stepFinish) {
                robotDriver.gyroSlide(0.4d, -32d, 270d, 3d, null);
            }

            robotDriver.gyroDrive(0.4d, 20d, 270d, 2d, colorDetector);
        }
    }
}
