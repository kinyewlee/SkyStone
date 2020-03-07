package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous(name = "Skystone: Red Navigation", group = "Skystone")
public class Autonomous_Skystone_Red_Navigation extends LinearOpMode {
    @Override
    public void runOpMode() {
        AztecRobot robot = new AztecRobot(hardwareMap);
        robot.resetDrive();

        RobotDriver robotDriver = new RobotDriver(robot, this);
        DistanceDetector distanceDetector = new DistanceDetector(robot.sensorRange, 18d, 90d);
        ColorDetector colorDetector = new ColorDetector(robot.sensorColor);
        TouchDetector touchDetector = new TouchDetector(robot.digitalLeft);
        ColorDistanceDetector colorDistanceDetector = new ColorDistanceDetector(colorDetector, distanceDetector);

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
            sleep(15000L);

            //move under skybridge
            robotDriver.gyroDrive(0.2d, -3d, 0d, 2d, null);
            robotDriver.gyroTurn(0.3d, 285d, 5d);
            sleep(2000L);

            //if another robot blocking under skybridge stop
            boolean stepFinish = robotDriver.gyroDrive(0.4d, 24d, 270d, 4d, colorDistanceDetector);
            if (!stepFinish && !colorDetector.objectDetected()) {
                robotDriver.gyroSlide(0.4d, -30d, 270d, 3d, null);
            }

            robotDriver.gyroDrive(0.4d, 20d, 270d, 2d, colorDetector);
        }
    }
}
